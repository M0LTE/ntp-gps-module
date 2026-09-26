// NTP -> GPS emulator for the mitxela Precision Clock Mk II / Mk II½ (rev 3b).
//
// The clock's ATtiny2313 firmware (github.com/mitxela/PrecisionClockMkII)
// wants, from its GPS header:
//   * UART 9600 8N1, "$G?RMC,hhmmss.ss,A,<7 fields>,ddmmyy,..." (UTC).
//     Status 'A' = valid; anything else makes the display blink.
//   * PPS: rising edge on INT0 at the top of each UTC second. The RMC
//     sentence that follows the edge carries the time of that second.
//
// This firmware keeps the ESP32 system clock disciplined with NTP and emits
// both signals. Wi-Fi is configured through a setup access point / web UI.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <sys/time.h>
#include <driver/gpio.h>

// ---- Pins -------------------------------------------------------------------
static const int NMEA_TX_PIN = 4;   // -> clock GPS data (PD0 / RXD)
static const int PPS_PIN = 3;       // -> clock PPS (PD2 / INT0)
static const int BOOT_BTN_PIN = 9;  // hold 3 s to open the setup AP

// ---- Tunables ---------------------------------------------------------------
static const char *HOSTNAME = "ntpclock";
static const char *DEFAULT_NTP = "pool.ntp.org";
static const uint32_t PPS_WIDTH_MS = 100;
static const uint32_t POLL_INTERVAL_MS = 64000;
static const int IBURST_SAMPLES = 4;
static const int FILTER_SIZE = 4;
static const int64_t STEP_THRESHOLD_US = 128000;
static const uint32_t STALE_AFTER_MS = 60UL * 60 * 1000;  // report 'V' after 1h without sync
static const uint32_t STA_CONNECT_TIMEOUT_MS = 30000;
static const uint32_t AP_OFF_AFTER_CONNECT_MS = 120000;

// ---- State ------------------------------------------------------------------
static Preferences prefs;
static WebServer server(80);
static DNSServer dns;
static WiFiUDP udp;

static String cfgSsid, cfgPass, cfgNtp;
static bool apActive = false;
static uint32_t staConnectedSince = 0;
static uint32_t staLostSince = 0;
static String apSsid;
static String scanOptions;

static volatile bool everSynced = false;
static volatile uint32_t lastSyncMs = 0;
static int64_t lastOffsetUs = 0, lastDelayUs = 0;
static String lastNtpError = "not yet polled";
static IPAddress lastNtpIp;

// ---- Time helpers -----------------------------------------------------------
static int64_t nowUs() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  return (int64_t)tv.tv_sec * 1000000LL + tv.tv_usec;
}

static const uint64_t NTP_UNIX_DELTA = 2208988800ULL;

static uint64_t usToNtp(int64_t us) {
  uint64_t sec = (uint64_t)(us / 1000000) + NTP_UNIX_DELTA;
  uint64_t frac = ((uint64_t)(us % 1000000) << 32) / 1000000;
  return (sec << 32) | (frac & 0xffffffffULL);
}

static int64_t ntpToUs(uint64_t n) {
  uint64_t sec = n >> 32;
  if (sec < 0x80000000ULL) sec += 0x100000000ULL;  // NTP era 1 (after 2036)
  int64_t us = (int64_t)(((n & 0xffffffffULL) * 1000000ULL) >> 32);
  return ((int64_t)sec - (int64_t)NTP_UNIX_DELTA) * 1000000LL + us;
}

static uint64_t rd64(const uint8_t *p) {
  uint64_t v = 0;
  for (int i = 0; i < 8; i++) v = (v << 8) | p[i];
  return v;
}

static void wr64(uint8_t *p, uint64_t v) {
  for (int i = 7; i >= 0; i--) { p[i] = v & 0xff; v >>= 8; }
}

static bool timeValid() {
  return everSynced && (millis() - lastSyncMs) < STALE_AFTER_MS;
}

// ---- PPS + NMEA output ------------------------------------------------------
static void sendRmc(time_t sec, bool valid) {
  struct tm t;
  gmtime_r(&sec, &t);
  char body[80];
  snprintf(body, sizeof body, "GPRMC,%02d%02d%02d.00,%c,,,,,,,%02d%02d%02d,,,%c",
           t.tm_hour, t.tm_min, t.tm_sec, valid ? 'A' : 'V',
           t.tm_mday, t.tm_mon + 1, t.tm_year % 100, valid ? 'A' : 'N');
  uint8_t cs = 0;
  for (const char *p = body; *p; p++) cs ^= (uint8_t)*p;
  char line[96];
  int n = snprintf(line, sizeof line, "$%s*%02X\r\n", body, cs);
  Serial1.write((const uint8_t *)line, n);
}

// Highest-priority task: sleep until ~3 ms before the second boundary, then
// spin on the system clock so the edge lands within a few microseconds.
static void ppsTask(void *) {
  for (;;) {
    if (!everSynced) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }

    int64_t now = nowUs();
    int64_t next = (now / 1000000 + 1) * 1000000;
    int64_t wait = next - now;
    if (wait > 3000) {
      uint32_t ms = (uint32_t)((wait - 3000) / 1000);
      vTaskDelay(pdMS_TO_TICKS(ms ? ms : 1));
      continue;
    }

    bool aborted = false;
    for (;;) {
      now = nowUs();
      if (now >= next) break;
      if (next - now > 5000) { aborted = true; break; }  // clock stepped back
    }
    if (aborted || now - next > 2000) continue;          // clock stepped forward

    // Send the RMC after the pulse, as mitxela's nmea.py does: the ATtiny is
    // busy in its PPS/rollover interrupts right after the edge and its UART
    // only buffers two bytes, so characters sent then get dropped.
    gpio_set_level((gpio_num_t)PPS_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(PPS_WIDTH_MS));
    gpio_set_level((gpio_num_t)PPS_PIN, 0);
    sendRmc((time_t)(next / 1000000), timeValid());
  }
}

// ---- NTP client -------------------------------------------------------------
struct NtpSample {
  int64_t offsetUs;
  int64_t delayUs;
  int64_t appliedAtSample;  // value of totalAppliedUs when the sample was taken
};

static NtpSample filter[FILTER_SIZE];
static int filterCount = 0, filterHead = 0;
static int64_t totalAppliedUs = 0;
static uint32_t lastPollMs = 0;
static bool needIburst = true;

static bool ntpQuery(const IPAddress &ip, NtpSample &out) {
  uint8_t pkt[48] = {0};
  pkt[0] = 0x23;  // LI=0, VN=4, Mode=3 (client)

  while (udp.parsePacket() > 0) udp.flush();  // drop stale replies

  int64_t t1 = nowUs();
  uint64_t t1n = usToNtp(t1);
  wr64(pkt + 40, t1n);
  udp.beginPacket(ip, 123);
  udp.write(pkt, sizeof pkt);
  udp.endPacket();

  uint32_t start = millis();
  int64_t t4 = 0;
  for (;;) {
    if (udp.parsePacket() >= 48) { t4 = nowUs(); break; }
    if (millis() - start > 1500) { lastNtpError = "timeout"; return false; }
    taskYIELD();
  }

  uint8_t r[48];
  udp.read(r, sizeof r);
  uint8_t li = r[0] >> 6, mode = r[0] & 7, stratum = r[1];
  if (mode != 4 || li == 3 || stratum == 0 || stratum > 15) {
    lastNtpError = "bad reply (li=" + String(li) + " stratum=" + String(stratum) + ")";
    return false;
  }
  if (rd64(r + 24) != t1n) { lastNtpError = "originate mismatch"; return false; }

  int64_t t2 = ntpToUs(rd64(r + 32));
  int64_t t3 = ntpToUs(rd64(r + 40));
  out.offsetUs = ((t2 - t1) + (t3 - t4)) / 2;
  out.delayUs = (t4 - t1) - (t3 - t2);
  out.appliedAtSample = totalAppliedUs;
  return true;
}

static void clearFilter() { filterCount = 0; filterHead = 0; }

static void applyFilter() {
  // Pick the minimum-delay sample; correct its offset for adjustments made since.
  int best = -1;
  for (int i = 0; i < filterCount; i++)
    if (best < 0 || filter[i].delayUs < filter[best].delayUs) best = i;
  if (best < 0) return;

  int64_t off = filter[best].offsetUs - (totalAppliedUs - filter[best].appliedAtSample);
  lastOffsetUs = off;
  lastDelayUs = filter[best].delayUs;

  if (!everSynced || off > STEP_THRESHOLD_US || off < -STEP_THRESHOLD_US) {
    int64_t t = nowUs() + off;
    struct timeval tv = {(time_t)(t / 1000000), (suseconds_t)(t % 1000000)};
    settimeofday(&tv, nullptr);
    clearFilter();
    Serial.printf("NTP: stepped clock by %lld us\n", off);
  } else {
    struct timeval tv = {(time_t)(off / 1000000), (suseconds_t)(off % 1000000)};
    adjtime(&tv, nullptr);  // slews at 1/64, so <= 128 ms finishes in ~8 s
    totalAppliedUs += off;
    Serial.printf("NTP: slewing %lld us (delay %lld us)\n", off, lastDelayUs);
  }
  everSynced = true;
  lastSyncMs = millis();
}

static void ntpPoll() {
  IPAddress ip;
  if (!WiFi.hostByName(cfgNtp.c_str(), ip)) { lastNtpError = "DNS lookup failed"; return; }
  lastNtpIp = ip;

  int samples = needIburst ? IBURST_SAMPLES : 1;
  int got = 0;
  for (int i = 0; i < samples; i++) {
    if (i) { delay(2000); server.handleClient(); }
    NtpSample s;
    if (ntpQuery(ip, s)) {
      filter[filterHead] = s;
      filterHead = (filterHead + 1) % FILTER_SIZE;
      if (filterCount < FILTER_SIZE) filterCount++;
      got++;
    }
  }
  if (got) {
    lastNtpError = "";
    needIburst = false;
    applyFilter();
  }
}

// ---- Config -----------------------------------------------------------------
static void loadConfig() {
  prefs.begin("ntpgps", true);
  cfgSsid = prefs.getString("ssid", "");
  cfgPass = prefs.getString("pass", "");
  cfgNtp = prefs.getString("ntp", DEFAULT_NTP);
  prefs.end();
  if (cfgNtp.isEmpty()) cfgNtp = DEFAULT_NTP;
}

static void saveConfig() {
  prefs.begin("ntpgps", false);
  prefs.putString("ssid", cfgSsid);
  prefs.putString("pass", cfgPass);
  prefs.putString("ntp", cfgNtp);
  prefs.end();
}

// ---- Wi-Fi ------------------------------------------------------------------
static String htmlEscape(const String &s) {
  String o;
  for (char c : s) {
    switch (c) {
      case '&': o += "&amp;"; break;
      case '<': o += "&lt;"; break;
      case '>': o += "&gt;"; break;
      case '"': o += "&quot;"; break;
      default: o += c;
    }
  }
  return o;
}

static void scanNetworks() {
  int n = WiFi.scanNetworks();
  scanOptions = "";
  for (int i = 0; i < n; i++) {
    String s = WiFi.SSID(i);
    if (s.isEmpty() || scanOptions.indexOf("value=\"" + htmlEscape(s) + "\"") >= 0) continue;
    scanOptions += "<option value=\"" + htmlEscape(s) + "\">";
  }
  WiFi.scanDelete();
}

static void startAp() {
  if (apActive) return;
  WiFi.mode(WIFI_AP_STA);
  scanNetworks();
  WiFi.softAP(apSsid.c_str());
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  dns.start(53, "*", WiFi.softAPIP());
  apActive = true;
  Serial.printf("Setup AP '%s' up at http://%s/\n", apSsid.c_str(),
                WiFi.softAPIP().toString().c_str());
}

static void stopAp() {
  if (!apActive) return;
  dns.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  apActive = false;
  Serial.println("Setup AP stopped");
}

static void connectSta() {
  if (cfgSsid.isEmpty()) return;
  WiFi.disconnect();
  WiFi.begin(cfgSsid.c_str(), cfgPass.c_str());
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  staLostSince = millis();
  staConnectedSince = 0;
  Serial.printf("Joining '%s'\n", cfgSsid.c_str());
}

// ---- Web UI -----------------------------------------------------------------
static const char PAGE_HEAD[] PROGMEM = R"(<!doctype html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>NTP Clock Source</title><style>
body{font-family:system-ui,sans-serif;max-width:32em;margin:0 auto;padding:1em;background:#f6f6f4;color:#222}
h1{font-size:1.3em}fieldset{border:1px solid #ccc;border-radius:8px;margin:1em 0;padding:1em;background:#fff}
label{display:block;margin:.6em 0 .2em;font-weight:600}input{width:100%;box-sizing:border-box;padding:.5em;font-size:1em}
button{margin-top:1em;padding:.6em 1.2em;font-size:1em}table{width:100%;border-collapse:collapse}
td{padding:.25em 0;vertical-align:top}td:first-child{color:#666;width:40%}.ok{color:#080}.bad{color:#b00}
</style></head><body><h1>NTP &rarr; GPS clock source</h1>)";

static String statusTable() {
  String s = "<fieldset><legend>Status</legend><table>";
  wl_status_t ws = WiFi.status();
  s += "<tr><td>Wi-Fi</td><td>";
  if (ws == WL_CONNECTED)
    s += "<span class=ok>connected</span> to " + htmlEscape(WiFi.SSID()) + "<br>IP " +
         WiFi.localIP().toString() + " (" + String(WiFi.RSSI()) + " dBm)<br>http://" + HOSTNAME + ".local/";
  else if (cfgSsid.isEmpty())
    s += "<span class=bad>not configured</span>";
  else
    s += "<span class=bad>joining " + htmlEscape(cfgSsid) + "&hellip;</span>";
  s += "</td></tr><tr><td>NTP server</td><td>" + htmlEscape(cfgNtp);
  if (lastNtpIp) s += " (" + lastNtpIp.toString() + ")";
  s += "</td></tr><tr><td>Sync</td><td>";
  if (everSynced) {
    s += String(timeValid() ? "<span class=ok>OK</span>" : "<span class=bad>stale</span>") +
         ", " + String((millis() - lastSyncMs) / 1000) + " s ago<br>offset " +
         String(lastOffsetUs / 1000.0, 2) + " ms, delay " + String(lastDelayUs / 1000.0, 2) + " ms";
  } else {
    s += "<span class=bad>never</span>";
  }
  if (!lastNtpError.isEmpty()) s += "<br>last poll: " + htmlEscape(lastNtpError);
  s += "</td></tr><tr><td>UTC</td><td>";
  if (everSynced) {
    time_t t = time(nullptr);
    char buf[32];
    strftime(buf, sizeof buf, "%Y-%m-%d %H:%M:%S", gmtime(&t));
    s += buf;
  } else {
    s += "&ndash;";
  }
  s += "</td></tr><tr><td>Output</td><td>" +
       String(!everSynced ? "none (waiting for first sync)" : timeValid() ? "PPS + RMC (valid)" : "PPS + RMC (flagged invalid)") +
       "</td></tr></table></fieldset>";
  return s;
}

static void handleRoot() {
  String p = FPSTR(PAGE_HEAD);
  p += statusTable();
  p += "<form method=post action=/save><fieldset><legend>Wi-Fi</legend>"
       "<label for=ssid>Network name</label><input id=ssid name=ssid list=nets autocomplete=off value=\"" +
       htmlEscape(cfgSsid) + "\"><datalist id=nets>" + scanOptions + "</datalist>"
       "<label for=pass>Passphrase</label><input id=pass name=pass type=password placeholder=\"" +
       String(cfgPass.isEmpty() ? "" : "(unchanged)") + "\">"
       "</fieldset><fieldset><legend>Time</legend><label for=ntp>NTP server</label>"
       "<input id=ntp name=ntp value=\"" + htmlEscape(cfgNtp) + "\"></fieldset>"
       "<button type=submit>Save</button></form>"
       "<p><a href=/>Refresh status</a> &middot; <a href=/scan>Rescan networks</a></p></body></html>";
  server.send(200, "text/html", p);
}

static void handleSave() {
  String ssid = server.arg("ssid"), pass = server.arg("pass"), ntp = server.arg("ntp");
  ssid.trim();
  ntp.trim();
  if (ntp.isEmpty()) ntp = DEFAULT_NTP;

  bool wifiChanged = ssid != cfgSsid || !pass.isEmpty();
  bool ntpChanged = ntp != cfgNtp;
  cfgSsid = ssid;
  if (!pass.isEmpty() || ssid.isEmpty()) cfgPass = pass;
  cfgNtp = ntp;
  saveConfig();

  String p = FPSTR(PAGE_HEAD);
  p += "<p>Saved.</p>";
  if (wifiChanged && !cfgSsid.isEmpty())
    p += "<p>Joining <b>" + htmlEscape(cfgSsid) + "</b>. This setup network stays up for two minutes "
         "after it connects; <a href=/>refresh</a> to see the address it gets. Afterwards use http://" +
         String(HOSTNAME) + ".local/ or that IP.</p>";
  p += "<p><a href=/>Back</a></p></body></html>";
  server.send(200, "text/html", p);

  if (ntpChanged) { needIburst = true; lastPollMs = 0; lastNtpIp = IPAddress(); }
  if (wifiChanged) { delay(500); connectSta(); }
}

static void handleScan() {
  scanNetworks();
  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "");
}

static void handleNotFound() {
  // Captive portal: bounce everything else to the setup page.
  server.sendHeader("Location", String("http://") +
                    (apActive ? WiFi.softAPIP().toString() : WiFi.localIP().toString()) + "/");
  server.send(302, "text/plain", "");
}

// ---- Main -------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  gpio_reset_pin((gpio_num_t)PPS_PIN);
  gpio_set_direction((gpio_num_t)PPS_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)PPS_PIN, 0);
  // 2 stop bits: extra slack for the ATtiny, which runs on its internal RC
  // oscillator and may be a few percent off 8 MHz.
  Serial1.begin(9600, SERIAL_8N2, -1, NMEA_TX_PIN);
  pinMode(BOOT_BTN_PIN, INPUT_PULLUP);

  loadConfig();

  uint8_t mac[6];
  WiFi.macAddress(mac);
  char buf[32];
  snprintf(buf, sizeof buf, "NTPClock-Setup-%02X%02X", mac[4], mac[5]);
  apSsid = buf;

  WiFi.persistent(false);
  WiFi.setHostname(HOSTNAME);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // modem sleep adds jitter to NTP round trips
  // Many small C3 boards have a poorly matched antenna; full power distorts
  // so badly that the AP is invisible. 8.5 dBm is the well-known workaround.
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.setAutoReconnect(true);

  if (cfgSsid.isEmpty()) startAp();
  else connectSta();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/scan", HTTP_GET, handleScan);
  server.onNotFound(handleNotFound);
  server.begin();

  udp.begin(12300);
  xTaskCreate(ppsTask, "pps", 4096, nullptr, configMAX_PRIORITIES - 1, nullptr);
}

void loop() {
  if (apActive) dns.processNextRequest();
  server.handleClient();

  bool connected = WiFi.status() == WL_CONNECTED;
  uint32_t now = millis();

  if (connected) {
    if (!staConnectedSince) {
      staConnectedSince = now;
      Serial.printf("Connected, IP %s\n", WiFi.localIP().toString().c_str());
      MDNS.end();
      if (MDNS.begin(HOSTNAME)) MDNS.addService("http", "tcp", 80);
    }
    staLostSince = 0;
    if (apActive && now - staConnectedSince > AP_OFF_AFTER_CONNECT_MS) stopAp();
    if (!lastPollMs || now - lastPollMs >= POLL_INTERVAL_MS) {
      lastPollMs = now;
      ntpPoll();
    }
  } else {
    staConnectedSince = 0;
    if (!staLostSince) staLostSince = now;
    if (!apActive && now - staLostSince > STA_CONNECT_TIMEOUT_MS) startAp();
  }

  // Hold BOOT for 3 s to bring the setup AP back.
  static uint32_t btnDown = 0;
  if (digitalRead(BOOT_BTN_PIN) == LOW) {
    if (!btnDown) btnDown = now;
    else if (now - btnDown > 3000 && !apActive) { startAp(); staConnectedSince = connected ? now : 0; }
  } else {
    btnDown = 0;
  }

  delay(2);
}
