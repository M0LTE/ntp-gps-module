# ntp-gps-module

ESP32-C3 firmware that stands in for the GPS module on a
[mitxela Precision Clock Mk II½](https://mitxela.com/projects/precision_clock_mk_iii)
(ATtiny2313 running [PrecisionClockMkII](https://github.com/mitxela/PrecisionClockMkII)),
taking time from NTP instead of satellites.

## What it outputs

- **PPS**: 100 ms high pulse, rising edge at the top of each UTC second.
- **NMEA**: `$GPRMC,hhmmss.00,A,,,,,,,ddmmyy,,,A*CS` at 9600 baud, 8N2, sent
  just after the pulse ends. 2 stop bits give the ATtiny (internal RC
  oscillator, no crystal) enough slack to receive reliably.
- Nothing is sent until the first NTP sync. If NTP has not synced for an hour,
  the status flag becomes `V` and the clock blinks as it would without a fix.

The ESP32 clock is disciplined by a small built-in SNTP client (64 s poll,
minimum-delay filter, slews offsets under 128 ms and steps larger ones), and
the PPS edge is placed by spinning on the system clock for the last few ms.

It also learns how fast its crystal runs (a least-squares fit over the last
6 hours of polls, once it has 30 minutes) and corrects for that continuously.
So if Wi-Fi drops, the time holds to roughly tens of milliseconds a day
(temperature permitting) instead of the crystal's raw 1 to 2 seconds a day.
The estimate is saved, so a restart begins with it. The status page shows it.
After a gap of more than about 5 minutes without a sync, old samples are
discarded and it takes a fresh burst, so it corrects within seconds of
getting back online.

## Wiring

| ESP32-C3 | Clock GPS header |
|----------|------------------|
| GPIO4    | TXD (GPS data out → ATtiny RXD) |
| GPIO3    | PPS |
| GND      | GND |
| 5V       | Clock power (optional) |

## Setup

1. On first boot, with no network saved, it opens an open AP
   `NTPClock-Setup-XXXX`; join it and a captive page appears
   (or browse to http://192.168.4.1/).
2. Enter the Wi-Fi SSID/passphrase and NTP server (default `pool.ntp.org`).
3. Afterwards the same page is at the device's IP or http://ntpclock.local/.
   Hold BOOT for 3 s to bring the setup AP back, for example after changing
   the Wi-Fi passphrase. If the saved network goes away it does not open the
   setup AP by itself; it keeps retrying every minute and keeps time on the
   crystal meanwhile.

Wi-Fi TX power is limited to 8.5 dBm, as many small ESP32-C3 boards have a
poorly matched antenna and their AP is invisible at full power.

## Build

```
pio run -t upload
```

(`platformio.ini` targets COM13; change `upload_port` as needed.)
