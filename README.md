# ntp-gps-module

ESP32-C3 firmware that stands in for the GPS module on a
[mitxela Precision Clock Mk II½](https://mitxela.com/projects/precision_clock_mk_iii)
(ATtiny2313 running [PrecisionClockMkII](https://github.com/mitxela/PrecisionClockMkII)),
taking time from NTP instead of satellites.

## What it outputs

- **PPS** – 100 ms high pulse, rising edge at the top of each UTC second.
- **NMEA** – `$GPRMC,hhmmss.00,A,,,,,,,ddmmyy,,,A*CS` at 9600 baud, 8N2, sent
  just after the pulse ends. 2 stop bits give the ATtiny (internal RC
  oscillator, no crystal) enough slack to receive reliably.
- Nothing is sent until the first NTP sync. If NTP has not synced for an hour,
  the status flag becomes `V` and the clock blinks as it would without a fix.

The ESP32 clock is disciplined by a small built-in SNTP client (64 s poll,
minimum-delay filter, slews offsets under 128 ms and steps larger ones), and
the PPS edge is placed by spinning on the system clock for the last few ms.

## Wiring

| ESP32-C3 | Clock GPS header |
|----------|------------------|
| GPIO4    | TXD (GPS data out → ATtiny RXD) |
| GPIO3    | PPS |
| GND      | GND |
| 5V       | Clock power (optional) |

## Setup

1. On first boot (or when the saved network can't be joined) it opens an
   open AP `NTPClock-Setup-XXXX`; join it and a captive page appears
   (or browse to http://192.168.4.1/).
2. Enter the Wi-Fi SSID/passphrase and NTP server (default `pool.ntp.org`).
3. Afterwards the same page is at the device's IP or http://ntpclock.local/.
   Hold BOOT for 3 s to bring the setup AP back.

Wi-Fi TX power is limited to 8.5 dBm – many small ESP32-C3 boards have a
poorly matched antenna and their AP is invisible at full power.

## Build

```
pio run -t upload
```

(`platformio.ini` targets COM13; change `upload_port` as needed.)
