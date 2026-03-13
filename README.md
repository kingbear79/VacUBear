# VacUBear (ESP-12F)

Dieses Projekt ist auf ESP8266/ESP-12F portiert.

## Build

- PlatformIO Environment: `esp12f`
- Board: `esp12e` (kompatibel fuer ESP-12F)

## FT232RL Programmieradapter

Nur mit **3.3V Pegeln** arbeiten (kein 5V an RX/TX/VCC).

| FT232RL Adapter | ESP-12F Modul | Hinweis |
| --- | --- | --- |
| GND | GND (Pin 15) | Gemeinsame Masse |
| TXD | RXD / GPIO3 (Pin 21) | Gekreuzt |
| RXD | TXD / GPIO1 (Pin 22) | Gekreuzt |
| 3V3 | VCC (Pin 8) | Versorgung 3.3V |

### Boot/Flash Modus

- Flashen (UART Download): `GPIO0 = LOW`, dann Reset/Power-Cycle
- Normaler Start: `GPIO0 = HIGH`, dann Reset/Power-Cycle

Hinweis: `GPIO2` muss beim Boot auf HIGH bleiben, `GPIO15` auf LOW.
