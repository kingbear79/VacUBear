# VacUBear (ESP-12F)

Dieses Projekt ist auf ESP8266/ESP-12F portiert.

## Build

- PlatformIO Environment: `esp12f`
- Board: `esp12e` (kompatibel fuer ESP-12F)
- Standard-OTA-Manifest: `http://ota.kinkbear.de/master/manifest.json`

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

## OTA Deployment

- OTA fuer ESP8266/ESP-12F laeuft jetzt auch ueber `HTTP`, damit kein BearSSL-Speicherproblem mit GitHub Raw entsteht.
- Die Firmware unterscheidet automatisch zwischen `http://...` und `https://...`.
- Bisherige GitHub-Raw-Manifest-URLs werden beim Laden der Konfiguration auf die neue Standard-URL migriert.

### Lokal deployen

Das Skript [deploy_ota_bundle.sh](/Users/christianschweden/Documents/PlatformIO/Projects/VacUBear/deploy_ota_bundle.sh) laedt `manifest.json` und `firmware.bin` per SSH/`rsync` auf den OTA-Server hoch. Der Upload wird komplett lokal gesteuert.

Vorgehen:

1. Firmware bauen:
   `pio run -e esp12f`
2. OTA-Bundle lokal deployen:
   `./deploy_ota_bundle.sh`

Benötigte lokale Variablen:

- `OTA_DEPLOY_HOST`: Hostname des OTA-Servers
- `OTA_DEPLOY_PORT`: SSH-Port, meist `22`
- `OTA_DEPLOY_USER`: SSH-Benutzer
- `OTA_DEPLOY_BASE`: Zielverzeichnis auf dem Server, z. B. `/var/www/ota.kinkbear.de`
- `OTA_DEPLOY_BRANCH`: optionales Zielverzeichnis, Standard ist der aktuelle Git-Branch
- `OTA_DEPLOY_SSH_KEY`: optionaler Pfad zu einem SSH-Key, Standard ist `~/.ssh/id_ed25519_ota`, falls vorhanden

Die Variablen koennen entweder direkt in der Shell gesetzt werden oder in einer lokalen Datei `.ota-deploy.env` liegen. Diese Datei ist in `.gitignore` hinterlegt und wird nicht versioniert.

Beispiel fuer `.ota-deploy.env`:

```bash
OTA_DEPLOY_HOST=ota.kinkbear.de
OTA_DEPLOY_PORT=22
OTA_DEPLOY_USER=deploy
OTA_DEPLOY_BASE=/var/www/ota.kinkbear.de
OTA_DEPLOY_BRANCH=master
OTA_DEPLOY_SSH_KEY=/Users/christianschweden/.ssh/id_ed25519_ota
```

Das Skript deployed branchbezogen nach:

- `http://ota.kinkbear.de/<branch>/manifest.json`
- `http://ota.kinkbear.de/<branch>/firmware.bin`
