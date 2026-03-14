# VacUBear (ESP-12F)

Dieses Projekt laeuft auf ESP8266/ESP-12F und steuert Pumpen, Ventil, OTA und die optionale RGBW-Beleuchtung fuer VacUBear.

## Build

- PlatformIO Environment: `esp12f`
- Board: `esp12e` (kompatibel zu ESP-12F)
- Standard-OTA-Manifest: `http://ota.kinkbear.de/master/manifest.json`

## FT232RL Programmieradapter

Nur mit **3.3V Pegeln** arbeiten. Kein 5V an `RX`, `TX` oder `VCC` anlegen.

| FT232RL Adapter | ESP-12F Modul | Hinweis |
| --- | --- | --- |
| GND | GND (Pin 15) | Gemeinsame Masse |
| TXD | RXD / GPIO3 (Pin 21) | Gekreuzt |
| RXD | TXD / GPIO1 (Pin 22) | Gekreuzt |
| 3V3 | VCC (Pin 8) | Versorgung 3.3V |

### Boot/Flash Modus

- Flashen per UART: `GPIO0 = LOW`, danach Reset oder Power-Cycle
- Normaler Start: `GPIO0 = HIGH`, danach Reset oder Power-Cycle

Hinweis: `GPIO2` muss beim Boot auf HIGH bleiben, `GPIO15` auf LOW.

## OTA Deployment

- OTA fuer ESP8266/ESP-12F laeuft bevorzugt ueber `HTTP`, damit kein BearSSL-Speicherproblem mit GitHub Raw entsteht.
- Die Firmware unterscheidet automatisch zwischen `http://...` und `https://...`.
- Bisherige GitHub-Raw-Manifest-URLs werden beim Laden der Konfiguration auf die aktuelle Standard-URL migriert.

### Lokal deployen

Das Skript [deploy_ota_bundle.sh](/Users/christianschweden/Documents/PlatformIO/Projects/VacUBear/deploy_ota_bundle.sh) laedt `manifest.json` und `firmware.bin` per SSH und `rsync` auf den OTA-Server hoch. Der Upload wird komplett lokal gesteuert.

1. Firmware bauen:
   `pio run -e esp12f`
2. OTA-Bundle deployen:
   `./deploy_ota_bundle.sh`

Benoetigte lokale Variablen:

- `OTA_DEPLOY_HOST`: Hostname des OTA-Servers
- `OTA_DEPLOY_PORT`: SSH-Port, meist `22`
- `OTA_DEPLOY_USER`: SSH-Benutzer
- `OTA_DEPLOY_BASE`: Zielverzeichnis auf dem Server, z. B. `/var/www/ota.kinkbear.de`
- `OTA_DEPLOY_BRANCH`: optionales Zielverzeichnis, Standard ist der aktuelle Git-Branch
- `OTA_DEPLOY_SSH_KEY`: optionaler Pfad zu einem SSH-Key, Standard ist `~/.ssh/id_ed25519_ota`, falls vorhanden

Die Variablen koennen direkt in der Shell oder in einer lokalen `.ota-deploy.env` stehen. Diese Datei ist in `.gitignore` hinterlegt und wird nicht versioniert.

Beispiel:

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

## MQTT und Home Assistant

### Basislogik

- Standard-Basis-Topic: `deviceId`
- Die `deviceId` wird aus der MAC-Adresse erzeugt, z. B. `vacubear-AABBCCDDEEFF`
- Telemetrie wird immer auf `tele/<deviceId>/STATE` publiziert
- Home-Assistant-Discovery wird unter `homeassistant/...` publiziert
- Verfuegbarkeit wird auf `<base>/availability` mit `online` und `offline` signalisiert

### Funktionsverhalten der Beleuchtung

- Die HA-Entitaet `Beleuchtung` schaltet die Show-Beleuchtung logisch frei oder sperrt sie.
- Waehren der Show leuchten die LEDs nur, wenn `Beleuchtung = ON` ist.
- Ausserhalb der Show bleiben die LEDs aus.
- Ausnahmen: rotes Pulsieren beim Booten und eine 5-Sekunden-Vorschau nach Aenderung der Lichtfarbe im Webinterface oder per MQTT.
- Der RGB-Farbanteil wird als Farbe gesetzt, der Weiss-Kanal separat als `W`-Wert.

### Steuer- und Status-Topics

| Topic | Richtung | Payload | Funktion |
| --- | --- | --- | --- |
| `<base>/show/set` | zu Geraet | `ON`, `OFF` | Startet oder stoppt die Show |
| `<base>/show/state` | vom Geraet | `ON`, `OFF` | Aktueller Show-Zustand |
| `<base>/light/set` | zu Geraet | `ON`, `OFF` oder JSON | Aktiviert oder deaktiviert die Show-Beleuchtung; JSON kann `state`, `color` und `white_value` enthalten |
| `<base>/light/state` | vom Geraet | `ON`, `OFF` | Gespeicherter Freigabe-Zustand der Show-Beleuchtung |
| `<base>/light/rgbw/set` | zu Geraet | `r,g,b,w` | Setzt die RGBW-Farbwerte direkt |
| `<base>/light/rgbw/state` | vom Geraet | `r,g,b,w` | Aktuell gespeicherte RGBW-Farbwerte |
| `<base>/config/show_length_s/set` | zu Geraet | Ganzzahl in Sekunden | Setzt die Show-Laenge |
| `<base>/config/show_length_s/state` | vom Geraet | Ganzzahl in Sekunden | Aktuell gespeicherte Show-Laenge |
| `<base>/config/nachlauf_s/set` | zu Geraet | Ganzzahl in Sekunden | Setzt die Nachlaufzeit |
| `<base>/config/nachlauf_s/state` | vom Geraet | Ganzzahl in Sekunden | Aktuell gespeicherte Nachlaufzeit |
| `<base>/ota/update/install` | zu Geraet | `install` | Startet ein OTA-Update, wenn bereits ein Update gefunden wurde |
| `<base>/availability` | vom Geraet | `online`, `offline` | MQTT-Verfuegbarkeit des Moduls |
| `tele/<deviceId>/STATE` | vom Geraet | JSON | Sammeltelemetrie fuer HA, Diagnose und Automationen |

### JSON-Payload fuer `<base>/light/set`

Unterstuetzte Felder:

- `state`: `ON` oder `OFF`
- `color.r`: `0..255`
- `color.g`: `0..255`
- `color.b`: `0..255`
- `color.w`: `0..255`
- `white_value`: `0..255`
- `white`: `0..255`

Beispiel:

```json
{
  "state": "ON",
  "color": {"r": 255, "g": 120, "b": 40, "w": 20}
}
```

### Telemetrie auf `tele/<deviceId>/STATE`

Die Telemetrie ist das zentrale Status-JSON und wird zyklisch sowie bei wichtigen Zustandswechseln gesendet.

Top-Level-Felder:

- `UptimeSec`: Laufzeit in Sekunden
- `WiFi`: WLAN-Statusblock
- `Show`: Show-Statusblock
- `Pumpen`: Pumpen-Statusblock
- `Lichtfarbe`: Farb- und Preview-Statusblock
- `LED`: interner LED-Ausgabestatus
- `OTA`: OTA-Statusblock

Detailfelder:

| JSON-Pfad | Bedeutung |
| --- | --- |
| `WiFi.RSSI` | WLAN-Signalstaerke in dBm |
| `WiFi.SSID` | Aktuell verbundene SSID |
| `WiFi.IP` | Aktuelle STA-IP-Adresse des Moduls |
| `WiFi.Host` | Geraete-ID bzw. Hostname |
| `WiFi.Status` | `Connected` oder `Disconnected` |
| `Show.Aktiv` | `true`, wenn die Show gerade laeuft |
| `Show.Status` | `ON` oder `OFF` |
| `Show.Phase` | `Pause`, `Vakuumieren`, `Haltezeit` oder `Belueften` |
| `Show.LaengeMs` | Show-Laenge in Millisekunden |
| `Show.NachlaufMs` | Nachlaufzeit in Millisekunden |
| `Show.LaengeS` | Show-Laenge in Sekunden |
| `Show.NachlaufS` | Nachlaufzeit in Sekunden |
| `Show.EndAt` | Zeitstempel fuer Show-Ende in `millis()` |
| `Show.OpenValveAt` | Zeitstempel fuer das Oeffnen des Ventils in `millis()` |
| `Pumpen.PWM` | Aktuell ausgegebener PWM-Wert |
| `Pumpen.TargetPWM` | Ziel-PWM fuer Softstart/Softstop |
| `Lichtfarbe.Enabled` | Gespeicherte Show-Freigabe fuer die Beleuchtung |
| `Lichtfarbe.R` | Rot-Anteil `0..255` |
| `Lichtfarbe.G` | Gruen-Anteil `0..255` |
| `Lichtfarbe.B` | Blau-Anteil `0..255` |
| `Lichtfarbe.W` | Weiss-Anteil `0..255` |
| `Lichtfarbe.PreviewRemainingMs` | Restdauer einer aktiven 5-Sekunden-Vorschau |
| `LED.Enabled` | Interne Freigabe der Show-Beleuchtung |
| `LED.Count` | Anzahl der LEDs laut Firmware |
| `LED.Level` | Aktuelles Fade-Level |
| `LED.TargetLevel` | Ziel-Level fuer den Soft-Fade |
| `LED.OnUntilAt` | Nachlauf-Zeitfenster nach der Belueftung |
| `LED.PreviewUntilAt` | Zeitstempel, bis zu dem eine Vorschau aktiv bleibt |
| `OTA.CurrentVersion` | Aktuell installierte Firmware-Version |
| `OTA.LatestVersion` | Zuletzt gefundene neueste Version aus dem Manifest |
| `OTA.ReleaseNotes` | Release-Hinweis aus dem Manifest |
| `OTA.UpdateAvailable` | `true`, wenn eine neue Version verfuegbar ist |
| `OTA.CheckInProgress` | `true`, waehrend ein Manifest-Check laeuft |
| `OTA.UpdateInProgress` | `true`, waehrend ein Update heruntergeladen oder geflasht wird |
| `OTA.Phase` | Interne OTA-Phase |
| `OTA.Source` | Quelle des Updates, z. B. `manifest` oder `browser` |
| `OTA.StatusText` | Menschlich lesbarer OTA-Status |
| `OTA.RebootPending` | `true`, wenn nach dem Update ein Neustart ansteht |
| `OTA.ProgressBytes` | Bereits uebertragene Bytes |
| `OTA.ProgressTotal` | Gesamtes erwartetes Volumen |
| `OTA.ProgressPercent` | Prozentwert des Fortschritts |
| `OTA.LastError` | Letzter OTA-Fehlertext |
| `OTA.LastCheckMs` | Zeitpunkt des letzten Checks in `millis()` |
| `OTA.RebootAtMs` | Geplanter Neustart in `millis()` |

### Home-Assistant-Discovery Topics

Diese Topics werden von der Firmware aktiv veroeffentlicht und muessen nicht manuell erstellt werden.

| Topic | Komponente | Zweck |
| --- | --- | --- |
| `homeassistant/switch/<deviceId>/show/config` | `switch` | Show starten und stoppen |
| `homeassistant/light/<deviceId>/light/config` | `light` | Entitaet `Beleuchtung` fuer Freigabe und RGBW-Farbe |
| `homeassistant/update/<deviceId>/firmware/config` | `update` | Firmware-Update in HA |
| `homeassistant/binary_sensor/<deviceId>/show_aktiv/config` | `binary_sensor` | Diagnosesensor fuer Show aktiv, standardmaessig deaktiviert |
| `homeassistant/sensor/<deviceId>/show_phase/config` | `sensor` | Diagnosesensor fuer Show-Phase, standardmaessig deaktiviert |
| `homeassistant/sensor/<deviceId>/ip_adresse/config` | `sensor` | Aktiver Sensor fuer die aktuelle IP-Adresse |
| `homeassistant/sensor/<deviceId>/wifi_rssi/config` | `sensor` | WLAN-Signalstaerke |
| `homeassistant/sensor/<deviceId>/uptime/config` | `sensor` | Laufzeit |
| `homeassistant/sensor/<deviceId>/pumpen_pwm/config` | `sensor` | Diagnosesensor fuer Pumpen-PWM, standardmaessig deaktiviert |
| `homeassistant/sensor/<deviceId>/show_laenge/config` | `sensor` | Diagnosesensor fuer Show-Laenge, standardmaessig deaktiviert |
| `homeassistant/sensor/<deviceId>/nachlaufzeit/config` | `sensor` | Diagnosesensor fuer Nachlaufzeit, standardmaessig deaktiviert |
| `homeassistant/number/<deviceId>/show_laenge_setzen/config` | `number` | Eingabe fuer die Show-Laenge in Sekunden |
| `homeassistant/number/<deviceId>/nachlaufzeit_setzen/config` | `number` | Eingabe fuer die Nachlaufzeit in Sekunden |
| `homeassistant/sensor/<deviceId>/firmware_version/config` | `sensor` | Diagnosesensor fuer installierte Firmware-Version, standardmaessig deaktiviert |
| `homeassistant/sensor/<deviceId>/firmware_verfuegbar/config` | `sensor` | Diagnosesensor fuer im Manifest gefundene Version, standardmaessig deaktiviert |
| `homeassistant/binary_sensor/<deviceId>/update_verfuegbar/config` | `binary_sensor` | Zeigt an, ob ein Update verfuegbar ist |

### Home-Assistant Standardverhalten in diesem Projekt

- Die Update-Entitaet `Firmware` bleibt aktiv, damit verfuegbare Updates in `Geraete und Dienste` angezeigt werden koennen.
- Folgende Diagnose-Entitaeten werden standardmaessig deaktiviert publiziert:
  - `Firmware Verfuegbar`
  - `Firmware Version`
  - `Nachlaufzeit` als Diagnose-Sensor
  - `Pumpen PWM`
  - `Show aktiv`
  - `Show Laenge`
  - `Show Phase`
- Die Entitaet `IP-Adresse` wird aktiv publiziert.
