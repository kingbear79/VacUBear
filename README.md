# VacUBear (ESP-12F)

Dieses Projekt laeuft auf ESP8266/ESP-12F und steuert Pumpen, Ventil, OTA und die optionale RGBW-Beleuchtung fuer VacUBear.

## Dokumentation

- Technische Projekt- und Schnittstellenbeschreibung: [README.md](/Users/christianschweden/Documents/PlatformIO/Projects/VacUBear/README.md)
- Endbenutzer-Handbuch: [docs/ENDUSER_HANDBUCH.md](/Users/christianschweden/Documents/PlatformIO/Projects/VacUBear/docs/ENDUSER_HANDBUCH.md)

## Build

- PlatformIO Environment: `esp12f`
- Board: `esp12e` (kompatibel zu ESP-12F)
- Standard-OTA-Manifest: `http://ota.kingbear.de/vacubear/manifest.json`

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
- `OTA_DEPLOY_BASE`: Basisverzeichnis auf dem Server, z. B. `/var/www/ota.kingbear.de`
- `OTA_DEPLOY_PROJECT`: optionales Projektverzeichnis, Standard ist `vacubear`
- `OTA_DEPLOY_SSH_KEY`: optionaler Pfad zu einem SSH-Key, Standard ist `~/.ssh/id_ed25519_ota`, falls vorhanden

Die Variablen koennen direkt in der Shell oder in einer lokalen `.ota-deploy.env` stehen. Diese Datei ist in `.gitignore` hinterlegt und wird nicht versioniert.

Beispiel:

```bash
OTA_DEPLOY_HOST=ota.kingbear.de
OTA_DEPLOY_PORT=22
OTA_DEPLOY_USER=deploy
OTA_DEPLOY_BASE=/var/www/ota.kingbear.de
OTA_DEPLOY_PROJECT=vacubear
OTA_DEPLOY_SSH_KEY=/Users/christianschweden/.ssh/id_ed25519_ota
```

Das Skript deployed fuer dieses Projekt standardmaessig nach:

- `http://ota.kingbear.de/vacubear/manifest.json`
- `http://ota.kingbear.de/vacubear/firmware.bin`
- `http://ota.kingbear.de/vacubear/index.html`

### Release Notes in `manifest.json`

Der Inhalt von `notes` wird beim Erzeugen des OTA-Bundles in [build_ota_bundle.sh](/Users/christianschweden/Documents/PlatformIO/Projects/VacUBear/build_ota_bundle.sh) gesetzt. `deploy_ota_bundle.sh` uebertraegt diese Information unveraendert mit dem erzeugten `manifest.json` auf den OTA-Server.

Die Aufloesung erfolgt in dieser Reihenfolge:

1. `OTA_NOTES`
   Direkter String aus der Shell. Sinnvoll fuer einmalige Overrides.
2. `OTA_NOTES_FILE`
   Pfad auf eine lokale Text- oder Markdown-Datei.
3. Automatische Dateisuche
   Falls vorhanden, wird eine dieser Dateien geladen:
   - `release_notes/<FW_VERSION>.md`
   - `release_notes/<FW_VERSION>.txt`
   - `release-notes/<FW_VERSION>.md`
   - `release-notes/<FW_VERSION>.txt`
4. Automatische Git-Auswertung
   Wenn keine Datei vorhanden ist, erzeugt das Skript Release Notes aus den Commit-Betreffzeilen seit dem letzten Versionssprung in `platformio.ini`.

Empfohlener Ablauf fuer Releases:

1. `FW_VERSION` in [platformio.ini](/Users/christianschweden/Documents/PlatformIO/Projects/VacUBear/platformio.ini) erhoehen
2. Optional eine kuratierte Datei `release_notes/<FW_VERSION>.md` anlegen
3. `pio run -e esp12f`
4. `./deploy_ota_bundle.sh`

Beispiel fuer ein manuelles Override:

```bash
OTA_NOTES_FILE=release_notes/0.1.3-passionate_gimp.md pio run -e esp12f
./deploy_ota_bundle.sh
```

Wenn keine manuelle Datei existiert, sieht `notes` typischerweise so aus:

```text
Aenderungen seit Version 0.1.2-passionate_gimp:
- Improve Home Assistant OTA update state
- Stage show actuators after LED fades
- Replace NeoPixelBus with minimal SK6812 driver
```

## REST-API

Die REST-API nutzt bewusst die bereits vorhandene `ESP8266WebServer`-Instanz. Es wird keine zusaetzliche HTTP-Bibliothek und kein zweiter Server gestartet. Das haelt den Speicherdruck auf dem ESP8266 niedrig und vermeidet doppelte Logik.

Rahmenbedingungen:

- Format: JSON fuer Anfrage und Antwort
- Schreibende Requests: `POST`
- Lesende Requests: `GET`
- Authentifizierung: derzeit keine
- Passwort-Felder sind in Lese-Responses absichtlich **nicht** im Klartext enthalten. Stattdessen werden nur `password_set`-Flags geliefert.
- Die bisherigen UI-/Legacy-Endpunkte wie `/status` und `/ota/*` bleiben aus Kompatibilitaetsgruenden erhalten.

### REST-Endpunkte

| Endpoint | Methode | Zweck |
| --- | --- | --- |
| `/api/status` | `GET` | Laufzeitstatus des Geraets |
| `/api/config` | `GET` | Aktuelle Konfiguration ohne Klartext-Passwoerter |
| `/api/config` | `POST` | Teilweises Aktualisieren der Konfiguration |
| `/api/show` | `GET` | Aktueller Show-Status |
| `/api/show` | `POST` | Show starten oder stoppen |
| `/api/light` | `GET` | Aktueller Beleuchtungsstatus |
| `/api/light` | `POST` | Beleuchtung konfigurieren und optional Vorschau ausloesen |
| `/api/ota/status` | `GET` | OTA-Status und letzter Update-Check |
| `/api/ota/check` | `POST` | OTA-Manifest-Pruefung anstossen |
| `/api/ota/update` | `POST` | OTA-Installation starten, wenn ein Update verfuegbar ist |

### `GET /api/status`

Liefert den kompakten Laufzeitstatus fuer UIs, Health-Checks oder externe Automationen.

Wichtige Felder:

- `wifi_connected`
- `sta_ip`
- `ap_enabled`
- `show_running`
- `show_length`
- `show_nachlauf`
- `pump_pwm`
- `pump_target_pwm`
- `light_enabled`
- `light_level`
- `light_target_level`
- `light_preview_remaining_ms`
- `ota.*`

Beispiel:

```json
{
  "wifi_connected": true,
  "sta_ip": "192.168.1.42",
  "show_running": false,
  "show_length": 10000,
  "show_nachlauf": 20000,
  "pump_pwm": 0,
  "light_enabled": true,
  "light": {"r": 255, "g": 180, "b": 60, "w": 20},
  "ota": {
    "current_version": "0.1.0-passionate_gimp",
    "latest_version": "0.1.0-passionate_gimp",
    "update_available": false
  }
}
```

### `GET /api/config`

Liefert die aktuell gespeicherte Konfiguration. Passwoerter werden nicht im Klartext zurueckgegeben.

Beispiel:

```json
{
  "device_id": "vacubear-AABBCCDDEEFF",
  "wifi": {
    "ssid": "MeinWLAN",
    "password_set": true
  },
  "mqtt": {
    "host": "192.168.1.10",
    "port": 1883,
    "user": "mqtt-user",
    "password_set": true,
    "topic": "vacubear-AABBCCDDEEFF"
  },
  "ota": {
    "manifest_url": "http://ota.kingbear.de/vacubear/manifest.json"
  },
  "show": {
    "length_ms": 10000,
    "nachlauf_ms": 20000,
    "length_s": 10,
    "nachlauf_s": 20
  },
  "light": {
    "enabled": true,
    "r": 255,
    "g": 255,
    "b": 255,
    "w": 0
  }
}
```

### `POST /api/config`

Aktualisiert Konfigurationswerte partiell. Nicht vorhandene Felder bleiben unveraendert. Die Felder werden nach denselben Regeln wie im Webinterface validiert und begrenzt.

Unterstuetzte Bloecke:

- `wifi.ssid`
- `wifi.password`
- `mqtt.host`
- `mqtt.port`
- `mqtt.user`
- `mqtt.password`
- `mqtt.topic`
- `ota.manifest_url`
- `show.length_ms` oder `show.length_s`
- `show.nachlauf_ms` oder `show.nachlauf_s`
- `light.enabled`
- `light.r`
- `light.g`
- `light.b`
- `light.w`

Wirkung zur Laufzeit:

- WiFi-Aenderungen stossen direkt einen neuen Verbindungsaufbau an
- MQTT-Aenderungen initialisieren die MQTT-Topics neu und triggern einen Reconnect
- Show- und Lichtwerte werden sofort uebernommen
- Eine Farbaenderung ausserhalb der Show startet automatisch die 5-Sekunden-Vorschau

Beispiel-Request:

```json
{
  "mqtt": {
    "host": "192.168.1.20",
    "port": 1883,
    "topic": "vacubear-labor"
  },
  "show": {
    "length_s": 12,
    "nachlauf_s": 25
  },
  "light": {
    "enabled": true,
    "r": 255,
    "g": 120,
    "b": 40,
    "w": 30
  }
}
```

Beispiel-Response:

```json
{
  "ok": true,
  "wifi_changed": false,
  "mqtt_changed": true,
  "light_changed": true,
  "config": {
    "device_id": "vacubear-AABBCCDDEEFF"
  }
}
```

### `GET /api/show`

Liefert den aktuellen Show-Zustand mit Phase und Zeitstempeln.

Wichtige Felder:

- `running`
- `phase`
- `length_ms`
- `nachlauf_ms`
- `fade_in_done_at_ms`
- `end_at_ms`
- `open_valve_at_ms`
- `finish_at_ms`

### `POST /api/show`

Steuert die Show direkt.

Request:

```json
{
  "state": "ON"
}
```

Zulaessige Werte:

- `ON`
- `OFF`

Hinweis: Der Start ist intern asynchron. Die Firmware setzt ein Start-Flag und uebernimmt die Zeitberechnung im naechsten `loop()`-Durchlauf.

### `GET /api/light`

Liefert den aktuellen Beleuchtungsstatus.

Wichtige Felder:

- `enabled`
- `preview_remaining_ms`
- `show_running`
- `color.r`
- `color.g`
- `color.b`
- `color.w`

### `POST /api/light`

Unterstuetzt sowohl `application/json` als auch `application/x-www-form-urlencoded`, damit derselbe Endpunkt vom Webinterface und von externen Clients genutzt werden kann.

Unterstuetzte Felder:

- `enabled`
- `r`
- `g`
- `b`
- `w`
- `preview`

Beispiel:

```json
{
  "enabled": true,
  "r": 255,
  "g": 180,
  "b": 80,
  "w": 20,
  "preview": true
}
```

Verhalten:

- `enabled` schaltet die Show-Beleuchtung logisch frei oder aus
- RGBW-Werte aktualisieren die gespeicherte Beleuchtung
- `preview = true` startet eine 5-Sekunden-Vorschau ausserhalb einer laufenden Show

### `GET /api/ota/status`

Liefert den aktuellen OTA-Status.

Wichtige Felder:

- `configured`
- `manifest_url`
- `current_version`
- `latest_version`
- `firmware_url`
- `release_notes`
- `update_available`
- `check_in_progress`
- `update_in_progress`
- `phase`
- `status_text`
- `progress_bytes`
- `progress_total`
- `progress_percent`
- `last_error`

### `POST /api/ota/check`

Stoesst einen Manifest-Check an. Der Request queued den Check nur. Der eigentliche Ablauf passiert asynchron in `loop()`.

Erfolgsantwort:

```json
{
  "ok": true,
  "queued": true
}
```

Typische Fehlerbedingungen:

- OTA-Manifest-URL fehlt
- WLAN nicht verbunden
- bereits laufender OTA-Check oder Update

### `POST /api/ota/update`

Startet die Installation eines bereits gefundenen Updates. Auch hier wird der Vorgang nur in die Firmware-Loop eingereiht.

Voraussetzungen:

- WLAN verbunden
- keine laufende Show
- kein paralleler OTA-Vorgang
- `update_available = true`

### Speicher- und Sicherheitsbewertung

- Die REST-API ist auf dem vorhandenen Webserver implementiert und benoetigt keine weitere Netzwerkschicht.
- JSON-Dokumente bleiben bewusst klein und thematisch getrennt, um Heap und Fragmentierung zu begrenzen.
- Im letzten Build liegt der RAM-Verbrauch bei `55260 / 81920`, Flash bei `550339 / 1044464`.
- Die API ist derzeit ungeschuetzt. Sie sollte deshalb nur in vertrauenswuerdigen Netzen oder hinter einer vorgeschalteten Absicherung genutzt werden.

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
- Home Assistant steuert die Lichtfarbe bewusst nur als `RGB`, damit kein kuehl wirkender Weiss-Anteil in die Farbe eingemischt wird.
- Der Weiss-Kanal bleibt separat ueber Webinterface, REST und das direkte MQTT-RGBW-Topic konfigurierbar.

### Steuer- und Status-Topics

| Topic | Richtung | Payload | Funktion |
| --- | --- | --- | --- |
| `<base>/show/set` | zu Geraet | `ON`, `OFF` | Startet oder stoppt die Show |
| `<base>/show/state` | vom Geraet | `ON`, `OFF` | Aktueller Show-Zustand |
| `<base>/light/set` | zu Geraet | `ON`, `OFF` oder JSON | Aktiviert oder deaktiviert die Show-Beleuchtung; JSON kann `state`, `color` und `white_value` enthalten |
| `<base>/light/state` | vom Geraet | `ON`, `OFF` | Gespeicherter Freigabe-Zustand der Show-Beleuchtung |
| `<base>/light/rgb/set` | zu Geraet | `r,g,b` | Setzt die RGB-Farbwerte direkt; wird von Home Assistant fuer die Lichtfarbe verwendet |
| `<base>/light/rgb/state` | vom Geraet | `r,g,b` | Aktuell gespeicherte RGB-Farbwerte |
| `<base>/light/rgbw/set` | zu Geraet | `r,g,b,w` | Setzt die RGBW-Farbwerte direkt |
| `<base>/light/rgbw/state` | vom Geraet | `r,g,b,w` | Aktuell gespeicherte RGBW-Farbwerte |
| `<base>/config/show_length_s/set` | zu Geraet | Ganzzahl in Sekunden | Setzt die Show-Laenge |
| `<base>/config/show_length_s/state` | vom Geraet | Ganzzahl in Sekunden | Aktuell gespeicherte Show-Laenge |
| `<base>/config/nachlauf_s/set` | zu Geraet | Ganzzahl in Sekunden | Setzt die Nachlaufzeit |
| `<base>/config/nachlauf_s/state` | vom Geraet | Ganzzahl in Sekunden | Aktuell gespeicherte Nachlaufzeit |
| `<base>/ota/update/install` | zu Geraet | `install` | Startet ein OTA-Update, wenn bereits ein Update gefunden wurde |
| `<base>/ota/update/state` | vom Geraet | JSON | Retainter OTA-Zustand fuer Home Assistant (`installed_version`, `latest_version`, `in_progress`, `update_percentage`) |
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
| `Show.Phase` | `Pause`, `FadeIn`, `Vakuumieren`, `Haltezeit` oder `FadeOut` |
| `Show.LaengeMs` | Show-Laenge in Millisekunden |
| `Show.NachlaufMs` | Nachlaufzeit in Millisekunden |
| `Show.LaengeS` | Show-Laenge in Sekunden |
| `Show.NachlaufS` | Nachlaufzeit in Sekunden |
| `Show.FadeInDoneAt` | Zeitstempel, ab dem Pumpen und Ventil umgeschaltet werden duerfen |
| `Show.EndAt` | Zeitstempel fuer das Ende der Vakuumierphase in `millis()` |
| `Show.OpenValveAt` | Zeitstempel fuer das Oeffnen des Ventils in `millis()` |
| `Show.FinishAt` | Zeitstempel fuer das Ende der Show nach dem FadeOut |
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
| `homeassistant/light/<deviceId>/light/config` | `light` | Entitaet `Beleuchtung` fuer Freigabe und RGB-Farbe; Weiss bleibt separat |
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
