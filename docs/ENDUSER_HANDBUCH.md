# VacUBear Endbenutzer-Handbuch

## Zweck dieses Handbuchs

Dieses Handbuch richtet sich an Anwenderinnen und Anwender, die VacUBear benutzen, ohne sich tief mit Elektronik oder Programmierung befassen zu wollen.

Es erklaert:

- den Taster am Geraet
- das Webinterface
- Firmware-Updates
- die REST-API
- MQTT und Home Assistant

Technische Funktionen wie REST, MQTT und Home Assistant sind in eigenen Abschnitten beschrieben. Wer VacUBear nur bedienen will, kann diese Kapitel einfach ueberspringen.

## Kurzueberblick

VacUBear ist ein netzwerkfaehiges Modul auf Basis eines ESP-12F/ESP8266. Das Geraet kann:

- eine Show starten und stoppen
- Pumpen und Ventil steuern
- eine Beleuchtung waehrend der Show ansteuern
- per Webinterface konfiguriert werden
- per MQTT und Home Assistant eingebunden werden
- Firmware-Updates ueber das Netzwerk erhalten

## Wichtige Begriffe

- `Show`: der eigentliche Ablauf des Geraets
- `Beleuchtung`: die LED-Beleuchtung waehrend der Show
- `AP-Modus`: das Geraet stellt ein eigenes WLAN bereit, damit es neu konfiguriert werden kann
- `OTA-Update`: ein Firmware-Update ueber das Netzwerk

## Bedienung am Geraet

Das Modul besitzt einen Hardware-Taster. Je nach Druckdauer wird eine andere Funktion ausgeloest.

### Kurzer Tastendruck

Ein kurzer Tastendruck startet oder stoppt die Show.

- Wenn die Show aus ist: Show startet
- Wenn die Show laeuft: Show stoppt

### Langer Tastendruck von mindestens 3 Sekunden

Ein Tastendruck von mindestens 3 Sekunden, aber kuerzer als 10 Sekunden, schaltet die Beleuchtung waehrend der Show um.

Wichtig:

- diese Funktion ist nur sinnvoll, wenn eine Show laeuft
- das Geraet bestaetigt die Aenderung optisch

LED-Bestaetigung:

- `3x gruen blinken`: Beleuchtung waehrend der Show ist jetzt aktiv
- `3x rot blinken`: Beleuchtung waehrend der Show ist jetzt deaktiviert

### Sehr langer Tastendruck von mindestens 10 Sekunden

Ein Tastendruck von mindestens 10 Sekunden versetzt das Geraet in den AP-Modus.

Das ist sinnvoll, wenn sich das WLAN geaendert hat und das Modul sich nicht mehr mit dem bisherigen Netzwerk verbinden kann.

Merkmale:

- das Geraet startet ein eigenes WLAN
- die LED pulsiert rot
- ueber dieses WLAN kann das Webinterface zur Neueinrichtung geoeffnet werden

Die bisher gespeicherten Einstellungen werden dabei nicht automatisch geloescht.

## LED-Signale

Die LED dient nicht nur als Beleuchtung, sondern auch als Statusanzeige.

### Rotes Pulsieren beim Start

Nach dem Einschalten pulsiert die LED rot, waehrend das Geraet startet und versucht, sich mit dem WLAN zu verbinden.

### Rotes Pulsieren im AP-Modus

Wenn das Geraet im AP-Modus ist, pulsiert die LED ebenfalls rot.

### Gruenes oder rotes Blinken nach 3-Sekunden-Tastendruck

- `Gruen`: Beleuchtung wurde aktiviert
- `Rot`: Beleuchtung wurde deaktiviert

### Vorschau der eingestellten Lichtfarbe

Wenn die Lichtfarbe im Webinterface, per REST oder per MQTT geaendert wird, leuchtet die LED fuer einige Sekunden mit der neuen Farbe. So kann die Einstellung direkt kontrolliert werden.

## Verbindung mit dem Geraet

VacUBear kann auf zwei Arten erreichbar sein:

### Normalbetrieb im vorhandenen WLAN

Wenn das Geraet erfolgreich mit dem konfigurierten WLAN verbunden ist, ist das Webinterface ueber die IP-Adresse des Geraets erreichbar.

Die aktuelle IP-Adresse kann in Home Assistant angezeigt werden, wenn die Integration aktiv ist.

### AP-Modus

Wenn noch keine WLAN-Daten gespeichert sind oder der AP-Modus per Taster aktiviert wurde, stellt das Geraet ein eigenes WLAN bereit.

In diesem Fall:

- mit dem VacUBear-WLAN verbinden
- das Webinterface im Browser oeffnen
- neue WLAN- und MQTT-Daten eintragen

## Das Webinterface

Das Webinterface ist die einfachste Moeglichkeit, das Geraet zu konfigurieren.

### Was im Webinterface eingestellt werden kann

- WLAN-Name und WLAN-Passwort
- MQTT-Server und MQTT-Zugangsdaten
- Basis-Topic fuer MQTT
- URL fuer das OTA-Manifest
- Show-Laenge
- Nachlaufzeit
- Beleuchtung waehrend der Show
- Lichtfarbe
- Weiss-Anteil der LED

### WLAN-Auswahl

Im Webinterface wird eine Liste der verfuegbaren WLAN-Netze angezeigt.

Funktionen:

- Auswahl eines gefundenen WLANs aus der Liste
- Uebernahme der ausgewaehlten SSID in das Eingabefeld
- Aktualisierung der Netzwerkliste per Button

### Beleuchtung

Im Bereich `Beleuchtung` kann festgelegt werden:

- ob die Beleuchtung waehrend der Show aktiv sein soll
- welche Farbe verwendet wird
- wie hoch der Weiss-Anteil ist

Hinweis:

- ausserhalb der Show bleibt die Beleuchtung aus
- ausgenommen sind Boot-Anzeige, AP-Anzeige und die kurze Farbvorschau

### OTA-Firmware

Im Bereich `OTA Firmware` kann:

- nach neuer Firmware gesucht werden
- ein Update gestartet werden
- alternativ eine lokale Firmware-Datei per Browser hochgeladen werden

Waehren eines Online-Updates kann das Geraet zeitweise nicht auf Statusanfragen reagieren. Das ist normal. Das Geraet darf in dieser Phase nicht ausgeschaltet oder manuell neu gestartet werden.

### Lokaler Firmware-Upload

Zusatzfunktion im Webinterface:

- eine `.bin`-Datei vom eigenen Rechner auswaehlen
- direkt im Browser hochladen
- das Geraet installiert die Firmware anschliessend selbst

Das ist besonders praktisch fuer manuelle Tests oder wenn eine Datei lokal vorliegt.

## Firmware-Updates

VacUBear unterstuetzt OTA-Updates.

### So funktioniert ein OTA-Update

1. Das Geraet prueft ein Manifest
2. Wenn eine neuere Version verfuegbar ist, wird die Firmware-URL uebernommen
3. Das Update kann gestartet werden
4. Die Firmware wird geladen und installiert
5. Danach startet das Geraet neu

### Wichtige Hinweise

- waehrend des Updates nicht vom Strom trennen
- waehrend des Updates keinen manuellen Reset ausloesen
- bei langsamer Rueckmeldung im Webinterface abwarten

## REST-API

Die REST-API ist fuer fortgeschrittene Nutzer gedacht. Sie eignet sich fuer eigene Apps, Automationen oder Tools.

Alle Endpunkte sind direkt im Geraet integriert und liefern JSON zurueck.

### Wofuer die REST-API geeignet ist

- Status abfragen
- Konfiguration automatisiert setzen
- Show starten oder stoppen
- Beleuchtung setzen
- OTA-Status lesen und Updates ausloesen

### Wichtige REST-Endpunkte

| Endpunkt | Methode | Zweck |
| --- | --- | --- |
| `/api/status` | `GET` | allgemeiner Geraetestatus |
| `/api/config` | `GET` | aktuelle Konfiguration lesen |
| `/api/config` | `POST` | Konfiguration teilweise aendern |
| `/api/show` | `GET` | Show-Zustand lesen |
| `/api/show` | `POST` | Show starten oder stoppen |
| `/api/light` | `GET` | Lichtstatus lesen |
| `/api/light` | `POST` | Lichtwerte setzen |
| `/api/ota/status` | `GET` | OTA-Status lesen |
| `/api/ota/check` | `POST` | nach neuer Firmware suchen |
| `/api/ota/update` | `POST` | Firmware-Update starten |
| `/api/wifi/scan` | `GET` | verfuegbare WLAN-Netze scannen |

### Beispiel: Show starten

Anfrage:

```http
POST /api/show
Content-Type: application/json
```

```json
{
  "state": "ON"
}
```

### Beispiel: Licht setzen

```http
POST /api/light
Content-Type: application/json
```

```json
{
  "enabled": true,
  "r": 255,
  "g": 120,
  "b": 40,
  "w": 20,
  "preview": true
}
```

### REST fuer technische Laien

Wenn keine eigene Software oder Automation gebaut werden soll, wird die REST-API normalerweise nicht benoetigt. Fuer den Alltag reichen Webinterface und Home Assistant in der Regel voellig aus.

## MQTT

MQTT ist ein leichtgewichtiges Nachrichtenprotokoll fuer Smart-Home-Systeme und Automationen.

VacUBear kann:

- Befehle per MQTT empfangen
- Status per MQTT veroeffentlichen
- Home Assistant automatisch per Discovery einrichten

### Grundidee

- Topics mit `/set` schicken Befehle an das Geraet
- Topics mit `/state` liefern den aktuellen Zustand

### Wichtige MQTT-Topics

| Topic | Richtung | Bedeutung |
| --- | --- | --- |
| `<base>/show/set` | an VacUBear | Show starten oder stoppen |
| `<base>/show/state` | von VacUBear | Show-Zustand |
| `<base>/light/set` | an VacUBear | Beleuchtung logisch ein- oder ausschalten |
| `<base>/light/state` | von VacUBear | gespeicherter Beleuchtungszustand |
| `<base>/light/rgb/set` | an VacUBear | RGB-Farbe fuer Home Assistant |
| `<base>/light/rgb/state` | von VacUBear | aktuell gespeicherte RGB-Farbe |
| `<base>/light/rgbw/set` | an VacUBear | direkte RGBW-Werte |
| `<base>/light/rgbw/state` | von VacUBear | aktuell gespeicherte RGBW-Werte |
| `<base>/config/show_length_s/set` | an VacUBear | Show-Laenge in Sekunden |
| `<base>/config/nachlauf_s/set` | an VacUBear | Nachlaufzeit in Sekunden |
| `<base>/availability` | von VacUBear | `online` oder `offline` |
| `tele/<deviceId>/STATE` | von VacUBear | Sammelstatus fuer Telemetrie |

### Beispiele

Show starten:

```text
Topic: <base>/show/set
Payload: ON
```

Show stoppen:

```text
Topic: <base>/show/set
Payload: OFF
```

RGB-Farbe setzen:

```text
Topic: <base>/light/rgb/set
Payload: 255,120,40
```

RGBW direkt setzen:

```text
Topic: <base>/light/rgbw/set
Payload: 255,120,40,20
```

## Home Assistant

Home Assistant ist die komfortabelste Smart-Home-Integration fuer VacUBear.

Die Firmware veroeffentlicht ihre Discovery-Daten selbst. Dadurch erscheinen die wichtigsten Entitaeten automatisch in Home Assistant.

### Typische Home-Assistant-Funktionen

- Show starten und stoppen
- Beleuchtung aktivieren oder deaktivieren
- Lichtfarbe setzen
- OTA-Update verfuegbar erkennen
- IP-Adresse des Moduls sehen
- Diagnosewerte bei Bedarf aktivieren

### Welche Entitaeten standardmaessig ausgeblendet sein koennen

Einige Diagnose-Entitaeten werden absichtlich standardmaessig deaktiviert angezeigt, damit die Oberflaeche nicht ueberladen wird.

Dazu gehoeren zum Beispiel:

- Firmware verfuegbar
- Firmware Version
- Nachlaufzeit
- Pumpen PWM
- Show aktiv
- Show Laenge
- Show Phase

Diese Entitaeten koennen in Home Assistant bei Bedarf manuell aktiviert werden.

### Licht in Home Assistant

Die Entitaet `Beleuchtung` wird in Home Assistant bewusst als `RGB` bereitgestellt.

Grund:

- die Farbdarstellung bleibt so naeher an der gewaehlten Monitorfarbe
- ein unbeabsichtigter Weiss-Anteil wird vermieden

Der separate Weisskanal bleibt weiterhin ueber Webinterface, REST und direktes MQTT verfuegbar.

### Firmware-Update in Home Assistant

Wenn die OTA-Konfiguration korrekt eingerichtet ist, kann Home Assistant ein verfuegbares Firmware-Update erkennen und in `Geraete und Dienste` anzeigen.

## Typische Anwendungsfaelle

### Fall 1: WLAN wurde geaendert

1. Taster mindestens 10 Sekunden druecken
2. Warten, bis die LED rot pulsiert
3. Mit dem VacUBear-WLAN verbinden
4. Webinterface oeffnen
5. Neue WLAN-Daten eintragen
6. Speichern

### Fall 2: Beleuchtung waehrend der Show schnell abschalten

1. Waehren der Show Taster mindestens 3 Sekunden halten
2. Auf LED-Bestaetigung warten
3. `3x rot` bedeutet: Beleuchtung aus

### Fall 3: Neue Firmware aus dem Browser hochladen

1. Webinterface oeffnen
2. Bereich `OTA Firmware` aufrufen
3. Firmware-Datei `.bin` auswaehlen
4. Upload starten
5. Geraet waehrenddessen nicht ausschalten

## Fehlerbehebung

### Das Geraet verbindet sich nicht mehr mit dem WLAN

Moegliche Ursache:

- WLAN-Name oder Passwort hat sich geaendert

Loesung:

- AP-Modus per 10-Sekunden-Tastendruck aktivieren
- neue Daten im Webinterface eintragen

### Das Webinterface ist nicht erreichbar

Pruefen:

- ist das Geraet im gleichen Netzwerk
- ist die IP-Adresse korrekt
- ist das Geraet im AP-Modus

### Die Show startet nicht

Pruefen:

- ob ein kurzer Tastendruck verwendet wurde
- ob in Home Assistant oder per MQTT versehentlich ein Stopp-Befehl gesendet wurde

### Die Beleuchtung leuchtet nicht

Pruefen:

- ist `Beleuchtung` aktiviert
- laeuft gerade eine Show
- wurde die Funktion per 3-Sekunden-Tastendruck deaktiviert

### OTA-Update reagiert traege

Das kann normal sein. Waehren Online-Update und Neustart beantwortet das Geraet nicht immer sofort alle Statusabfragen.

Wichtig:

- Geduld haben
- nicht ausschalten
- keinen manuellen Reset ausloesen

## Zusammenfassung

Fuer den normalen Alltag reichen meist drei Dinge:

1. kurzer Tastendruck fuer Start oder Stopp der Show
2. 3-Sekunden-Tastendruck zum Umschalten der Beleuchtung waehrend der Show
3. 10-Sekunden-Tastendruck, um bei Netzwerkproblemen den AP-Modus zu starten

Fuer die Einrichtung und Wartung ist das Webinterface die einfachste Stelle. REST, MQTT und Home Assistant sind Zusatzwege fuer Automatisierung und Smart-Home-Integration.
