# OpenMower-TAF-ROS - Kurzdokumentation - F9P-Neustart per MQTT unter gps_state - v0.1

Stand: 2026-07-08

Diese Änderung ergänzt eine MQTT-/ROS-Kette, mit der ein u-blox/ZED-F9P über UBX-CFG-RST aus der GPS-State-Oberfläche heraus neu gestartet werden kann.

## MQTT-Topics

- `gps_state/restart/set/json` - nimmt den Neustartbefehl entgegen.
- `gps_state/restart/status/json` - retained Status des letzten Requests beziehungsweise der letzten Treibermeldung.
- `gps_state/restart/validation/json` - Validierung des letzten MQTT-Kommandos.
- `gps_state/restart/set/renew/json` - Status und GPS-State-Metadaten erneut veröffentlichen.
- `gps_state/settings/json` - enthält zusätzlich die Gruppe `restart` mit dem Command-Descriptor `f9p_restart`.

## Befehle

```json
{"mode":"hot_start"}
{"mode":"warm_start"}
{"mode":"cold_start"}
```

Optional kann `reset_mode` angegeben werden:

```json
{"mode":"hot_start","reset_mode":"controlled_software"}
{"mode":"hot_start","reset_mode":"gnss_only"}
{"mode":"cold_start","reset_mode":"hardware_watchdog"}
```

Empfohlen ist `controlled_software`, weil das dem dokumentierten kontrollierten Software-Reset per UBX-CFG-RST entspricht. Der GPS-Treiber erwartet intern einen `std_msgs/String` in der Form `<mode>:<reset_mode>`, zum Beispiel `hot_start:controlled_software`.

## Datenfluss

MQTT `gps_state/restart/set/json` -> `xbot_monitoring` validiert und normalisiert -> ROS `/ll/position/gps/restart_request` -> `xbot_driver_gps` sendet UBX-CFG-RST -> ROS `/ll/position/gps/restart_status` -> MQTT `gps_state/restart/status/json`.

## Hinweise

Der Treiber wartet nicht auf ein ACK, da der Empfänger je nach Firmware direkt resetten kann. Die Funktion ist nur aktiv, wenn `xbot_driver_gps` im UBX-Modus läuft. Bei NMEA-Betrieb wird der Request zurückgewiesen.
