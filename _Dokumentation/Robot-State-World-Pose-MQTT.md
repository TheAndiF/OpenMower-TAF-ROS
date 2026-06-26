# Robot State: reale Weltkoordinate per MQTT

## Ziel

`robot_state/json` enthält zusätzlich zur lokalen OpenMower-Position `pose.x`/`pose.y` jetzt auch eine reale Weltkoordinate im WGS84-System.

## MQTT-Struktur

Topic:

```text
robot_state/json
```

Neuer Block im Payload:

```json
"world_pose": {
  "valid": true,
  "coordinate_system": "WGS84",
  "source": "robot_pose_to_wgs84",
  "latitude": 52.2057601,
  "longitude": 13.0761302,
  "altitude": 0.0,
  "pos_accuracy": 0.03
}
```

Bei fehlender Umrechnungsgrundlage:

```json
"world_pose": {
  "valid": false,
  "coordinate_system": "WGS84",
  "source": "robot_pose_to_wgs84",
  "reason": "gps_datum_unavailable"
}
```

## Datenquelle

Die Koordinate wird nicht direkt aus einem rohen NMEA-Satz übernommen. Sie wird aus der Roboterposition berechnet, die OpenMower intern für Navigation und Karte verwendet:

```text
robot_state.pose.x / robot_state.pose.y
+ /ll/services/gps/datum_lat
+ /ll/services/gps/datum_long
+ /ll/services/gps/datum_height
→ WGS84 latitude / longitude / altitude
```

Damit ist `world_pose` synchron zur bestehenden OpenMower-Position in `robot_state/json`.

## Geänderte Dateien

```text
src/lib/xbot_monitoring/src/xbot_monitoring.cpp
src/lib/xbot_monitoring/CMakeLists.txt
src/lib/xbot_monitoring/package.xml
src/lib/xbot_monitoring/README.md
```

## Prüfung auf dem Roboter

```bash
mosquitto_sub -h localhost -t 'robot_state/json' -v
```

Mit externem MQTT-Prefix:

```bash
mosquitto_sub -h localhost -t '<prefix>/robot_state/json' -v
```

Erwartet wird ein zusätzlicher Block `world_pose`. Bei `valid: true` können `latitude` und `longitude` direkt in einer Karte angezeigt werden.
