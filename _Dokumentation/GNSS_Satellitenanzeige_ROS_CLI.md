# GNSS-Satellitenanzeige über ROS CLI

Diese Änderung ergänzt für HARDWARE_PLATFORM=1 und OM_GPS_PROTOCOL=UBX eine reine Diagnoseanzeige der vom u-blox Empfänger gemeldeten Satelliten.

## Neues ROS-Topic

```bash
/ll/position/gps/satellites
```

Typ:

```bash
xbot_msgs/GnssSatelliteArray
```

## Anzeige

Nach Build und Start des Containers:

```bash
rostopic echo /ll/position/gps/satellites
```

oder als Tabelle:

```bash
rosrun xbot_driver_gps gps_satellite_list.py
```

## Deaktivieren

Standardmäßig ist die Diagnose aktiv. Sie kann über die Umgebung deaktiviert werden:

```bash
OM_GPS_PUBLISH_SATELLITES=False
```

## Technischer Weg

Der bestehende GPS-Treiber `xbot_driver_gps` liest bereits UBX-Daten vom Empfänger. Ergänzt wurde UBX-NAV-SAT:

```text
u-blox Empfänger
→ /dev/ttyAMA2
→ xbot_driver_gps /ll/services/gps
→ UBX-NAV-SAT Parser
→ /ll/position/gps/satellites
```

Bestehende Topics wie `/ll/position/gps`, `/ll/position/gps/nmea` und `/ll/position/gps/rtcm` bleiben unverändert.
