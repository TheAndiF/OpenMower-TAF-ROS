System sensor display update
============================

Changes:
- openmower_system_monitor publishes system sensor values every 5 seconds (publish_rate_hz=0.2) instead of every 30 seconds.
- openmower_system_monitor publishes three initial samples shortly after startup to reduce the dashboard default-value/0 window.
- Added STRING sensor om_system_wifi_ssid / "System WLAN Name".
- Existing STRING sensors om_system_time, om_system_date and om_system_last_reboot continue to use the xbot_monitoring sensor path.
- xbot_monitoring now publishes sensor data and sensor BSON messages retained, so reconnecting clients receive the latest known value immediately.
- Web app bundle accepts STRING sensor metadata and renders string sensor values as normal sensor tiles without a gauge/bar.

Touched files:
- src/openmower_system_monitor/src/system_monitor_node.cpp
- src/openmower_system_monitor/launch/system_monitor.launch
- src/lib/xbot_monitoring/src/xbot_monitoring.cpp
- web/main.dart.js
- web/flutter_service_worker.js
