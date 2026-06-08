MQTT Satellite Logger
=====================

Neue MQTT Topics:

- satellite_logger/set/record_next/json
  Payload-Beispiel:
  {"mode":"from_start_to_docking"}
  oder:
  {"mode":"from_docking_to_docking"}

- satellite_logger/set/cancel
  Stoppt einen laufenden Mitschnitt oder nimmt die Vormerkung zurück.

- satellite_logger/set/renew/json
  Fordert den aktuellen retained Status erneut an.

- satellite_logger/status/json
  Retained Status mit state, mode, started_at, finished_at, session_id, files, path, ram_path und error.

Arbeitsweise:

Der Logger wird per MQTT für den nächsten Zyklus vorgemerkt. Beim Start wird scripts/record_satellites.sh gestartet. Das Skript schreibt laufende Daten zuerst in den Arbeitsspeicher unter /dev/shm/openmower_satellite_logs/<session_id>. Erst beim Stoppen, typischerweise beim erneuten Docking/Laden, werden die Logdateien nach /home/openmower/recordings/logs kopiert.

Parameter:

- xbot_monitoring/satellite_logger_script
- xbot_monitoring/satellite_logger_output_dir
- xbot_monitoring/satellite_logger_ram_dir
- xbot_monitoring/satellite_logger_container_name

Im normalen ROS-Containerbetrieb bleibt satellite_logger_container_name leer, sodass rostopic direkt im laufenden Container ausgeführt wird. Für manuelle Host-Nutzung kann SAT_LOG_CONTAINER=ros-1 gesetzt werden.
