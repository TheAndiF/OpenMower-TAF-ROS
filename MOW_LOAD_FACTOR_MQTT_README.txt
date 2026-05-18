Mäh-Lastfaktor: erste Ausbaustufe
====================================

Diese Änderung berechnet einen Lastfaktor aus
- Mähmotorstrom,
- Mähmotortemperatur,
- Mäh-ESC-Temperatur.

Der Faktor beeinflusst in dieser Ausbaustufe noch NICHT die Fahrgeschwindigkeit.
Er wird nur berechnet, publiziert und per MQTT schaltbar gemacht.

Defaults
--------
Die Startwerte sind in MowerLogic.cfg und openmower_defaults_v2.yaml hinterlegt:
- mow_load_factor_enabled: false
- mow_load_factor_min: 0.40
- mow_load_current_start: 0.75 A
- mow_load_current_end: 1.25 A
- mow_load_motor_temp_start: 55.0 °C
- mow_load_motor_temp_end: 68.0 °C
- mow_load_esc_temp_start: 60.0 °C
- mow_load_esc_temp_end: 78.0 °C

Dauerhafte roboterspezifische Überschreibung
--------------------------------------------
In PARAMS_PATH/mower_params.yaml bzw. /home/openmower/params/mower_params.yaml:

mower_logic:
  mow_load_factor_enabled: false
  mow_load_factor_min: 0.40
  mow_load_current_start: 0.75
  mow_load_current_end: 1.25

MQTT
----
Status, retained:
  mow_load_factor/json

Beispiel:
  {"enabled":false,"min_factor":0.400000,"current_start":0.750000,"current_end":1.250000,"factor_current":0.820000,"factor_motor_temp":1.000000,"factor_esc_temp":1.000000,"computed_factor":0.820000,"effective_factor":1.000000}

Aktivieren des berechneten Faktors:
  Topic: mow_load_factor/set/json
  Payload: {"enabled":true}

Wieder fest auf 1.0 schalten:
  Topic: mow_load_factor/set/json
  Payload: {"enabled":false}

Minimalfaktor zur Laufzeit ändern:
  Topic: mow_load_factor/set/json
  Payload: {"min_factor":0.55}
  Erlaubter Bereich: 0.10 bis 1.00

Stromkennlinie zur Laufzeit ändern:
  Topic: mow_load_factor/set/json
  Payload: {"current_start":0.80,"current_end":1.40}
  Regeln:
  - current_start >= 0.0 A
  - current_end > current_start

Die Werte können auch einzeln geändert werden, solange die Grenzen danach gültig bleiben:
  Payload: {"current_start":0.80}
  Payload: {"current_end":1.40}

Kombinierte Änderung:
  Payload: {"enabled":true,"min_factor":0.45,"current_start":0.80,"current_end":1.40}

Status erneut anfordern:
  Topic: mow_load_factor/set/renew/json
  Payload: beliebig oder leer

ROS Topics
----------
- /mower_logic/mow_load_factor/computed   std_msgs/Float32
- /mower_logic/mow_load_factor/effective  std_msgs/Float32
- /mower_logic/mow_load_factor/status_json std_msgs/String

Hinweis
-------
Rohwerte wie Mähmotorstrom, Mähmotortemperatur, Mäh-ESC-Temperatur und RPM bleiben
wie bereits vorhanden unter sensors/om_mow_.../data im MQTT verfügbar.
