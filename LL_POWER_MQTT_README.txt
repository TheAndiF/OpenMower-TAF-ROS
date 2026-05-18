OpenMower - LL power parameters via MQTT (mower_comms_v1)

This overlay adds runtime MQTT write access for the low-level power configuration used by mower_comms_v1.
Changes are live for the running ROS/container session, but they are not persistent across a restart.
For persistent charging threshold changes, keep using the .env/OM_* configuration path.

MQTT status topic (retained):
  ll_power/json

MQTT set topic:
  ll_power/set/json

MQTT renew topic:
  ll_power/set/renew/json

Supported JSON fields:
  battery_critical_voltage
  battery_empty_voltage
  battery_full_voltage
  battery_critical_high_voltage
  charge_critical_high_voltage
  charge_critical_high_current

Examples:
  {"charge_critical_high_current":1.75}
  {"charge_critical_high_voltage":30.0,"battery_critical_high_voltage":29.0}

Status example:
  {"battery_critical_voltage":-1.000000,"battery_empty_voltage":22.000000,"battery_full_voltage":28.000000,"battery_critical_high_voltage":29.000000,"charge_critical_high_voltage":30.000000,"charge_critical_high_current":1.750000}

ROS bridge used internally:
  /ll/services/power/set/<field>  std_msgs/Float64
  /ll/services/power/renew        std_msgs/Empty
  /ll/services/power/status_json  std_msgs/String

The mower_comms_v1 callback updates its active power_config, writes the ROS parameter for visibility, triggers the existing low-level config send path, and publishes a new status JSON.
