MQTT Satellite Logging
======================

Current implementation
----------------------
Satellite logging is now integrated under mower_logic.

Use these topics for normal operation:

- settings/mower_logic/json
- settings/mower_logic/set/session/json
- settings/mower_logic/set/persistent/json
- settings/mower_logic/set/renew/json
- mower_logic/satellite_logging/json
- mower_logic/satellite_logging/set/control/json
- mower_logic/satellite_logging/set/renew/json

The old satellite_logger/... topics are not the canonical interface anymore.

Normal operation through the enable switch
------------------------------------------
The usual app/MQTT workflow is:

1. Configure the desired defaults through settings/mower_logic:

   satellite_logging_default_trigger
   satellite_logging_default_mode
   satellite_logging_default_area_id

2. Set:

   {"satellite_logging_enabled":{"value":true}}

   on:

   settings/mower_logic/set/session/json

mower_logic then checks the script path, makes the script executable if needed, and starts or arms the logger according to the configured values.

Switching satellite_logging_enabled back to false stops a running log and cancels an armed request.

When the configured end condition is reached, mower_logic stops logging and automatically sets satellite_logging_enabled back to false so the app shows Off again.

Start variants
--------------
- ad_hoc: start immediately
- next_cycle: arm for the next matching mowing cycle
- area_id: arm for satellite_logging_default_area_id

End/mode variants
-----------------
- until_docking: stop at docking
- from_start_to_docking: start with active mowing work and stop at docking
- from_docking_to_docking: start after leaving docking and stop at the next docking

Container handling
------------------
satellite_logging_container_name is optional. Leave it empty for automatic detection.

If the script is already running in the ROS container, rostopic is executed directly. If the script is running on the host and Docker is available, the script tries to find a ROS container automatically, preferring names such as:

- openmower-open_mower_ros-1
- OpenMowerROS
- names containing open_mower_ros

Advanced runtime control
------------------------
The control endpoint remains available:

  mower_logic/satellite_logging/set/control/json

Examples:

  {"command":"start","trigger":"next_cycle","mode":"from_start_to_docking"}
  {"command":"start","trigger":"ad_hoc","mode":"until_docking"}
  {"command":"start","trigger":"area_id","mode":"until_docking","area_id":"3"}
  {"command":"stop"}
  {"command":"cancel"}
