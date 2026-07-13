OpenMower GPS Logging - canonical gps_state API with mower_logic execution

Canonical public placement
--------------------------
GPS logging is exposed to MQTT clients and the app exclusively below gps_state.
The public configuration, command, runtime status, validation and last-session
data are:

  gps_state/settings/json
  gps_state/settings/set/session/json
  gps_state/settings/set/persistent/json
  gps_state/settings/set/renew/json
  gps_state/logging/set/control/json
  gps_state/logging/set/renew/json
  gps_state/logging/status/json
  gps_state/logging/last/json
  gps_state/logging/validation/json

No settings/mower_logic/satellite_logging MQTT aliases exist. The app must not
subscribe to, publish to or probe those former topic names.

Responsibility split
--------------------
The internal mower_logic implementation remains cycle-aware and owns the process
lifecycle. This keeps docking, active-work and area-id decisions close to the
mower state while presenting one coherent GPS API to the app.

Public settings
---------------
The gps_state/settings/json payload contains these GPS logging settings:

  logging_default_trigger
  logging_default_mode
  logging_default_area_id
  logging_script_path
  logging_ram_path
  logging_output_path
  logging_container_name

The path and container settings are expert settings. Public logging_* keys are
mapped internally to mower_logic dynamic_reconfigure fields. Those internal
satellite_logging_* names are implementation details and are filtered from the
public settings/mower_logic/json payload.

There is no satellite_logging_enabled field. Start, stop and cancel are commands
and must not be persisted as a boolean state.

Runtime control
---------------
Use:

  gps_state/logging/set/control/json

Examples:

  {"command":"start","trigger":"next_cycle","mode":"from_start_to_docking"}
  {"command":"start","trigger":"ad_hoc","mode":"until_docking"}
  {"command":"start","trigger":"area_id","mode":"until_docking","area_id":"3"}
  {"command":"stop"}
  {"command":"cancel"}

The command field is mandatory. If trigger, mode or area_id are omitted from a
start command, the confirmed GPS-State defaults are used. A duplicate start is
rejected while a request is active, armed or running.

Runtime status
--------------
The retained gps_state/logging/status/json payload uses schema:

  openmower.gps_state.logging.v2

It contains only the structured public contract:

  status, severity, summary
  runtime.state
  runtime.request_active
  runtime.armed
  runtime.running
  runtime.pid
  runtime.session_id
  runtime.requested_at
  runtime.started_at
  runtime.finished_at
  runtime.duration_s
  runtime.stop_reason
  request.trigger
  request.mode
  request.target_area_id
  storage.ram_path
  storage.output_path
  storage.files
  implementation.script_path
  implementation.container_name
  error

There are no duplicated top-level state, armed, running, session_id or timestamp
fields. There is no request_origin and no legacy setting field. App code must
read the structured objects.

Last completed session
----------------------
The retained gps_state/logging/last/json payload uses schema:

  openmower.gps_state.logging.last.v2

It is updated after a session has a session_id and finished_at timestamp. It
contains result, stop reason, request parameters, timestamps, duration, output
path, files and error.

Validation
----------
Control validation is published non-retained on:

  gps_state/logging/validation/json

GPS logging setting writes use:

  gps_state/settings/validation/json

A proxied setting update is first reported as forwarded/pending. The bridge then
republishes gps_state/settings/json from the confirmed mower_logic state and
publishes an applied validation result when active or persistent values match.

App implementation rules
------------------------
1. Build the GPS logging settings screen dynamically from the logging group in
   gps_state/settings/json.
2. Render logging_control as a command action, not as a persistent switch.
3. Subscribe to gps_state/logging/status/json and last/json before enabling
   actions.
4. Use runtime.request_active, runtime.armed and runtime.running for button state.
5. Do not infer runtime state from the most recently published command.
6. Display error prominently whenever severity is 4 or error is non-null.
7. Keep implementation and path fields in an expert section.
8. After a settings write, wait for pending=false or confirm the requested value
   in gps_state/settings/json.
9. Use request_id in control validation as an optional correlation value.
10. Parse openmower.gps_state.logging.v2 and read only structured fields.
11. Do not implement fallback topic probing or fallback field parsing.

Breaking change from package v0.1
---------------------------------
- Removed MQTT aliases below settings/mower_logic/satellite_logging.
- Removed the internal satellite_logging_enabled dynamic setting.
- Removed request_origin and legacy_setting_enabled from the public status.
- Removed duplicated top-level runtime fields.
- Bumped status and last-session schemas to v2.

Internal ROS topics below /mower_logic/satellite_logging remain an implementation
boundary between xbot_monitoring and mower_logic. They are not an app API.
