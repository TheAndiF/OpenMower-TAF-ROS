OpenMower GPS Logging - gps_state public API with mower_logic runtime integration

Canonical public placement
--------------------------
GPS logging is exposed to MQTT clients and the app below gps_state. The public
configuration, command, runtime status, validation and last-session data are:

  gps_state/settings/json
  gps_state/settings/set/session/json
  gps_state/settings/set/persistent/json
  gps_state/settings/set/renew/json
  gps_state/logging/set/control/json
  gps_state/logging/set/renew/json
  gps_state/logging/status/json
  gps_state/logging/last/json
  gps_state/logging/validation/json

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

The path and container settings are expert settings. The public keys are mapped
to the existing internal mower_logic dynamic_reconfigure keys. The internal
satellite_logging_enabled field is intentionally not exposed as an app toggle.
Start, stop and cancel are commands and must not be persisted as a boolean state.

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

If trigger, mode or area_id are omitted, the configured defaults are used. An
explicit command creates an independent runtime request and does not require the
legacy satellite_logging_enabled flag.

Runtime status
--------------
The retained gps_state/logging/status/json payload uses schema:

  openmower.gps_state.logging.v1

It contains:

  status, severity, summary
  runtime.state
  runtime.request_active
  runtime.request_origin
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
  implementation.legacy_setting_enabled
  error

Frequently used compatibility fields such as state, armed, running, session_id,
started_at, finished_at and stop_reason are also present at the top level.

Last completed session
----------------------
The retained gps_state/logging/last/json payload uses schema:

  openmower.gps_state.logging.last.v1

It is updated after a session has a session_id and finished_at timestamp. It
contains result, stop reason, request parameters, timestamps, duration, output
path, files and error.

Validation
----------
Control validation is published non-retained on:

  gps_state/logging/validation/json

GPS logging setting writes use the normal GPS-state validation topic:

  gps_state/settings/validation/json

A proxied setting update is first reported as forwarded/pending. The bridge then
republishes gps_state/settings/json from the confirmed mower_logic state and
publishes an applied validation result when active or persistent values match.

App implementation rules
------------------------
1. Build the GPS logging settings screen dynamically from the logging group in
   gps_state/settings/json.
2. Render logging_control as a command action, not as a persistent switch.
3. Subscribe to gps_state/logging/status/json before showing action buttons.
4. Use runtime.running and runtime.armed for button state; do not infer state
   from the last command sent.
5. Display error prominently whenever severity is 4 or error is non-null.
6. Use gps_state/logging/last/json for the last completed recording card.
7. Keep implementation and path fields in an expert section.
8. After a settings write, wait for pending=false or confirm the requested value
   in gps_state/settings/json.
9. Treat request_id in command validation as an optional correlation value.
10. Do not depend on the deprecated settings/mower_logic satellite logging topics.

Compatibility
-------------
The former MQTT topics below settings/mower_logic/satellite_logging remain as
migration aliases. The former satellite_logging_* fields are removed from the
published settings/mower_logic/json payload to avoid duplicate app controls.
They remain internal dynamic_reconfigure fields so existing installations and
legacy clients can continue to operate during migration.

Responsibility split
--------------------
gps_state MQTT API: public settings, commands, validation, status and last session.
xbot_monitoring: validation, public/internal key mapping, schema transformation,
                  retained MQTT publication and compatibility aliases.
mower_logic: cycle-aware arming, start/stop decisions and logger process lifecycle.
record_satellites.sh: records ROS topics in RAM and copies files to persistent
                      storage when the process is terminated.
