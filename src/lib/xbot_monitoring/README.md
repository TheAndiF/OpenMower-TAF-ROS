# xBot Monitoring

This package is meant for monitoring your robot system.

The goal is to have this as generic as possible so that we can monitor arbitrary values of any robotics system easily.

## MQTT topic conventions

The monitoring bridge exposes JSON topics as the primary human-readable MQTT API. Existing BSON topics remain available where they are already used for the web app or compatibility, but new MQTT features should be designed around JSON first.

Resource-oriented topics follow this pattern where applicable:

- `<resource>/json` publishes the retained confirmed state.
- `<resource>/set/json` writes a full replacement or requested change.
- `<resource>/set/renew/json` requests a republish of the current state.
- `<resource>/validation/json` publishes retained validation feedback for accepted or rejected writes.

Map overlay publication uses `map/overlay/json` and `map/overlay/bson` as the canonical topics. The old `map_overlay/json` and `map_overlay/bson` topics are retained as compatibility aliases during migration.

Settings writes now publish validation feedback on:

- `settings/ll_board/validation/json`
- `settings/mower_logic/validation/json`

The deprecated `settings/mow_load_factor/...` MQTT topics are no longer published by the bridge. Load regulation settings are part of `settings/mower_logic/json`.

The validation payload reports the namespace, write mode (`session` or `persistent`), accepted keys, and rejected keys with rejection reasons. Valid keys from a mixed payload are applied; invalid or unknown keys are reported as rejected.

## Robot State world pose

`robot_state/json` now contains the local OpenMower pose and, when a GPS datum is available, the corresponding WGS84 world coordinate. The coordinate is derived from the same `robot_pose` used by OpenMower navigation, not from a separate raw NMEA sentence.

```json
{
  "pose": {
    "x": 12.34,
    "y": 5.67,
    "heading": 1.23,
    "pos_accuracy": 0.03,
    "heading_accuracy": 0.01,
    "heading_valid": true
  },
  "world_pose": {
    "valid": true,
    "coordinate_system": "WGS84",
    "source": "robot_pose_to_wgs84",
    "latitude": 52.2057601,
    "longitude": 13.0761302,
    "altitude": 0.0,
    "pos_accuracy": 0.03
  }
}
```

If the mower is running without an absolute GPS datum, `world_pose.valid` remains `false` and `world_pose.reason` explains why, for example `gps_datum_unavailable`. The conversion uses `/ll/services/gps/datum_lat`, `/ll/services/gps/datum_long` and `/ll/services/gps/datum_height`, converts that datum to UTM, adds `pose.x`/`pose.y` in meters and converts the result back to WGS84 latitude/longitude.


## Sensors settings metadata API

`sensors/settings/json` follows the same structural idea as the dynamic settings pages while living below the existing `sensors/...` MQTT branch. The retained JSON payload uses the sensor settings v2 schema and contains both group metadata and sensor metadata:

```json
{
  "namespace": "sensors",
  "schema": "settings_v2",
  "readonly": true,
  "groups": {
    "host_system": {
      "label": "Host-System",
      "order": 10
    },
    "openmower": {
      "label": "OpenMower",
      "order": 20
    }
  },
  "settings": {
    "<sensor_id>": {
      "label": "Display label",
      "description": "Display description",
      "group": "host_system",
      "order": 10,
      "type": "number",
      "unit": "%",
      "value": null,
      "active": null,
      "persistent": null,
      "visible": true,
      "expert": false,
      "readonly": true,
      "different": false,
      "restart_required": false,
      "session_apply_supported": false,
      "sensor_id": "<sensor_id>",
      "sensor_name": "Original ROS sensor name",
      "value_topic": "sensors/<sensor_id>/data"
    }
  }
}
```

The technical sensor data is still produced by ROS `SensorInfo` messages and the live values remain on `sensors/<sensor_id>/data`. `sensors/settings/json` adds editable display metadata (`label`, `description`, `group`, `order`, `visible`, `expert`) so the app can group and order the sensor view like other dynamically generated settings pages. Group metadata is stored separately under `groups.<group_id>.label` and `groups.<group_id>.order`. If no persistent `groups` object exists yet, the bridge derives default groups from `settings.<sensor_id>.group`.

### MQTT topics

- `sensors/settings/json` publishes the retained settings-v2-like sensor metadata state including `groups` and `settings`.
- `sensors/settings/set/renew/json` requests a republish of the current retained sensor metadata state.
- `sensors/settings/set/persistent/json` stores editable display metadata in the persistent settings file under namespace `sensors`.
- `sensors/settings/validation/json` publishes retained validation feedback for the last persistent write.
- `sensors/settings/bson` is the BSON equivalent of `sensors/settings/json` for clients that still require BSON.

Example persistent write with the current app structure:

```json
{
  "namespace": "sensors",
  "schema": "settings_v2",
  "groups": {
    "host_system": {
      "label": "Host-System",
      "order": 10
    }
  },
  "settings": {
    "om_v_battery": {
      "label": "Akkuspannung",
      "group": "openmower_power",
      "order": 10,
      "visible": true,
      "expert": false
    }
  }
}
```

Legacy persistent writes without a top-level `settings` object are still accepted and are migrated into `settings.sensors.settings` inside `/data/ros/settings_persistent.json`:

```json
{
  "om_v_battery": {
    "label": "Akkuspannung",
    "group": "openmower_power",
    "order": 10,
    "visible": true,
    "expert": false
  }
}
```

The bridge accepts only editable display fields. Sensor values and technical ROS fields are read-only through this API. Unknown fields, unknown sensor ids, invalid group ids and invalid metadata types are rejected. A successful persistent write stores the migrated namespace structure below `settings.sensors.settings` and `settings.sensors.groups`, publishes validation feedback and immediately republishes the complete retained `sensors/settings/json` state.

## GPS State F9P restart command

`gps_state` also exposes a JSON command topic for u-blox/ZED-F9P receiver restarts. The MQTT bridge validates the request, forwards it to the GPS driver through ROS and republishes the driver response as a retained status.

### MQTT topics

- `gps_state/restart/set/json` sends a restart request.
- `gps_state/restart/status/json` publishes the retained status of the last request or driver response.
- `gps_state/restart/validation/json` publishes validation feedback for the last MQTT command.
- `gps_state/restart/set/renew/json` republishes the retained restart status and the `gps_state/settings/json` metadata.

Example hot start request:

```json
{
  "mode": "hot_start"
}
```

Supported `mode` values are `hot_start`, `warm_start` and `cold_start`. The default `reset_mode` is `controlled_software`. Expert clients may set `reset_mode` to `gnss_only`, `hardware_watchdog` or `hardware_after_shutdown` if that behavior is explicitly desired. `hardware_after_shutdown` maps to UBX-CFG-RST `resetMode=0x04` and does not require the external RESET_N pin.

The GPS-State settings payload contains a `restart` group with a `f9p_restart` command descriptor. Apps can use that descriptor to render the command below GPS State without moving it to another MQTT namespace.

## GPS State MQTT topics

The GPS MQTT API exposes only the canonical states `state1`, `state2`, `state3` and `state4`. The former State0, standalone State1 and temporary State01 interfaces are not available.

Canonical topics:

- `gps_state/state1/definition`, `gps_state/state1/status`, `gps_state/state1/request`
- `gps_state/state2/definition`, `gps_state/state2/status`, `gps_state/state2/satellites`, `gps_state/state2/request`
- `gps_state/state3/definition`, `gps_state/state3/status`, `gps_state/state3/request`
- `gps_state/state4/definition`, `gps_state/state4/status`, `gps_state/state4/request`

Static definitions are retained. `state1/status` is retained and remains continuously available. Dynamic State2-State4 status and satellite payloads are not retained and are published only while an App lease is active. No compatibility aliases such as `gps_state/state0/*`, `gps_state/state01/*` or the flat `gps_state/stateN` topics are subscribed or published.

The functional separation is:

- State1: combined GPS drive-readiness result and complete decision chain.
- State2: technical GNSS, RTK and pose summary with an optional separate satellite list.
- State3: currently used satellites.
- State4: all visible satellites and extended diagnostics.

## Central GPS State refresh

Manual refresh requests use `gps_state/set/renew/json`. An empty payload republishes settings plus the supported definitions and statuses. A request can select states and payload parts explicitly:

```json
{
  "states": ["state1", "state2"],
  "parts": ["status"]
}
```

Supported state selectors are `state1`, `state2`, `state3`, `state4`, the integers `1` to `4`, and `all`. State0, State01 and the former standalone State1 contract are rejected.

## GPS State1 drive diagnostics

The canonical State1 combines the former State0 decision chain and the former standalone State1 summary. It remains split into static and live data:

- `gps_state/state1/definition` contains the retained static definition of the decision stages.
- `gps_state/state1/status` contains the retained full snapshot, current values, thresholds, severity and first blocking stage. The `data` object additionally contains `current_status` and `gps_quality`; `current_status` mirrors the Dashboard operating state from `robot_state/current_state` in lowercase, while `gps_quality` mirrors State2 `quality_class`.
- `gps_state/state1/request` accepts `{"command":"publish_now","request_id":"..."}` for an immediate snapshot.
- `gps_state/settings/json` no longer contains `publish_state0` or `publish_state1` switches.

### Update behavior

- Static definitions are published on MQTT connection, settings refresh/change and renew requests.
- State1 dynamic status follows the configured publish rate and relevant event-driven changes.
- State2-State4 are activated and stopped through their request topics with per-client leases.
- Session changes use `gps_state/settings/set/session/json`; persistent changes use `gps_state/settings/set/persistent/json`.

### Status values

- `ok`: condition is fulfilled.
- `warning`: condition is not fully fulfilled but the system is still inside a tolerated range.
- `blocked`: GPS-dependent driving is currently blocked.
- `stop`: safety stop condition, driving and blades are disabled by the logic.
- `inactive`: stage was not evaluated because an earlier stage already blocks the chain.
- `unknown`: the required diagnostic input is not available.
- `unavailable`: the complete state data source is not available.

### Stage order

1. `gps_enabled` - GPS processing in `xbot_positioning` is active.
2. `rtk_fixed` - `/ll/position/gps` reports RTK Fixed.
3. `gps_input_accuracy` - input GPS accuracy is below `/xbot_positioning/max_gps_accuracy`.
4. `valid_gps_samples` - `xbot_positioning` collected more than 10 valid GPS samples.
5. `absolute_gps_pose_recent` - the last accepted absolute GPS pose is younger than 10 seconds.
6. `xb_pose_received` - `/xbot_positioning/xb_pose` is available.
7. `xb_pose_age` - the pose is younger than one second.
8. `orientation_valid` - the pose orientation is valid.
9. `pose_accuracy` - pose accuracy is below `/mower_logic/max_position_accuracy`.
10. `recent_absolute_pose` - the pose has `FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE`.
11. `gps_timeout` - the last good GPS state is still inside `/mower_logic/gps_timeout` if the recent absolute pose is missing.
12. `gps_drive_ready` - overall drive-readiness result.

`xbot_positioning` publishes the auxiliary ROS topic `/xbot_positioning/gps_debug_state` as retained JSON. `xbot_monitoring` uses it to fill State 0 stages that are otherwise internal to `xbot_positioning`, especially `gps_enabled`, `valid_gps_samples`, `has_gps`, outlier count and the age of the last accepted GPS update.


## GPS logging under GPS State

The canonical app-facing GPS logging API now lives below `gps_state`. The detailed contract, payload examples, state machine and app implementation guidance are documented in [GPS_LOGGING_API.md](GPS_LOGGING_API.md).

Canonical topics:

- `gps_state/logging/set/control/json`
- `gps_state/logging/set/renew/json`
- `gps_state/logging/status/json`
- `gps_state/logging/last/json`
- `gps_state/logging/validation/json`

Logging defaults and expert paths are part of the `logging` group in `gps_state/settings/json` and are changed through the normal GPS-State session/persistent settings endpoints. The public settings use `logging_*` names and are mapped to the existing internal `satellite_logging_*` mower-logic fields.

Start, stop and cancel are commands. The app must not represent runtime control as a persistent boolean switch. `gps_state/logging/status/json` is retained and is the source of truth for button state. `gps_state/logging/last/json` is retained separately so the last completed recording remains visible after the live runtime returns to an idle state.

No MQTT aliases below `settings/mower_logic/satellite_logging/...` are supported. Internal logging configuration fields remain filtered from the published `settings/mower_logic/json` payload so a dynamically generated app renders the single canonical `gps_state` logging group. The app must parse the structured `openmower.gps_state.logging.v2` status payload and must not implement fallback topic or fallback field handling.

## Canonical GPS-State MQTT contract (2026-07-15)

The canonical GPS API uses only `gps_state/state1` through `gps_state/state4`. Legacy State0, State01, flat state aliases and legacy logging/restart aliases are not published or subscribed.

- State1 is the continuously available, retained drive-readiness snapshot.
- State2 is a lease-controlled GNSS/pose diagnostic view with an optional separate satellite list.
- State3 is lease-controlled and contains only satellites currently used by the navigation solution.
- State4 is lease-controlled and contains every satellite seen by the receiver, including both used and not-used satellites.

State3 and State4 are inactive by default. `set_active` starts or renews a client-specific lease; `active=false` removes that client's lease; lease expiry stops traffic automatically when no client remains. `publish_now` creates one snapshot without permanently enabling the view.

GPS settings use the same session/persistent/renew/validation pattern as the hardware and mower-logic settings. Default State2-State4 intervals and the default activation lease are settings; runtime activation remains a command. Restart and logging remain dedicated action trees with validation, live status and last-result topics.
