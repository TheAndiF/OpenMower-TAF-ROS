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

Supported `mode` values are `hot_start`, `warm_start` and `cold_start`. The default `reset_mode` is `controlled_software`. Expert clients may set `reset_mode` to `gnss_only` or `hardware_watchdog` if that behavior is explicitly desired.

The GPS-State settings payload contains a `restart` group with a `f9p_restart` command descriptor. Apps can use that descriptor to render the command below GPS State without moving it to another MQTT namespace.

## GPS State standardized topics

The GPS MQTT payloads use schema `gps_state.v3`. All five states use the same topic structure and a common status envelope. Static field metadata and dynamic values are separated consistently.

For every state the canonical topics are:

- `gps_state/state0/definition` and `gps_state/state0/status`
- `gps_state/state1/definition` and `gps_state/state1/status`
- `gps_state/state2/definition` and `gps_state/state2/status`
- `gps_state/state3/definition` and `gps_state/state3/status`
- `gps_state/state4/definition` and `gps_state/state4/status`

All definition topics are retained. State 0 to State 2 status topics are retained. State 3 and State 4 status topics are not retained because their satellite lists are short-lived snapshots. During the migration period the former `gps_state/state1` to `gps_state/state4` topics are still published as deprecated compatibility aliases.

Every status contains the shared fields `schema`, `state`, `type`, `definition_version`, `published_at`, `source_at`, `age_ms`, `available`, `stale`, `status`, `severity`, `summary` and `data`. Existing state-specific fields remain available at the top level for compatibility and are additionally grouped below `data`.

The functional separation remains unchanged:

- State 0: complete 12-stage drive-readiness decision chain for expert/debug use.
- State 1: compact operator status for GPS drive readiness.
- State 2: technical GNSS and pose summary without a satellite list.
- State 3: currently used satellites only.
- State 4: complete expert/debug list of all visible satellites.

## Central GPS State refresh

Manual refresh requests use one central topic:

- `gps_state/set/renew/json`

An empty payload or `{}` republishes definition and status for all currently enabled states. A request can select states and payload parts explicitly:

```json
{
  "states": [0, 2],
  "parts": ["status"]
}
```

```json
{
  "states": [0, 1, 2, 3, 4],
  "parts": ["definition"]
}
```

State values may be integers from `0` to `4`, strings such as `state2`, or the string `all`. `parts` accepts `definition` and `status`. Explicitly selected states are published independently of their regular publish switches. The central request recalculates time-dependent drive status from the latest ROS inputs and uses an unavailable/empty fallback for states whose satellite snapshot has not arrived yet.

The former `gps_state/state0/set/renew/json` topic remains as a deprecated compatibility alias and republishes only State 0 definition and status. New clients should use the central topic.

## GPS State 0 drive diagnostics

State 0 remains split into static and live data so long descriptions do not have to be sent repeatedly.

- `gps_state/state0/definition` contains the retained static definition of the 12 decision stages.
- `gps_state/state0/status` contains the retained live state, current values, thresholds, severity and the first blocking stage.
- `gps_state/settings/json` contains `publish_state0` and `publish_rate_hz` for regular publication.

### Update behavior

- Static definitions are published on MQTT connection, settings refresh/change and central renew requests; they are not repeated in every regular 1 Hz cycle.
- Dynamic status payloads follow the configured publish rate and event-driven updates such as GNSS fix-state changes.
- `publish_rate_hz` defaults to `1.0 Hz` and accepts `0.1` to `5.0 Hz`, corresponding to intervals from 10 seconds to 0.2 seconds.
- The central renew request is answered immediately and does not request a new physical measurement from the F9P. It evaluates the latest ROS inputs.
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
