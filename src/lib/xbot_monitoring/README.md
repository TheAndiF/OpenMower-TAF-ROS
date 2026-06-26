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
