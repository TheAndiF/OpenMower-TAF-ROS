# GPS logging API for app integration

## Scope

The canonical app and MQTT API is below `gps_state`. `mower_logic` remains the internal cycle-aware executor, but its logging fields are not rendered as a second settings page.

## Topic contract

| Topic | Retained | Direction | App purpose |
|---|---:|---|---|
| `gps_state/settings/json` | yes | mower -> app | Dynamic settings metadata, including group `logging` |
| `gps_state/settings/set/session/json` | no | app -> mower | Apply logging defaults for the current session |
| `gps_state/settings/set/persistent/json` | no | app -> mower | Persist logging defaults |
| `gps_state/settings/validation/json` | no | mower -> app | Pending/applied/rejected result for settings writes |
| `gps_state/logging/set/control/json` | no | app -> mower | Start, stop or cancel |
| `gps_state/logging/set/renew/json` | no | app -> mower | Republish settings, runtime status and last session |
| `gps_state/logging/status/json` | yes | mower -> app | Current runtime state |
| `gps_state/logging/last/json` | yes | mower -> app | Last completed session |
| `gps_state/logging/validation/json` | no | mower -> app | Immediate control-command validation |

## Settings keys

The app must discover these entries from `gps_state/settings/json` rather than hard-coding labels or expert visibility:

- `logging_default_trigger`
- `logging_default_mode`
- `logging_default_area_id`
- `logging_output_path`
- `logging_ram_path`
- `logging_script_path`
- `logging_container_name`
- `logging_control`
- `logging_status`
- `logging_last`
- `logging_validation`
- `logging_renew`

`logging_default_trigger` and `logging_default_mode` contain an `enum` array. The app should render them as selectors. Path and container fields have `expert=true`.

## Control payloads

```json
{"command":"start","trigger":"ad_hoc","mode":"until_docking","request_id":"ui-42"}
```

```json
{"command":"start","trigger":"next_cycle","mode":"from_start_to_docking"}
```

```json
{"command":"start","trigger":"area_id","mode":"until_docking","area_id":"3"}
```

```json
{"command":"stop"}
```

```json
{"command":"cancel"}
```

`request_id` is optional and is echoed by the bridge validation response. The runtime controller ignores unknown correlation metadata. `cancel` cancels an armed request; if recording has already started, it stops the process, preserves the generated session files and records the result as `cancelled`.

## Runtime state machine

| Runtime condition | `runtime.state` | `runtime.armed` | `runtime.running` | Recommended app action |
|---|---|---:|---:|---|
| No request | `idle` or `finished` | false | false | Enable Start |
| Waiting for cycle/area | `armed` | true | false | Show Cancel |
| Recording | `running` | false | true | Show Stop |
| Completed | `finished` | false | false | Show result and Start |
| Error | `error` | false | false | Show error and retry options |

The app must use the confirmed status topic, not the last command sent, as the source of truth. While a recording is running, `duration_s` only changes when a fresh status is published. For a smooth timer, calculate the live duration locally from `runtime.started_at` and replace it whenever a new retained status arrives.

## Status payload

```json
{
  "schema": "openmower.gps_state.logging.v1",
  "type": "status",
  "published_at": 1783951200.0,
  "status": "running",
  "severity": 1,
  "summary": "GPS-Aufzeichnung läuft",
  "runtime": {
    "state": "running",
    "request_active": true,
    "request_origin": "command",
    "armed": false,
    "running": true,
    "pid": 1842,
    "session_id": "20260713_143522",
    "requested_at": "2026-07-13T14:35:21Z",
    "started_at": "2026-07-13T14:35:22Z",
    "finished_at": null,
    "duration_s": 142.5,
    "stop_reason": null
  },
  "request": {
    "trigger": "ad_hoc",
    "mode": "until_docking",
    "target_area_id": null
  },
  "storage": {
    "ram_path": "/dev/shm/openmower_satellite_logs",
    "output_path": "/home/openmower/recordings/logs",
    "files": []
  },
  "implementation": {
    "script_path": "/home/openmower/scripts/record_satellites.sh",
    "container_name": null,
    "legacy_setting_enabled": false
  },
  "error": null
}
```

Severity values follow the existing GPS-state convention: `0` normal, `1` active/informational and `4` error.

## Settings write lifecycle

1. The app publishes a session or persistent payload with public `logging_*` keys.
2. `gps_state/settings/validation/json` reports `status=forwarded` and `pending=true`.
3. The bridge maps the keys to internal mower-logic fields.
4. The confirmed mower-logic settings state is received.
5. `gps_state/settings/json` is republished with confirmed public values.
6. A second validation reports `status=applied` and `pending=false`.

The app should keep the edited control in a pending state between steps 2 and 6. A timeout should offer Renew instead of assuming success.

## Recommended screen layout

- **Header card:** status summary, locally calculated active duration, Start/Stop/Cancel button.
- **Current request:** trigger, mode and target area.
- **Current session:** session ID, start time, file count and output path link/action where supported.
- **Last session:** result, duration, stop reason and files from `gps_state/logging/last/json`.
- **Defaults:** trigger, mode and default area.
- **Expert section:** RAM path, output path, script path, container name, PID and raw error.

Do not show internal `satellite_logging_enabled` or any `satellite_logging_*` key. Do not use a toggle as a substitute for control commands.

## Compatibility

Legacy topics under `settings/mower_logic/satellite_logging/...` continue to work during migration. New app code must only use the canonical `gps_state/logging/...` topics. The bridge removes internal logging settings from the public `settings/mower_logic/json` payload to prevent duplicate controls.
