# GPS logging API for app integration

## Scope

The app and MQTT API is available **only** below `gps_state`. `mower_logic` remains the internal cycle-aware executor, but its internal logging fields and ROS topics are not part of the app contract.

This document describes package version v0.3 and the status schema `openmower.gps_state.logging.v2`. The API deliberately contains no compatibility topics or compatibility fields.

## Topic contract

| Topic | Retained | Direction | App purpose |
|---|---:|---|---|
| `gps_state/settings/json` | yes | mower -> app | Dynamic settings metadata, including group `logging` |
| `gps_state/settings/set/session/json` | no | app -> mower | Apply logging defaults for the current session |
| `gps_state/settings/set/persistent/json` | no | app -> mower | Persist logging defaults |
| `gps_state/settings/validation/json` | no | mower -> app | Pending/applied/rejected result for settings writes |
| `gps_state/logging/set/control/json` | no | app -> mower | Start, stop or cancel |
| `gps_state/logging/set/renew/json` | no | app -> mower | Republish settings, runtime status and last session |
| `gps_state/logging/status/json` | yes | mower -> app | Current runtime state, schema v2 |
| `gps_state/logging/last/json` | yes | mower -> app | Last completed session, schema v2 |
| `gps_state/logging/validation/json` | no | mower -> app | Immediate control-command validation |

### Nonexistent app endpoints

The following old MQTT namespace is intentionally unsupported:

```text
settings/mower_logic/satellite_logging/*
```

The app must not probe that namespace, subscribe to it or publish to it. Failure to receive a message there is expected behavior.

## Settings keys

The app should discover these entries from `gps_state/settings/json` rather than hard-coding labels, ordering or expert visibility:

- `logging_default_trigger` (`ad_hoc`, `next_cycle`, `area_id`)
- `logging_default_mode` (`manual`, `until_docking`, `from_start_to_docking`, `from_docking_to_docking`)
- `logging_default_area_id` (string or `null`)
- `logging_output_path`
- `logging_ram_path`
- `logging_script_path`
- `logging_container_name`
- `logging_control`
- `logging_status`
- `logging_last`
- `logging_validation`
- `logging_renew`

`logging_default_trigger` and `logging_default_mode` contain an `enum` array. Render them as selectors. Path and container fields have `expert=true`.

There is no logging enable setting. Runtime activation is command-based.


### Start resolution

A start payload may contain only `command` and an optional `request_id`:

```json
{"command":"start","request_id":"ui-42"}
```

For omitted logging fields, the runtime controller creates an immutable request snapshot from the currently active settings. Resolution order is:

1. Explicit fields in the start command (one-shot override).
2. Active session values.
3. Persistent values after restart.
4. Backend defaults (`ad_hoc` and `until_docking`).

Changing settings while a request is armed or running does not modify that request snapshot.

## Control payloads

The `command` field is mandatory.

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
{"command":"stop","request_id":"ui-43"}
```

```json
{"command":"cancel","request_id":"ui-44"}
```

`request_id` is optional and is echoed by the immediate bridge validation response. The runtime controller ignores correlation metadata.

`cancel` cancels an armed request. If recording has started, it stops the process, preserves the generated session files and records the result as `cancelled` in `last/json`.

## Immediate command validation

`gps_state/logging/validation/json` is not retained. A successful bridge validation resembles:

```json
{
  "schema": "openmower.gps_state.logging.validation.v1",
  "type": "control",
  "valid": true,
  "status": "forwarded",
  "request_id": "ui-42",
  "accepted": [{"command":"start"}],
  "rejected": [],
  "published_at": 1783951200.0
}
```

`forwarded` confirms only that the bridge accepted and forwarded the command. It does not confirm `armed` or `running`. The app must wait for `gps_state/logging/status/json`.

## Runtime state machine

| Runtime condition | `runtime.state` | `runtime.request_active` | `runtime.armed` | `runtime.running` | Recommended app action |
|---|---|---:|---:|---:|---|
| No request | `idle` or `finished` | false | false | false | Enable Start |
| Waiting for cycle/area | `armed` | true | true | false | Show Cancel |
| Recording | `running` | true | false | true | Show Stop |
| Completed | `finished` | false | false | false | Show result and Start |
| Error | `error` | false or true | false | false | Show error and Renew/retry |

Use the confirmed status topic, not the last command sent, as the source of truth. While recording, `duration_s` changes only when a new status is published. For a smooth timer, calculate the live duration locally from `runtime.started_at` and resynchronize on each status update.

## Status payload v2

```json
{
  "schema": "openmower.gps_state.logging.v2",
  "type": "status",
  "published_at": 1783951200.0,
  "status": "running",
  "severity": 1,
  "summary": "GPS-Aufzeichnung läuft",
  "runtime": {
    "state": "running",
    "request_active": true,
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
    "container_name": null
  },
  "error": null
}
```

Severity values follow the GPS-State convention: `0` normal, `1` active/informational and `4` error. Never communicate status only through color.

### Required field paths for the app

| Purpose | Field path |
|---|---|
| Current state | `runtime.state` |
| Start lock | `runtime.request_active` |
| Cancel visibility | `runtime.armed` |
| Stop visibility | `runtime.running` |
| Active timer | `runtime.started_at` and `runtime.duration_s` |
| Session details | `runtime.session_id` |
| End reason | `runtime.stop_reason` |
| Request summary | `request.trigger`, `request.mode`, `request.target_area_id` |
| Files | `storage.files` |
| Standard error UI | `summary`, `severity`, `error` |
| Expert diagnostics | `runtime.pid`, `storage.*`, `implementation.*` |

Do not read removed top-level fields such as `state`, `armed`, `running`, `session_id`, `started_at`, `finished_at` or `stop_reason`. They are not present in v2.

## Last completed session v2

`gps_state/logging/last/json` uses `openmower.gps_state.logging.last.v2` and remains retained independently of the live status.

```json
{
  "schema": "openmower.gps_state.logging.last.v2",
  "type": "last",
  "result": "finished",
  "session_id": "20260713_143522",
  "requested_at": "2026-07-13T14:35:21Z",
  "started_at": "2026-07-13T14:35:22Z",
  "finished_at": "2026-07-13T15:17:04Z",
  "duration_s": 2502.0,
  "stop_reason": "docked",
  "trigger": "next_cycle",
  "mode": "from_start_to_docking",
  "target_area_id": null,
  "output_path": "/home/openmower/recordings/logs",
  "files": ["gps_position_20260713_143522.log"],
  "error": null
}
```

Possible `result` values are `finished`, `cancelled` and `error`.

## Settings write lifecycle

1. Publish a session or persistent payload with public `logging_*` keys.
2. `gps_state/settings/validation/json` reports `status=forwarded` and `pending=true`.
3. The bridge maps the public keys to the internal mower-logic configuration.
4. The confirmed mower-logic settings state is received.
5. `gps_state/settings/json` is republished with confirmed public values.
6. A second validation reports `status=applied` and `pending=false`.

Keep edited controls in a pending state between steps 2 and 6. On timeout, retain the last confirmed value and offer Renew rather than assuming success.

Example:

```json
{
  "logging_default_trigger": {"value": "area_id"},
  "logging_default_area_id": {"value": "3"}
}
```

## Connection and initialization sequence

1. Subscribe to `gps_state/settings/json`.
2. Subscribe to `gps_state/settings/validation/json`.
3. Subscribe to `gps_state/logging/status/json`.
4. Subscribe to `gps_state/logging/last/json`.
5. Subscribe to `gps_state/logging/validation/json`.
6. Wait for retained settings/status/last messages.
7. If required retained data is missing after the app timeout, publish `{}` to `gps_state/logging/set/renew/json`.
8. Enable Start/Stop/Cancel only after the first valid v2 status payload is parsed.

## Recommended screen layout

- **Header card:** `summary`, text/icon state, locally calculated duration and Start/Stop/Cancel.
- **Current request:** trigger, mode, target area and requested time.
- **Current session:** session ID, start time and file count.
- **Last session:** result, duration, stop reason and files.
- **Defaults:** trigger, mode and default area.
- **Expert section:** PID, RAM path, output path, script path, container name and raw error.

## Button logic

```text
if no valid v2 status has been received:
    disable all runtime actions
else if runtime.running:
    primary action = Stop
else if runtime.armed or runtime.request_active:
    primary action = Cancel
else:
    primary action = Start
```

After publishing a command, do not optimistically change the runtime state. Show a pending indicator and wait for validation plus the confirmed status transition.

## Breaking changes from v0.1

- Removed all MQTT aliases below `settings/mower_logic/satellite_logging/*`.
- Removed the `satellite_logging_enabled` configuration field.
- Removed `runtime.request_origin`.
- Removed `implementation.legacy_setting_enabled`.
- Removed duplicated top-level runtime fields.
- Bumped status and last schemas to v2.

The app must be updated atomically with this backend package. There is no fallback path in the backend.
