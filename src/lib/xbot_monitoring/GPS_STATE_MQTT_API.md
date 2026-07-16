# OpenMower-TAF GPS-State MQTT API

Version: 1.0  
Date: 2026-07-15

## Canonical tree

```text
gps_state/
  settings/json
  settings/set/session/json
  settings/set/persistent/json
  settings/set/renew/json
  settings/validation/json
  state1/{definition,status,request}
  state2/{definition,status,satellites,request}
  state3/{definition,status,request}
  state4/{definition,status,request}
  logging/set/control/json
  logging/set/renew/json
  logging/{validation,status,last}/json
  restart/set/json
  restart/set/renew/json
  restart/{validation,status,last}/json
```

No State0, State01, flat state aliases or legacy logging/restart aliases are supported.

## State semantics

- `state1`: retained, continuously updated drive-readiness decision and full decision chain. Its status also exposes `current_status` and `gps_quality`; `gps_quality` mirrors the `state2` field `quality_class`.
- `state2`: lease-controlled GNSS, RTK and pose diagnostics; its large satellite list is separate.
- `state3`: only satellites with `used=true`.
- `state4`: all visible satellites, with `used=true` or `used=false`.

### State1 compact status fields

The retained `gps_state/state1/status` payload contains two additional operator-facing fields inside `data`:

- `current_status`: current mower operating state from `robot_state/current_state`, normalized to lowercase (for example `mowing`, `idle`, `docking` or `unknown`).
- `gps_quality`: GNSS quality class copied from State2 `quality_class` (`unavailable`, `poor`, `fair`, `good` or `very_good`).

This keeps the most important status and quality values available without activating the lease-controlled State2 view.

Satellite objects use the canonical fields `system`, `gnss_id`, `sv`, `used`, `visible`, `cn0_dbhz`, `elevation_deg`, `azimuth_deg`, `pr_res` and `quality`.

## Lease control

Activate or renew:

```json
{"command":"set_active","state":"state3","active":true,"request_id":"app-state3-1","interval_ms":1000,"lease_ms":15000}
```

Deactivate:

```json
{"command":"set_active","state":"state3","active":false,"request_id":"app-state3-1"}
```

One-shot refresh:

```json
{"command":"publish_now","state":"state3","request_id":"app-state3-1"}
```

Each request ID owns an independent lease. Publishing stops only after all leases for the state have ended. State2-State4 status payloads are not retained.

## Settings

The settings payload exposes `state2_default_interval_ms`, `state3_default_interval_ms`, `state4_default_interval_ms` and `activation_default_lease_ms`. Session and persistent writes use the standard GPS settings endpoints. These defaults do not activate a state.

## Logging and restart

Logging and restart are actions rather than settings. Their defaults and paths are represented in `gps_state/settings/json`, while commands use their dedicated `set` topics. Immediate validation, retained runtime status and retained last-result records are separate.
