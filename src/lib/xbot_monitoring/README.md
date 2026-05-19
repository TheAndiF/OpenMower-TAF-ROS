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
- `settings/mow_load_factor/validation/json`

The validation payload reports the namespace, write mode (`session` or `persistent`), accepted keys, and rejected keys with rejection reasons. Valid keys from a mixed payload are applied; invalid or unknown keys are reported as rejected.
