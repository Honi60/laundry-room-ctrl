---
agent: devin-local
session: poised-saltopus
created: 2026-09-18T14:14:15Z
---
# Fix ESP-IDF extension not recognizing the project in Devin IDE

The ESP-IDF extension activated before the project was detectable and never completed setup resolution; a window reload (plus an optional activation-mode setting) should restore the Commands tree and status bar tools.

## Summary

The `espressif.esp-idf-extension` v2.2.0 is installed in Devin and its view container is registered (the ESP-IDF icon appears in the activity bar), but the Commands tree shows "no data provider registered" and no status bar tools appear — meaning `activate()` early-returned before registering the UI.

**Evidence from logs** (`~/.config/Devin/logs/20260917T234325/window1/exthost/exthost.log` + `~/.devin/extensions/espressif.esp-idf-extension-2.2.0-universal/esp_idf_vsc_ext.log`):

- Extension activated at 00:16 via `onLanguage:kconfig` (sdkconfig opened), but `Vd` (setup resolution) never ran — no log entries after that point.
- Default `idf.extensionActivationMode` is `"detect"`: the extension reads each workspace folder's root `CMakeLists.txt` and requires the literal string `include($ENV{IDF_PATH}/tools/cmake/project.cmake)`. If not found it shows "Activate Anyway?" and returns without registering UI if dismissed.
- `CMakeLists.txt` now contains the marker (line 3), and `.vscode/settings.json` (created 16:47 today) contains a valid `idf.currentSetup` = `/home/honi/esp/esp-idf` which matches `~/.espressif/esp_idf.json` → `idfInstalled` → path `/home/honi/esp/esp-idf` (IDF 5.5.0, python env `idf5.5_py3.12_env` — all present and verified).
- The missing `~/.espressif/tools/eim_idf.json` error and the `esp-clang` stderr warning in the log are both non-fatal fallbacks/warnings.

## Implementation Steps

1. **Reload the window** — `Ctrl+Shift+P` → `Developer: Reload Window` (or restart Devin). On re-activation, project detection will pass (CMakeLists.txt marker exists) and `idf.currentSetup` will match the installed setup, so the Commands tree, status bar buttons (port/target/build/flash/monitor) and `ESP-IDF: Open ESP-IDF Terminal` become available.

2. **Optional hardening** — add to `.vscode/settings.json` so activation never depends on the detect heuristic:
   ```json
   "idf.extensionActivationMode": "always"
   ```

## Files to Modify

- `/home/honi/Documents/ESP32_projects/laundry_room_ctrl/.vscode/settings.json` — optionally add `idf.extensionActivationMode: "always"` (not strictly required; detect mode will work now that CMakeLists.txt is correct).

## Verification

- [ ] After reload: ESP-IDF Explorer → Commands tree lists commands (Build, Flash, Monitor, etc.)
- [ ] Status bar shows ESP-IDF items (port, target `esp32`, flash type, build/flash/monitor icons)
- [ ] `Ctrl+Shift+P` → `ESP-IDF: Doctor Command` reports the setup as valid
- [ ] If still broken: check Output → "ESP-IDF" channel and tail `esp_idf_vsc_ext.log` for the new error

## Risks/Considerations

- `idf.port` is `/dev/ttyUSB0` — only relevant when flashing; adjust via the status bar port selector if the board enumerates differently.
- If after reload the UI is still absent, the remaining suspect is an unsupported API in this IDE fork (e.g. `vscode.lm.registerTool`); the next diagnostic step would be the extension's Output channel, which currently has no entries post-activation — consistent with an early return rather than a crash (no "activating extension failed" in exthost.log).
