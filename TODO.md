# Laundry Room Controller — TODO

## Mechanical / Install
- [ ] Design wall-mount enclosure for the controller board
- [ ] Wire relay output to the fan's power socket
- [ ] Install the Tuya smart switch
- [ ] Build small box for the IR transmitter, mount near the fan (LED aimed at receiver window)

## Firmware
- [ ] Radar: implement energy-per-range logic (user's MD with findings to follow)
- [ ] Tuya device at 192.168.1.153 unreachable — verify powered/online, check IP hasn't changed
- [ ] Flash size header 2MB vs 4MB chip — fix in menuconfig if more space needed

## Web GUI
- [ ] Facelift pass on dashboard

## Done
- [x] IR remote protocol fully decoded & verified — 12-bit duty-cycle code, all commands (on/off/timer/mode/swing) work
- [x] Carrier gating + sacrificial tail quirks worked around (see AGENTS.md)
- [x] Sensors, Wi-Fi, BT SPP, HTTP, mDNS, CLI all up
