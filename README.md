# Eredin

For now, custom low level control of multirotor drones in Rust. HITL support soon.

## Building

### Eredin Core (STM32)
```bash
# Install and build code
cargo build --features run-deploy # For normal build
cargo build --features run-hitl   # For RTT logging

# Deploy to board (using STLINKV2, needs install of probe-rs)
cd eredin_core
probe-rs run --chip STM32H743ZI target/thumbv7em-none-eabihf/debug/eredin_core
```

### Eredin Comms (ESP32)
```bash
# Install and build code (needs pio)
pio run -e eredin_comms --upload-port /dev/ttyUSB0 --target upload
```
Note: Be sure to include the secrets.h file in pio /include to set the wifi credentials

## Logging

### RTT core logging:
```bash
# RTT session should start automatically with run-deploy
# If not on the session, reattach
probe-rs attach --chip STM32H743ZI target/thumbv7em-none-eabihf/debug/eredin_core
```

### Wifi telemetry logging:
```bash
# Create wifi tty socket for esp comms
sudo socat -d -d pty,raw,echo=0,link=/tmp/ttyV0 tcp:192.168.243.73:3333
# Open tty 
sudo tio /tmp/ttyV0 -b 115200
```

### Troubleshooting
* Probe-rs probe not found: Install [udev rules](https://probe.rs/docs/getting-started/probe-setup/#linux%3A-udev-rules)
