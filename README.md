# Eredin

For now, custom low level control of multirotor drones in Rust. HITL support soon.

## Building

### Eredin Core (STM32)
```bash
# Install and build code
cargo build

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
```bash
# Create wifi tty socket for esp comms
sudo socat -d -d pty,raw,echo=0,link=/tmp/ttyV0 tcp:192.168.243.73:3333
# Open tty 
sudo tio /tmp/ttyV0 -b 115200
```
