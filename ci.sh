#!/bin/bash

set -euxo pipefail

# build examples
#==================

(cd examples; cargo build --target thumbv7em-none-eabihf --bins)


# build with log/defmt combinations
#=====================================

cd nrf-softdevice

cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-sec,ble-central,ble-peripheral,ble-l2cap,ble-gatt-client,ble-gatt-server
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-sec,ble-central,ble-peripheral,ble-l2cap,ble-gatt-client,ble-gatt-server,defmt
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-sec,ble-central,ble-peripheral,ble-l2cap,ble-gatt-client,ble-gatt-server,log


# build softdevice+chip combinations (with all supported features enabled)
# This htis each softdevice and each chip at least once.
#================================================================================

cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s112,nrf52805,ble-sec,ble-peripheral,ble-gatt-client,ble-gatt-server
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s113,nrf52810,ble-sec,ble-peripheral,ble-l2cap,ble-gatt-client,ble-gatt-server
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s122,nrf52833,ble-central,ble-gatt-client,ble-gatt-server
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s132,nrf52832,ble-sec,ble-central,ble-peripheral,ble-l2cap,ble-gatt-client,ble-gatt-server
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52811,ble-sec,ble-central,ble-peripheral,ble-l2cap,ble-gatt-client,ble-gatt-server
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52820,ble-sec,ble-central,ble-peripheral,ble-l2cap,ble-gatt-client,ble-gatt-server
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-sec,ble-central,ble-peripheral,ble-l2cap,ble-gatt-client,ble-gatt-server


# build all feature combinations
#==================================

cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-peripheral
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-peripheral,ble-gatt-server
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-peripheral,ble-gatt-client
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-peripheral,ble-gatt-client,ble-gatt-server
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-peripheral,ble-l2cap
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-peripheral,ble-l2cap,ble-gatt-server
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-peripheral,ble-l2cap,ble-gatt-client
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-peripheral,ble-l2cap,ble-gatt-client,ble-gatt-server

cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-central
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-central,ble-gatt-server
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-central,ble-gatt-client
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-central,ble-gatt-client,ble-gatt-server
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-central,ble-l2cap
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-central,ble-l2cap,ble-gatt-server
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-central,ble-l2cap,ble-gatt-client
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-central,ble-l2cap,ble-gatt-client,ble-gatt-server

cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-central,ble-peripheral
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-central,ble-peripheral,ble-gatt-server
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-central,ble-peripheral,ble-gatt-client
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-central,ble-peripheral,ble-gatt-client,ble-gatt-server
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-central,ble-peripheral,ble-l2cap
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-central,ble-peripheral,ble-l2cap,ble-gatt-server
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-central,ble-peripheral,ble-l2cap,ble-gatt-client
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-central,ble-peripheral,ble-l2cap,ble-gatt-client,ble-gatt-server
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-central,ble-peripheral,ble-l2cap,ble-gatt-client,ble-gatt-server,ble-l2cap-credit-wrokaround,ble-rssi

cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-sec,ble-peripheral
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-sec,ble-peripheral,ble-gatt-server
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-sec,ble-peripheral,ble-gatt-client
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-sec,ble-peripheral,ble-gatt-client,ble-gatt-server
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-sec,ble-peripheral,ble-l2cap
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-sec,ble-peripheral,ble-l2cap,ble-gatt-server
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-sec,ble-peripheral,ble-l2cap,ble-gatt-client
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-sec,ble-peripheral,ble-l2cap,ble-gatt-client,ble-gatt-server

cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-sec,ble-central,ble-peripheral
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-sec,ble-central,ble-peripheral,ble-gatt-server
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-sec,ble-central,ble-peripheral,ble-gatt-client
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-sec,ble-central,ble-peripheral,ble-gatt-client,ble-gatt-server
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-sec,ble-central,ble-peripheral,ble-l2cap
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-sec,ble-central,ble-peripheral,ble-l2cap,ble-gatt-server
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-sec,ble-central,ble-peripheral,ble-l2cap,ble-gatt-client
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-sec,ble-central,ble-peripheral,ble-l2cap,ble-gatt-client,ble-gatt-server
cargo build --target thumbv7em-none-eabihf -p nrf-softdevice --features s140,nrf52840,ble-sec,ble-central,ble-peripheral,ble-l2cap,ble-gatt-client,ble-gatt-server,ble-l2cap-credit-wrokaround,ble-rssi
