#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#[path = "../example_common.rs"]
mod example_common;
use example_common::*;

use core::mem;
use cortex_m_rt::entry;
use defmt::info;

use nrf_softdevice::ble::gatt_server::{Characteristic, CharacteristicHandles, RegisterError};
use nrf_softdevice::ble::{gatt_server, peripheral, Connection, Uuid};
use nrf_softdevice::{raw, RawError, Softdevice};

use embassy::executor::{task, Executor};
use embassy::util::Forever;
static EXECUTOR: Forever<Executor> = Forever::new();

#[task]
async fn softdevice_task(sd: &'static Softdevice) {
    sd.run().await;
}

#[nrf_softdevice::gatt_server(uuid = "180f")]
struct BatteryService {
    #[characteristic(uuid = "2a19", read, write, notify)]
    battery_level: u8,
}

#[task]
async fn bluetooth_task(sd: &'static Softdevice) {
    let server: BatteryService = gatt_server::register(sd).dewrap();
    #[rustfmt::skip]
    let adv_data = &[
        0x02, 0x01, raw::BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE as u8,
        0x03, 0x03, 0x09, 0x18,
        0x0a, 0x09, b'H', b'e', b'l', b'l', b'o', b'R', b'u', b's', b't',
    ];
    #[rustfmt::skip]
    let scan_data = &[
        0x03, 0x03, 0x09, 0x18,
    ];

    loop {
        let conn = peripheral::advertise(
            sd,
            peripheral::ConnectableAdvertisement::ScannableUndirected {
                adv_data,
                scan_data,
            },
        )
        .await
        .dewrap();

        info!("advertising done!");

        // Run the GATT server on the connection. This returns when the connection gets disconnected.
        let res = gatt_server::run(&conn, &server, |e| match e {
            BatteryServiceEvent::BatteryLevelWrite(val) => {
                info!("wrote battery level: {:u8}", val);
                if let Err(e) = server.battery_level_notify(&conn, val + 1) {
                    info!("send notification error: {:?}", e);
                }
            }
            BatteryServiceEvent::BatteryLevelNotificationsEnabled => {
                info!("battery notifications enabled")
            }
            BatteryServiceEvent::BatteryLevelNotificationsDisabled => {
                info!("battery notifications disabled")
            }
        })
        .await;

        if let Err(e) = res {
            info!("gatt_server run exited with error: {:?}", e);
        }
    }
}

#[entry]
fn main() -> ! {
    info!("Hello World!");

    let config = nrf_softdevice::Config {
        clock: Some(raw::nrf_clock_lf_cfg_t {
            source: raw::NRF_CLOCK_LF_SRC_XTAL as u8,
            rc_ctiv: 0,
            rc_temp_ctiv: 0,
            accuracy: 7,
        }),
        conn_gap: Some(raw::ble_gap_conn_cfg_t {
            conn_count: 6,
            event_length: 6,
        }),
        conn_gatt: Some(raw::ble_gatt_conn_cfg_t { att_mtu: 128 }),
        gatts_attr_tab_size: Some(raw::ble_gatts_cfg_attr_tab_size_t {
            attr_tab_size: 32768,
        }),
        gap_role_count: Some(raw::ble_gap_cfg_role_count_t {
            adv_set_count: 1,
            periph_role_count: 3,
            central_role_count: 3,
            central_sec_count: 0,
            _bitfield_1: raw::ble_gap_cfg_role_count_t::new_bitfield_1(0),
        }),
        gap_device_name: Some(raw::ble_gap_cfg_device_name_t {
            p_value: b"HelloRust" as *const u8 as _,
            current_len: 9,
            max_len: 9,
            write_perm: unsafe { mem::zeroed() },
            _bitfield_1: raw::ble_gap_cfg_device_name_t::new_bitfield_1(
                raw::BLE_GATTS_VLOC_STACK as u8,
            ),
        }),
        ..Default::default()
    };

    let (sdp, p) = take_peripherals();
    let sd = Softdevice::enable(sdp, &config);

    let executor = EXECUTOR.put(Executor::new(cortex_m::asm::sev));
    executor.spawn(softdevice_task(sd)).dewrap();
    executor.spawn(bluetooth_task(sd)).dewrap();

    loop {
        executor.run();
        cortex_m::asm::wfe();
    }
}
