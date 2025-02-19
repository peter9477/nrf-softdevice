//! Generic Attribute client. GATT clients consume functionality offered by GATT servers.

#![allow(unused_mut, unused_variables)]

use heapless::Vec;

use crate::ble::*;
use crate::util::{get_flexarray, get_union_field, Portal};
use crate::{raw, RawError};

/// Discovered characteristic
pub struct Characteristic {
    pub uuid: Option<Uuid>,
    pub handle_decl: u16,
    pub handle_value: u16,
    pub props: raw::ble_gatt_char_props_t,
    pub has_ext_props: bool,
}

/// Discovered descriptor
pub struct Descriptor {
    pub uuid: Option<Uuid>,
    pub handle: u16,
}

#[repr(u8)]
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum HvxType {
    Invalid = 0,
    Notification,
    Indication,
}

pub struct InvalidHvxTypeError;

impl TryFrom<u8> for HvxType {
    type Error = InvalidHvxTypeError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match u32::from(value) {
            raw::BLE_GATT_HVX_INVALID => Ok(HvxType::Invalid),
            raw::BLE_GATT_HVX_NOTIFICATION => Ok(HvxType::Notification),
            raw::BLE_GATT_HVX_INDICATION => Ok(HvxType::Indication),
            _ => Err(InvalidHvxTypeError),
        }
    }
}

/// Trait for implementing GATT clients.
pub trait Client {
    type Event;

    /// Handles notification and indication events from the GATT server.
    fn on_hvx(&self, conn: &Connection, type_: HvxType, handle: u16, data: &[u8]) -> Option<Self::Event>;

    /// Get the UUID of the GATT service. This is used by [`discover`] to search for the
    /// service in the GATT server.
    fn uuid() -> Uuid;

    /// Create a new instance in a "not-yet-discovered" state.
    fn new_undiscovered(conn: Connection) -> Self;

    /// Called by [`discover`] for every discovered characteristic. Implementations must
    /// check if they're interested in the UUID of the characteristic, and save their
    /// handles if needed.
    fn discovered_characteristic(&mut self, characteristic: &Characteristic, descriptors: &[Descriptor]);

    /// Called by [`discover`] at the end of the discovery procedure. Implementations must check
    /// that all required characteristics have been discovered, and return [`DiscoverError::ServiceIncomplete`]
    /// otherwise.
    ///
    /// If no error is returned, this instance is considered ready to use and is returned to
    /// the caller of [`discover`]
    fn discovery_complete(&mut self) -> Result<(), DiscoverError>;
}

/// Error type for [`discover`]
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DiscoverError {
    /// Connection is disconnected.
    Disconnected,
    /// No service with the given UUID found in the server.
    ServiceNotFound,
    /// Service with the given UUID found, but it's missing some required characteristics.
    ServiceIncomplete,
    Gatt(GattError),
    Raw(RawError),
}

impl From<DisconnectedError> for DiscoverError {
    fn from(_: DisconnectedError) -> Self {
        Self::Disconnected
    }
}

impl From<GattError> for DiscoverError {
    fn from(err: GattError) -> Self {
        Self::Gatt(err)
    }
}

impl From<RawError> for DiscoverError {
    fn from(err: RawError) -> Self {
        Self::Raw(err)
    }
}

const DISC_CHARS_MAX: usize = 6;
const DISC_DESCS_MAX: usize = 6;

pub(crate) async fn discover_service(conn: &Connection, uuid: Uuid) -> Result<raw::ble_gattc_service_t, DiscoverError> {
    let conn_handle = conn.with_state(|state| state.check_connected())?;
    let ret = unsafe { raw::sd_ble_gattc_primary_services_discover(conn_handle, 1, uuid.as_raw_ptr()) };
    RawError::convert(ret).map_err(|err| {
        warn!("sd_ble_gattc_primary_services_discover err {:?}", err);
        err
    })?;

    portal(conn_handle)
        .wait_once(|ble_evt| unsafe {
            match (*ble_evt).header.evt_id as u32 {
                raw::BLE_GAP_EVTS_BLE_GAP_EVT_DISCONNECTED => return Err(DiscoverError::Disconnected),
                raw::BLE_GATTC_EVTS_BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP => {
                    let gattc_evt = check_status(ble_evt)?;
                    let params = get_union_field(ble_evt, &gattc_evt.params.prim_srvc_disc_rsp);
                    let v = get_flexarray(ble_evt, &params.services, params.count as usize);

                    match v.len() {
                        0 => Err(DiscoverError::ServiceNotFound),
                        1 => Ok(v[0]),
                        _n => {
                            warn!(
                                "Found {:?} services with the same UUID, using the first one",
                                params.count
                            );
                            Ok(v[0])
                        }
                    }
                }
                e => panic!("unexpected event {}", e),
            }
        })
        .await
}

// =============================

async fn discover_characteristics(
    conn: &Connection,
    start_handle: u16,
    end_handle: u16,
) -> Result<Vec<raw::ble_gattc_char_t, DISC_CHARS_MAX>, DiscoverError> {
    let conn_handle = conn.with_state(|state| state.check_connected())?;

    let ret = unsafe {
        raw::sd_ble_gattc_characteristics_discover(
            conn_handle,
            &raw::ble_gattc_handle_range_t {
                start_handle,
                end_handle,
            },
        )
    };
    RawError::convert(ret).map_err(|err| {
        warn!("sd_ble_gattc_characteristics_discover err {:?}", err);
        err
    })?;

    portal(conn_handle)
        .wait_once(|ble_evt| unsafe {
            match (*ble_evt).header.evt_id as u32 {
                raw::BLE_GAP_EVTS_BLE_GAP_EVT_DISCONNECTED => return Err(DiscoverError::Disconnected),
                raw::BLE_GATTC_EVTS_BLE_GATTC_EVT_CHAR_DISC_RSP => {
                    let gattc_evt = check_status(ble_evt)?;
                    let params = get_union_field(ble_evt, &gattc_evt.params.char_disc_rsp);
                    let v = get_flexarray(ble_evt, &params.chars, params.count as usize);
                    let v = Vec::from_slice(v)
                        .unwrap_or_else(|_| panic!("too many gatt chars, increase DiscCharsMax: {:?}", v.len()));
                    Ok(v)
                }
                e => panic!("unexpected event {}", e),
            }
        })
        .await
}

// =============================

async fn discover_descriptors(
    conn: &Connection,
    start_handle: u16,
    end_handle: u16,
) -> Result<Vec<raw::ble_gattc_desc_t, DISC_DESCS_MAX>, DiscoverError> {
    let conn_handle = conn.with_state(|state| state.check_connected())?;

    let ret = unsafe {
        raw::sd_ble_gattc_descriptors_discover(
            conn_handle,
            &raw::ble_gattc_handle_range_t {
                start_handle,
                end_handle,
            },
        )
    };
    RawError::convert(ret).map_err(|err| {
        warn!("sd_ble_gattc_descriptors_discover err {:?}", err);
        err
    })?;

    portal(conn_handle)
        .wait_once(|ble_evt| unsafe {
            match (*ble_evt).header.evt_id as u32 {
                raw::BLE_GAP_EVTS_BLE_GAP_EVT_DISCONNECTED => return Err(DiscoverError::Disconnected),
                raw::BLE_GATTC_EVTS_BLE_GATTC_EVT_DESC_DISC_RSP => {
                    let gattc_evt = check_status(ble_evt)?;
                    let params = get_union_field(ble_evt, &gattc_evt.params.desc_disc_rsp);
                    let v = get_flexarray(ble_evt, &params.descs, params.count as usize);
                    let v = Vec::from_slice(v)
                        .unwrap_or_else(|_| panic!("too many gatt descs, increase DiscDescsMax: {:?}", v.len()));
                    Ok(v)
                }
                e => panic!("unexpected event {}", e),
            }
        })
        .await
}

// =============================

async fn discover_inner<T: Client>(
    conn: &Connection,
    client: &mut T,
    svc: &raw::ble_gattc_service_t,
    curr: raw::ble_gattc_char_t,
    next: Option<raw::ble_gattc_char_t>,
) -> Result<(), DiscoverError> {
    // Calcuate range of possible descriptors
    let start_handle = curr.handle_value + 1;
    let end_handle = next.map(|c| c.handle_decl - 1).unwrap_or(svc.handle_range.end_handle);

    let characteristic = Characteristic {
        uuid: Uuid::from_raw(curr.uuid),
        handle_decl: curr.handle_decl,
        handle_value: curr.handle_value,
        has_ext_props: curr.char_ext_props() != 0,
        props: curr.char_props,
    };

    let mut descriptors: Vec<Descriptor, DISC_DESCS_MAX> = Vec::new();

    // Only if range is non-empty, discover. (if it's empty there must be no descriptors)
    if start_handle <= end_handle {
        let descs = {
            match discover_descriptors(conn, start_handle, end_handle).await {
                Ok(descs) => descs,
                Err(DiscoverError::Gatt(GattError::ATTERR_ATTRIBUTE_NOT_FOUND)) => Vec::new(),
                Err(err) => return Err(err),
            }
        };
        for desc in descs {
            descriptors
                .push(Descriptor {
                    uuid: Uuid::from_raw(desc.uuid),
                    handle: desc.handle,
                })
                .unwrap_or_else(|_| panic!("no size in descriptors"));
        }
    }

    client.discovered_characteristic(&characteristic, &descriptors[..]);

    Ok(())
}

/// Discover a service in the peer's GATT server and construct a Client instance
/// to use it.
pub async fn discover<T: Client>(conn: &Connection) -> Result<T, DiscoverError> {
    // TODO handle drop. Probably doable gracefully (no DropBomb)

    let svc = match discover_service(conn, T::uuid()).await {
        Err(DiscoverError::Gatt(GattError::ATTERR_ATTRIBUTE_NOT_FOUND)) => Err(DiscoverError::ServiceNotFound),
        x => x,
    }?;

    let mut client = T::new_undiscovered(conn.clone());

    let mut curr_handle = svc.handle_range.start_handle;
    let end_handle = svc.handle_range.end_handle;

    let mut prev_char: Option<raw::ble_gattc_char_t> = None;
    while curr_handle < end_handle {
        let chars = match discover_characteristics(conn, curr_handle, end_handle).await {
            Err(DiscoverError::Gatt(GattError::ATTERR_ATTRIBUTE_NOT_FOUND)) => break,
            x => x,
        }?;
        assert_ne!(chars.len(), 0);
        for curr in chars {
            if let Some(prev) = prev_char {
                discover_inner(conn, &mut client, &svc, prev, Some(curr)).await?;
            }
            prev_char = Some(curr);
            curr_handle = curr.handle_value + 1;
        }
    }
    if let Some(prev) = prev_char {
        discover_inner(conn, &mut client, &svc, prev, None).await?;
    }

    client.discovery_complete()?;

    Ok(client)
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ReadError {
    Disconnected,
    Truncated,
    Gatt(GattError),
    Raw(RawError),
}

impl From<DisconnectedError> for ReadError {
    fn from(_: DisconnectedError) -> Self {
        Self::Disconnected
    }
}

impl From<GattError> for ReadError {
    fn from(err: GattError) -> Self {
        Self::Gatt(err)
    }
}

impl From<RawError> for ReadError {
    fn from(err: RawError) -> Self {
        Self::Raw(err)
    }
}

pub async fn read(conn: &Connection, handle: u16, buf: &mut [u8]) -> Result<usize, ReadError> {
    let conn_handle = conn.with_state(|state| state.check_connected())?;

    let ret = unsafe { raw::sd_ble_gattc_read(conn_handle, handle, 0) };
    RawError::convert(ret).map_err(|err| {
        warn!("sd_ble_gattc_read err {:?}", err);
        err
    })?;

    portal(conn_handle)
        .wait_many(|ble_evt| unsafe {
            match (*ble_evt).header.evt_id as u32 {
                raw::BLE_GAP_EVTS_BLE_GAP_EVT_DISCONNECTED => return Some(Err(ReadError::Disconnected)),
                raw::BLE_GATTC_EVTS_BLE_GATTC_EVT_READ_RSP => {
                    let gattc_evt = match check_status(ble_evt) {
                        Ok(evt) => evt,
                        Err(e) => return Some(Err(e.into())),
                    };
                    let params = get_union_field(ble_evt, &gattc_evt.params.read_rsp);
                    let v = get_flexarray(ble_evt, &params.data, params.len as usize);
                    let len = core::cmp::min(v.len(), buf.len());
                    buf[..len].copy_from_slice(&v[..len]);

                    if v.len() > buf.len() {
                        return Some(Err(ReadError::Truncated));
                    }
                    Some(Ok(len))
                }
                _ => None,
            }
        })
        .await
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WriteError {
    Disconnected,
    Timeout,
    Gatt(GattError),
    Raw(RawError),
}

impl From<DisconnectedError> for WriteError {
    fn from(_: DisconnectedError) -> Self {
        Self::Disconnected
    }
}

impl From<GattError> for WriteError {
    fn from(err: GattError) -> Self {
        Self::Gatt(err)
    }
}

impl From<RawError> for WriteError {
    fn from(err: RawError) -> Self {
        Self::Raw(err)
    }
}

pub async fn write(conn: &Connection, handle: u16, buf: &[u8]) -> Result<(), WriteError> {
    let conn_handle = conn.with_state(|state| state.check_connected())?;

    assert!(buf.len() <= u16::MAX as usize);
    let params = raw::ble_gattc_write_params_t {
        write_op: raw::BLE_GATT_OP_WRITE_REQ as u8,
        flags: 0,
        handle,
        p_value: buf.as_ptr(),
        len: buf.len() as u16,
        offset: 0,
    };

    let ret = unsafe { raw::sd_ble_gattc_write(conn_handle, &params) };
    RawError::convert(ret).map_err(|err| {
        warn!("sd_ble_gattc_write err {:?}", err);
        err
    })?;

    portal(conn_handle)
        .wait_many(|ble_evt| unsafe {
            match (*ble_evt).header.evt_id as u32 {
                raw::BLE_GAP_EVTS_BLE_GAP_EVT_DISCONNECTED => return Some(Err(WriteError::Disconnected)),
                raw::BLE_GATTC_EVTS_BLE_GATTC_EVT_WRITE_RSP => {
                    match check_status(ble_evt) {
                        Ok(_) => {}
                        Err(e) => return Some(Err(e.into())),
                    };
                    Some(Ok(()))
                }
                raw::BLE_GATTC_EVTS_BLE_GATTC_EVT_TIMEOUT => {
                    return Some(Err(WriteError::Timeout));
                }
                _ => None,
            }
        })
        .await
}

pub async fn write_without_response(conn: &Connection, handle: u16, buf: &[u8]) -> Result<(), WriteError> {
    loop {
        let conn_handle = conn.with_state(|state| state.check_connected())?;

        assert!(buf.len() <= u16::MAX as usize);
        let params = raw::ble_gattc_write_params_t {
            write_op: raw::BLE_GATT_OP_WRITE_CMD as u8,
            flags: 0,
            handle,
            p_value: buf.as_ptr(),
            len: buf.len() as u16,
            offset: 0,
        };

        let ret = unsafe { raw::sd_ble_gattc_write(conn_handle, &params) };
        match RawError::convert(ret) {
            Err(RawError::Resources) => {}
            Err(e) => return Err(e.into()),
            Ok(()) => return Ok(()),
        }

        portal(conn_handle)
            .wait_many(|ble_evt| unsafe {
                match (*ble_evt).header.evt_id as u32 {
                    raw::BLE_GAP_EVTS_BLE_GAP_EVT_DISCONNECTED => return Some(Err(WriteError::Disconnected)),
                    raw::BLE_GATTC_EVTS_BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE => Some(Ok(())),
                    _ => None,
                }
            })
            .await?;
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TryWriteError {
    Disconnected,
    BufferFull,
    Gatt(GattError),
    Raw(RawError),
}

impl From<DisconnectedError> for TryWriteError {
    fn from(_: DisconnectedError) -> Self {
        Self::Disconnected
    }
}

impl From<GattError> for TryWriteError {
    fn from(err: GattError) -> Self {
        Self::Gatt(err)
    }
}

impl From<RawError> for TryWriteError {
    fn from(err: RawError) -> Self {
        Self::Raw(err)
    }
}

pub fn try_write_without_response(conn: &Connection, handle: u16, buf: &[u8]) -> Result<(), TryWriteError> {
    let conn_handle = conn.with_state(|state| state.check_connected())?;

    assert!(buf.len() <= u16::MAX as usize);
    let params = raw::ble_gattc_write_params_t {
        write_op: raw::BLE_GATT_OP_WRITE_CMD as u8,
        flags: 0,
        handle,
        p_value: buf.as_ptr(),
        len: buf.len() as u16,
        offset: 0,
    };

    let ret = unsafe { raw::sd_ble_gattc_write(conn_handle, &params) };
    match RawError::convert(ret) {
        Err(RawError::Resources) => Err(TryWriteError::BufferFull),
        Err(e) => Err(e.into()),
        Ok(()) => Ok(()),
    }
}

unsafe fn check_status(ble_evt: *const raw::ble_evt_t) -> Result<&'static raw::ble_gattc_evt_t, GattError> {
    let gattc_evt = get_union_field(ble_evt, &(*ble_evt).evt.gattc_evt);
    GattStatus::from(gattc_evt.gatt_status).to_result().and(Ok(gattc_evt))
}

pub(crate) unsafe fn on_evt(ble_evt: *const raw::ble_evt_t) {
    let gattc_evt = get_union_field(ble_evt, &(*ble_evt).evt.gattc_evt);

    #[cfg(not(feature = "ble-gatt-client-flex"))]
    if (*ble_evt).header.evt_id == raw::BLE_GATTC_EVTS_BLE_GATTC_EVT_HVX as u16 {
        hvx_portal(gattc_evt.conn_handle).call(ble_evt);
    } else {
        portal(gattc_evt.conn_handle).call(ble_evt);
    }

    #[cfg(feature = "ble-gatt-client-flex")]
    portal(gattc_evt.conn_handle).call(ble_evt);
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MtuExchangeError {
    /// Connection is disconnected.
    Disconnected,
    Gatt(GattError),
    Raw(RawError),
}

impl From<DisconnectedError> for MtuExchangeError {
    fn from(_: DisconnectedError) -> Self {
        Self::Disconnected
    }
}

impl From<GattError> for MtuExchangeError {
    fn from(err: GattError) -> Self {
        Self::Gatt(err)
    }
}

impl From<RawError> for MtuExchangeError {
    fn from(err: RawError) -> Self {
        Self::Raw(err)
    }
}

#[cfg(feature = "ble-central")]
pub(crate) async fn att_mtu_exchange(conn: &Connection, mtu: u16) -> Result<(), MtuExchangeError> {
    let conn_handle = conn.with_state(|state| state.check_connected())?;

    let current_mtu = conn.with_state(|state| state.att_mtu);

    if current_mtu >= mtu {
        debug!(
            "att mtu exchange: want mtu {:?}, already got {:?}. Doing nothing.",
            mtu, current_mtu
        );
        return Ok(());
    }

    debug!(
        "att mtu exchange: want mtu {:?}, got only {:?}, doing exchange...",
        mtu, current_mtu
    );

    let ret = unsafe { raw::sd_ble_gattc_exchange_mtu_request(conn_handle, mtu) };
    if let Err(err) = RawError::convert(ret) {
        warn!("sd_ble_gattc_exchange_mtu_request err {:?}", err);
        return Err(err.into());
    }

    portal(conn_handle)
        .wait_once(|ble_evt| unsafe {
            match (*ble_evt).header.evt_id as u32 {
                raw::BLE_GAP_EVTS_BLE_GAP_EVT_DISCONNECTED => return Err(MtuExchangeError::Disconnected),
                raw::BLE_GATTC_EVTS_BLE_GATTC_EVT_EXCHANGE_MTU_RSP => {
                    let gattc_evt = match check_status(ble_evt) {
                        Ok(evt) => evt,
                        Err(e) => return Err(e.into()),
                    };
                    let params = get_union_field(ble_evt, &gattc_evt.params.exchange_mtu_rsp);
                    let mtu = params.server_rx_mtu;
                    debug!("att mtu exchange: got mtu {:?}", mtu);
                    conn.with_state(|state| state.att_mtu = mtu);

                    Ok(())
                }
                e => panic!("unexpected event {}", e),
            }
        })
        .await
}

const PORTAL_NEW: Portal<*const raw::ble_evt_t> = Portal::new();
static PORTALS: [Portal<*const raw::ble_evt_t>; CONNS_MAX] = [PORTAL_NEW; CONNS_MAX];
static HVX_PORTALS: [Portal<*const raw::ble_evt_t>; CONNS_MAX] = [PORTAL_NEW; CONNS_MAX];
pub(crate) fn portal(conn_handle: u16) -> &'static Portal<*const raw::ble_evt_t> {
    &PORTALS[conn_handle as usize]
}
pub(crate) fn hvx_portal(conn_handle: u16) -> &'static Portal<*const raw::ble_evt_t> {
    &HVX_PORTALS[conn_handle as usize]
}

pub async fn run<'a, F, C>(conn: &Connection, client: &C, mut f: F) -> DisconnectedError
where
    F: FnMut(C::Event),
    C: Client,
{
    let handle = match conn.with_state(|state| state.check_connected()) {
        Ok(handle) => handle,
        Err(e) => return e,
    };

    hvx_portal(handle)
        .wait_many(|ble_evt| unsafe {
            let ble_evt = &*ble_evt;
            if u32::from(ble_evt.header.evt_id) == raw::BLE_GAP_EVTS_BLE_GAP_EVT_DISCONNECTED {
                return Some(DisconnectedError);
            }

            // We have a GATTC event
            let gattc_evt = get_union_field(ble_evt, &ble_evt.evt.gattc_evt);
            let conn = unwrap!(Connection::from_handle(gattc_evt.conn_handle));
            let evt = match ble_evt.header.evt_id as u32 {
                raw::BLE_GATTC_EVTS_BLE_GATTC_EVT_HVX => {
                    let params = get_union_field(ble_evt, &gattc_evt.params.hvx);
                    let v = get_flexarray(ble_evt, &params.data, params.len as usize);
                    trace!(
                        "GATT_HVX write handle={:?} type={:?} data={:?}",
                        params.handle,
                        params.type_,
                        v
                    );

                    match params.type_.try_into() {
                        Ok(type_) => client.on_hvx(&conn, type_, params.handle, v),
                        Err(_) => {
                            error!("gatt_client invalid hvx type: {}", params.type_);
                            None
                        }
                    }
                }
                _ => None,
            };

            if let Some(evt) = evt {
                f(evt);
            }

            None
        })
        .await
}


//--------------------------------------------------
// "Flex" stuff is a replacement for the original client support,
// which was very limited and would panic in certain cases that
// can easily be seen in the field, among other limitations.

pub fn flex_discover_service(conn: &Connection, start_handle: u16, uuid: Option<Uuid>) -> Result<(), FlexClientError> {
    let conn_handle = conn.with_state(|state| state.check_connected())?;
    let ret = unsafe { raw::sd_ble_gattc_primary_services_discover(conn_handle, start_handle,
        match &uuid {
            Some(uuid) => uuid.as_raw_ptr(),
            None => core::ptr::null(),
        }
    ) };
    RawError::convert(ret).map_err(|err| {
        warn!("sd_ble_gattc_primary_services_discover err {:?}", err);
        err.into()
    })
}
 
pub fn flex_discover_characteristic(conn: &Connection, start_handle: u16, end_handle: u16) -> Result<(), FlexClientError> {
    let conn_handle = conn.with_state(|state| state.check_connected())?;
    let ret = unsafe {
        raw::sd_ble_gattc_characteristics_discover(
            conn_handle,
            &raw::ble_gattc_handle_range_t {
                start_handle,
                end_handle,
            },
        )
    };
    RawError::convert(ret).map_err(|err| {
        warn!("sd_ble_gattc_characteristics_discover err {:?}", err);
        err.into()
    })
}
 
pub fn flex_discover_descriptors(conn: &Connection, start_handle: u16, end_handle: u16) -> Result<(), FlexClientError> {
    let conn_handle = conn.with_state(|state| state.check_connected())?;
    let ret = unsafe {
        raw::sd_ble_gattc_descriptors_discover(
            conn_handle,
            &raw::ble_gattc_handle_range_t {
                start_handle,
                end_handle,
            },
        )
    };
    RawError::convert(ret).map_err(|err| {
        warn!("sd_ble_gattc_descriptors_discover err {:?}", err);
        err.into()
    })
}
 
pub fn flex_write(conn: &Connection, handle: u16, buf: &[u8]) -> Result<(), FlexClientError> {
    let conn_handle = conn.with_state(|state| state.check_connected())?;

    assert!(buf.len() <= u16::MAX as usize);
    let params = raw::ble_gattc_write_params_t {
        write_op: raw::BLE_GATT_OP_WRITE_REQ as u8,
        flags: 0,
        handle,
        p_value: buf.as_ptr(),
        len: buf.len() as u16,
        offset: 0,
    };

    let ret = unsafe { raw::sd_ble_gattc_write(conn_handle, &params) };
    RawError::convert(ret).map_err(|err| {
        warn!("sd_ble_gattc_write err {:?}", err);
        err.into()
    })
}
 
pub fn flex_att_mtu_exchange(conn: &Connection, mtu: u16) -> Result<(), FlexClientError> {
    let conn_handle = conn.with_state(|state| state.check_connected())?;

    let current_mtu = conn.with_state(|state| state.att_mtu);

    // You can't reduce the MTU (or change it once set) so this is a no-op.
    // Note that with the "flex" design this means you won't know whether
    // or not to expect an event.
    if current_mtu >= mtu {
        return Ok(());
    }

    let ret = unsafe { raw::sd_ble_gattc_exchange_mtu_request(conn_handle, mtu) };
    RawError::convert(ret).map_err(|err| {
        warn!("sd_ble_gattc_exchange_mtu_request err {:?}", err);
        err.into()
    })
}

/// Confirm receipt of an indication.
pub fn flex_hv_confirm(conn: &Connection, handle: u16) -> Result<(), FlexClientError> {
    let conn_handle = conn.with_state(|state| state.check_connected())?;
    let ret = unsafe { raw::sd_ble_gattc_hv_confirm(conn_handle, handle) };
    RawError::convert(ret).map_err(|err| {
        warn!("sd_ble_gattc_hv_confirm err {:?}", err);
        err.into()
    })
}


#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DiscoveryError;


/// Discovered service
pub struct Service {
    pub uuid: Uuid,
    pub start_handle: u16,
    pub end_handle: u16,
}

// Had trouble implementing TryFrom (compiler and LSP whining about
// not having Error defined *on the struct* rather than on the trait
// impl, but whatever) so going manual for now:
impl Service {
    fn new(v: &raw::ble_gattc_service_t) -> Result<Self, DiscoveryError> {
        let Some(uuid) = Uuid::from_raw(v.uuid) else {
            return Err(DiscoveryError);
        };

        Ok(Self {
            uuid,
            start_handle: v.handle_range.start_handle,
            end_handle: v.handle_range.end_handle,
        })
    }
}

// impl TryFrom<&raw::ble_gattc_service_t> for Service {
//     type Error = DiscoveryError;
    
//     fn try_from(v: &raw::ble_gattc_service_t) -> Result<Self, Self::Error> {
//         let Some(uuid) = Uuid::from_raw(v.uuid) else {
//             return Err(Self::Error);
//         };

//         Ok(Self {
//             uuid,
//             start_handle: v.handle_range.start_handle,
//             end_handle: v.handle_range.end_handle,
//         })
//     }
// }


pub struct FlexCharacteristic {
    pub uuid: Uuid,
    pub handle: u16,
    // pub handle_cccd: Option<NonZeroU16>,
}

impl FlexCharacteristic {
    fn new(v: &raw::ble_gattc_char_t) -> Result<Self, DiscoveryError> {
        let Some(uuid) = Uuid::from_raw(v.uuid) else {
            return Err(DiscoveryError);
        };

        Ok(Self {
            uuid,
            handle: v.handle_value,
        })
    }
}
// impl TryFrom<&raw::ble_gattc_char_t> for FlexCharacteristic {
//     type Error = DiscoveryError;
    
//     fn try_from(v: &raw::ble_gattc_char_t) -> Result<Self, Self::Error> {
//         let Some(uuid) = Uuid::from_raw(v.uuid) else {
//             return Err(Self::Error);
//         };

//         Ok(Self {
//             uuid,
//             handle: v.handle_value,
//         })
//     }
// }
    
pub struct FlexDescriptor {
    pub uuid: Uuid,
    pub handle: u16,
}

impl FlexDescriptor {
    fn new(v: &raw::ble_gattc_desc_t) -> Result<Self, DiscoveryError> {
        let Some(uuid) = Uuid::from_raw(v.uuid) else {
            return Err(DiscoveryError);
        };

        Ok(Self {
            uuid,
            handle: v.handle,
        })
    }
}
// impl TryFrom<&raw::ble_gattc_desc_t> for FlexDescriptor {
//     type Error = DiscoveryError;
    
//     fn try_from(v: &raw::ble_gattc_desc_t) -> Result<Self, Self::Error> {
//         let uuid = Uuid::from_raw(v.uuid) else {
//             return Err(Self::Error);
//         };

//         Ok(Self {
//             uuid,
//             handle: v.handle,
//         })
//     }
// }
    
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FlexClientError {
    Disconnected,
    Timeout,
    Gatt(GattError),
    Raw(RawError),
}

impl From<DisconnectedError> for FlexClientError {
    fn from(_: DisconnectedError) -> Self {
        Self::Disconnected
    }
}

impl From<GattError> for FlexClientError {
    fn from(err: GattError) -> Self {
        Self::Gatt(err)
    }
}

impl From<RawError> for FlexClientError {
    fn from(err: RawError) -> Self {
        Self::Raw(err)
    }
}

/// Trait for implementing "flexible" GATT clients.
#[allow(unused_variables)]
pub trait FlexClient {
    fn on_exchange_mtu_response(&self, mtu: u16) {}

    fn on_service(&self, data: Service) {}
    fn on_characteristics(&self, data: impl Iterator<Item = FlexCharacteristic>) {}
    fn on_descriptors(&self, data: impl Iterator<Item = FlexDescriptor>) {}
    fn on_discovery_failed(&self) {}

    fn on_write_response(&self) {}
    fn on_write_cmd_response(&self, count: u8) {}
    fn on_read_response(&self, data: Result<&[u8], u16>) {}

    /// Handles notification and indication events from the GATT server.
    fn on_hvx(&self, type_: HvxType, handle: u16, data: &[u8]) {}
}


pub async fn run_flex<'a, C: FlexClient>(conn: &Connection, client: &C) -> FlexClientError {
    let handle = match conn.with_state(|state| state.check_connected()) {
        Ok(handle) => handle,
        Err(e) => return e.into()
    };

    // debug!("wait_many");
    portal(handle)
        .wait_many(|ble_evt| unsafe {
            let ble_evt = &*ble_evt;
            let evt_id = u32::from(ble_evt.header.evt_id);

            if evt_id == raw::BLE_GAP_EVTS_BLE_GAP_EVT_DISCONNECTED {
                warn!("disconnected");
                return Some(FlexClientError::Disconnected);
            }

            match check_status(ble_evt) {
                Ok(_e) => debug!("evt {}", evt_id),
                Err(err) => error!("err {:?}", err),
            }

            // If it's not a disconnect, we know it must be a GATTC event
            // because that's the only other way we get events (ble::on_evt).
            let gattc_evt = get_union_field(ble_evt, &ble_evt.evt.gattc_evt);
            // pub struct ble_gattc_evt_t {
            //     #[doc = "< Connection Handle on which event occurred."]
            //     pub conn_handle: u16,
            //     #[doc = "< GATT status code for the operation, see @ref BLE_GATT_STATUS_CODES."]
            //     pub gatt_status: u16,
            //     #[doc = "< In case of error: The handle causing the error. In all other cases @ref BLE_GATT_HANDLE_INVALID."]
            //     pub error_handle: u16,
            //     #[doc = "< Event Parameters. @note Only valid if @ref gatt_status == @ref BLE_GATT_STATUS_SUCCESS."]
            //     pub params: ble_gattc_evt_t__bindgen_ty_1,
            // 
            // Is there some reason we don't just use a closure over the conn
            // argument above?
            // let conn = unwrap!(Connection::from_handle(gattc_evt.conn_handle));

            let evt = match evt_id {
                raw::BLE_GATTC_EVTS_BLE_GATTC_EVT_HVX => {
                    // TODO: check gatt_status first, as params is invalid otherwise.
                    let params = get_union_field(ble_evt, &gattc_evt.params.hvx);
                    let data = get_flexarray(ble_evt, &params.data, params.len as usize);
                    trace!(
                        "GATT_HVX write handle={:?} type={:?} data={:?}",
                        params.handle,
                        params.type_,
                        data
                    );

                    match params.type_.try_into() {
                        Ok(type_) => client.on_hvx(type_, params.handle, data),
                        Err(_) => {
                            error!("gatt_client invalid hvx type: {}", params.type_);
                        }
                    }
                }

                // Response to MTU exchange request.
                raw::BLE_GATTC_EVTS_BLE_GATTC_EVT_EXCHANGE_MTU_RSP => {
                    let data = match check_status(ble_evt) {
                        Ok(_) => {
                            let params = get_union_field(ble_evt, &gattc_evt.params.exchange_mtu_rsp);
                            let mtu = params.server_rx_mtu;
                            debug!("att mtu exchange: got mtu {:?}", mtu);
                            conn.with_state(|state| state.att_mtu = mtu);
                            mtu
                        }
                        Err(e) => 0,
                    };

                    client.on_exchange_mtu_response(data);
                }

                // Response to primary service discovery
                raw::BLE_GATTC_EVTS_BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP => {
                    let res: Option<Service> = match check_status(ble_evt) {
                        Ok(_) => {
                            let params = get_union_field(ble_evt, &gattc_evt.params.prim_srvc_disc_rsp);
                            let v = get_flexarray(ble_evt, &params.services, params.count as usize);

                            match v.len() {
                                0 => None,
                                1 => Service::new(&v[0]).ok(),
                                _n => {
                                    warn!(
                                        "Found {:?} services with the same UUID, using the first one",
                                        params.count
                                    );
                                    Service::new(&v[0]).ok()
                                }
                            }
                        }

                        Err(_) => None
                    };

                    match res {
                        Some(data) => client.on_service(data.into()),
                        None => client.on_discovery_failed(),
                    }
                }

                // Response to characteristic discovery
                raw::BLE_GATTC_EVTS_BLE_GATTC_EVT_CHAR_DISC_RSP => {
                    match check_status(ble_evt) {
                        Ok(_) => {
                            let params = get_union_field(ble_evt, &gattc_evt.params.char_disc_rsp);
                            let v = get_flexarray(ble_evt, &params.chars, params.count as usize);
                            let mut idx = 0;
                            let iter = core::iter::from_fn(move || {
                                while idx < v.len() {
                                    let item = &v[idx];
                                    idx += 1;
                                    if let Ok(v) = FlexCharacteristic::new(item) {
                                        return Some(v);
                                    }
                                }
                                None
                            });

                            client.on_characteristics(iter)
                        }

                        Err(_) => client.on_discovery_failed()
                    }
                 }

                // Response to descriptor discovery
                raw::BLE_GATTC_EVTS_BLE_GATTC_EVT_DESC_DISC_RSP => {
                    match check_status(ble_evt) {
                        Ok(_) => {
                            let params = get_union_field(ble_evt, &gattc_evt.params.desc_disc_rsp);
                            let v = get_flexarray(ble_evt, &params.descs, params.count as usize);
                            let mut idx = 0;
                            let iter = core::iter::from_fn(move || {
                                while idx < v.len() {
                                    let item = &v[idx];
                                    idx += 1;
                                    if let Ok(v) = FlexDescriptor::new(item) {
                                        return Some(v);
                                    }
                                }
                                None
                            });
                            client.on_descriptors(iter)
                        }

                        Err(_) => client.on_discovery_failed()
                    }
                }

                // Response to read request
                raw::BLE_GATTC_EVTS_BLE_GATTC_EVT_READ_RSP => {
                    let data = match check_status(ble_evt) {
                        Ok(evt) => {
                            let params = get_union_field(ble_evt, &gattc_evt.params.read_rsp);
                            let v = get_flexarray(ble_evt, &params.data, params.len as usize);
                            Ok(v)
                        }
                        Err(e) => Err(0), // FIXME: need to extract from the event
                    };
                    // TODO: this is incomplete, as we should be handling things like
                    // invalid handle, invalid offset, and maybe others. Not sure
                    // where those show up.  Should we be turning this into
                    // a new type, or pass in a tuple, or what?
                    client.on_read_response(data)
                }

                raw::BLE_GATTC_EVTS_BLE_GATTC_EVT_WRITE_RSP => {
                    match check_status(ble_evt) {
                        Ok(_) => client.on_write_response(),
                        Err(e) => {}
                    }
                }

                // Response to write commmand. Tells you how many writes
                // have been cleared from the queue, which may be useful
                // in tracking whether there's space for more.
                raw::BLE_GATTC_EVTS_BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE => {
                    let count = 1; // TODO
                    client.on_write_cmd_response(count)
                }

                // _ => todo!()
                raw::BLE_GATTC_EVTS_BLE_GATTC_EVT_REL_DISC_RSP => {
                    todo!()
                }
                raw::BLE_GATTC_EVTS_BLE_GATTC_EVT_ATTR_INFO_DISC_RSP => {
                    todo!()
                }
                raw::BLE_GATTC_EVTS_BLE_GATTC_EVT_CHAR_VAL_BY_UUID_READ_RSP => {
                    todo!()
                }
                raw::BLE_GATTC_EVTS_BLE_GATTC_EVT_CHAR_VALS_READ_RSP => {
                    todo!()
                }

                // Timeout: a bit unclear what can cause this, but the docs
                // seem clear that once it occurs you have little choice but
                // to close the connection and start over.
                raw::BLE_GATTC_EVTS_BLE_GATTC_EVT_TIMEOUT => {
                    return Some(FlexClientError::Timeout);
                }

                // This should never be seen if all gattc events are covered
                // above, but since they're just integers we need this for
                // the rest of the range.
                _ => {}
            };

            None
        })
        .await
}
