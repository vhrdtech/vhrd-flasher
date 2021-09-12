#![feature(const_option)]
#![feature(array_methods)]


use std::{
    rc::Rc,
    time::{self, Instant},
};

use socketcan::{CANFrame, CANSocket, CANFilter};

use clap::{Arg, App};
use uavcan_llr::types::{TransferId, CanId, NodeId, SubjectId, Priority, TransferKind, Service, ServiceId};
use uavcan_llr::tailbyte::{TailByte, TailByteIter, Kind};
use std::time::Duration;
use std::convert::TryFrom;
use std::sync::mpsc;
use vhrd_module_nvconfig::{NVConfig, BoardConfig, Version, CANBusMode, CANBusSpeed, SIZE_OF_NVCONFIG};
use std::borrow::{Borrow, BorrowMut};
use std::path::Path;
use std::fs::File;
use std::io::Read;
use crc::{Crc, Algorithm, CRC_32_AUTOSAR};
use std::slice::{ChunksExact, Iter};
use std::any::Any;

use indicatif::{ProgressBar, ProgressStyle};
use std::thread;
use std::cmp::min;


#[derive(Debug)]
enum DataTransferState{
    StartOfTransfer,
    DataTransfer,
    EndOfTransfer,
}
#[derive(Debug)]
enum UavCanMsgType{
    Single,
    Multi(DataTransferState),
    NoData
}
#[derive(Debug)]
enum RxEvent{
    Message(NodeId, SubjectId, UavCanMsgType),
    Response(NodeId, ServiceId, UavCanMsgType),
    Request(NodeId, ServiceId, UavCanMsgType)
}

const ERROR_MSG: SubjectId = SubjectId::new(0).unwrap();
const HARD_BIT_MSG: SubjectId = SubjectId::new(10).unwrap();
const REBOOT_MSG: SubjectId = SubjectId::new(2).unwrap();
const BOOT_MSG: SubjectId = SubjectId::new(1).unwrap();

const READ_SERVICE: ServiceId = ServiceId::new(0).unwrap();
const WRITE_CONFIG_SERVICE: ServiceId = ServiceId::new(1).unwrap();
const WRITE_FIRMWARE_SERVICE: ServiceId = ServiceId::new(2).unwrap();
const UNLOCK_BOOTLOADER: ServiceId = ServiceId::new(3).unwrap();

const REBOOT_DEVICE: ServiceId = ServiceId::new(4).unwrap();

const READ_CONFIG_CMD: u8 = 0x01;
const READ_FIRMWARE_CMD: u8 = 0x02;
const READ_BOOTLOADER_CMD: u8 = 0x03;


const FLASHER_NODE_ID: NodeId = NodeId::new(126).unwrap();

#[derive(Debug, Copy, Clone, PartialEq)]
enum FlasherState{
    Waiting,
    GotHardBit,
    RequestNvConfig,
    WaitingNvConfig,
    GotNvConfig,
    RequestBootloader,
    WaitingBootloader,
    GotNvBootloader,
    WriteNewNvConfig,
    SendingNewNvConfig,
    DoneSendingNewConfig,
    WriteNewFirmware,
    SendingNewFirmware,
    DoneSendingNewFirmware,
}

struct Flasher{
    firmware: Vec<u8>,
    nv_config: NVConfig,
    prev_state: FlasherState,
    state: FlasherState,
    rx_raw_vec: Vec<u8>,

    send_arr: Vec<CANFrame>,
    send_idx: u32,

    pb: ProgressBar,
    tx_bytes_len: u64
}

impl Flasher{
    pub fn new(new_firmware: &Vec<u8>, nv_config_raw: &NVConfig, node_id: &NodeId) -> Flasher{
        let mut nv_config = nv_config_raw.clone();
        nv_config.board_config.uavcan_node_id = node_id.inner();

        println!("NEW_uavcan_node_id: {}",nv_config.board_config.uavcan_node_id);

        let mut pb = ProgressBar::new(new_firmware.len() as u64);
        pb.set_style(ProgressStyle::default_bar()
            .template("{spinner:.green} [{elapsed_precise}] [{wide_bar:.cyan/blue}] {bytes}/{total_bytes} ({eta})")
            .with_key("eta", |state| format!("{:.1}s", state.eta().as_secs_f64()))
            .progress_chars("#>-"));

        Flasher{
            firmware: new_firmware.clone(),
            nv_config,
            prev_state: FlasherState::Waiting,
            state: FlasherState::WriteNewFirmware,//FlasherState::Waiting,
            rx_raw_vec: Vec::<u8>::new(),

            send_arr: Vec::<CANFrame>::new(),
            send_idx: 0,
            pb,
            tx_bytes_len: 0
        }
    }

    pub fn rx_parser(&mut self, event: RxEvent, data: &[u8]){
        match event{
            RxEvent::Message(n, sub, typ) => {
                match sub{
                    ERROR_MSG => {
                        println!("Error");
                    }
                    HARD_BIT_MSG => {
                        if self.state == FlasherState::Waiting && data.is_empty(){
                            println!("HardBit");
                            self.state = FlasherState::GotHardBit;
                        }
                    }
                    REBOOT_MSG => {
                        println!("RX_Reboot");
                    }
                    BOOT_MSG => {
                        println!("Boot");
                    }
                    _ => {
                        println!("Unexpected message");
                    }
                }
            }
            RxEvent::Response(n, ser, typ) => {
                match ser{
                    READ_SERVICE => {
                        match typ{
                            UavCanMsgType::Single => {

                            }
                            UavCanMsgType::Multi(d) => {
                                match d{
                                    DataTransferState::StartOfTransfer => {
                                        self.rx_raw_vec.extend_from_slice(data);
                                        if self.state == FlasherState::WaitingNvConfig {
                                            self.state = FlasherState::WaitingNvConfig;
                                        }
                                        if self.state == FlasherState::WaitingBootloader {
                                            self.state = FlasherState::WaitingBootloader;
                                        }
                                    }
                                    DataTransferState::DataTransfer => {
                                        self.rx_raw_vec.extend_from_slice(data);
                                        if self.state == FlasherState::WaitingNvConfig {
                                            self.state = FlasherState::WaitingNvConfig;
                                        }
                                        if self.state == FlasherState::WaitingBootloader {
                                            self.state = FlasherState::WaitingBootloader;
                                        }
                                    }
                                    DataTransferState::EndOfTransfer => {
                                        self.rx_raw_vec.extend_from_slice(data);
                                        println!("{:?}",self.rx_raw_vec.len());
                                        if self.state == FlasherState::WaitingNvConfig {
                                            self.state = FlasherState::GotNvConfig;
                                        }
                                        if self.state == FlasherState::WaitingBootloader {
                                            self.state = FlasherState::GotNvBootloader;
                                        }
                                    }
                                }
                            }
                            UavCanMsgType::NoData => {

                            }
                        }
                    }
                    WRITE_CONFIG_SERVICE => {
                        self.state = FlasherState::SendingNewNvConfig;
                    }
                    WRITE_FIRMWARE_SERVICE => {
                        self.state = FlasherState::SendingNewFirmware;
                    }
                    UNLOCK_BOOTLOADER => {}
                    _ => {
                        println!("Unexpected service");
                    }
                }
            }
            RxEvent::Request(_, _, _) => {}
        }

    }
    pub fn worker(&mut self, dst_node_id: &NodeId) -> Option<CANFrame>{
        //println!("{:?}", self.state);
        let res = match self.state{
            FlasherState::Waiting => { None }
            FlasherState::GotHardBit => {
               match self.prev_state {
                    FlasherState::Waiting => {
                        self.state = FlasherState::WaitingNvConfig;
                        let mut data = [0u8;2];
                        data[0] = READ_CONFIG_CMD;
                        data[1] = TailByte::new_single_frame(TransferId::new(0).unwrap()).as_byte();
                        Some(CANFrame::new(unsafe{CanId::new_service_kind(FLASHER_NODE_ID, *dst_node_id, READ_SERVICE, true, Priority::High).into()},
                                           data.borrow()
                                           , false, false).unwrap())
                    }
                    FlasherState::GotHardBit => {
                        None
                    }
                    _ => {None}
                }
            }
            FlasherState::GotNvConfig => {
                if self.rx_raw_vec.len() == SIZE_OF_NVCONFIG{
                    let rx_nv_cfg: NVConfig = unsafe { std::ptr::read(self.rx_raw_vec.as_slice().as_ptr() as *const _) };
                    self.rx_raw_vec = Vec::<u8>::new();
                    let rx_nv_ptr = &rx_nv_cfg as *const _;
                    let mut nv_slice = unsafe{std::slice::from_raw_parts((rx_nv_ptr as *const u8), SIZE_OF_NVCONFIG)};
                    let crc = Crc::<u32>::new(&CRC_32_AUTOSAR).checksum(&nv_slice[8..SIZE_OF_NVCONFIG]) as u64;
                    /*for i in 0..10{
                        println!("nv_sile[{}]: {:08x}",i, nv_slice[i]);
                    }*/
                    if rx_nv_cfg.config_crc != crc{
                        //println!("Wrong CRC!");
                        self.state = FlasherState::WaitingBootloader;
                        let mut data = [0u8;2];
                        data[0] = READ_BOOTLOADER_CMD;
                        data[1] = TailByte::new_single_frame(TransferId::new(0).unwrap()).as_byte();
                        return Some(CANFrame::new(unsafe{CanId::new_service_kind(FLASHER_NODE_ID, *dst_node_id, READ_SERVICE, true, Priority::High).into()},
                                                  data.borrow()
                                                  , false, false).unwrap());
                    }
                }
                self.state = FlasherState::WaitingBootloader;
                let mut data = [0u8;2];
                data[0] = READ_BOOTLOADER_CMD;
                return Some(CANFrame::new(unsafe{CanId::new_service_kind(FLASHER_NODE_ID, *dst_node_id, READ_SERVICE, true, Priority::High).into()},
                                          data.borrow()
                                          , false, false).unwrap());
                //None
            }
            FlasherState::GotNvBootloader => {
                let bootloder_crc = Crc::<u32>::new(&CRC_32_AUTOSAR).checksum(self.rx_raw_vec.as_slice()) as u64;
                println!("Bootloader crc = 0x{:08x}", bootloder_crc);
                println!("Bootloader len = {}", self.rx_raw_vec.len());
                self.nv_config.board_config.bootloader_size = self.rx_raw_vec.len() as u32;
                self.nv_config.board_config.bootloader_crc = bootloder_crc;
                self.state = FlasherState::WriteNewNvConfig;
                None
            }
            FlasherState::WriteNewNvConfig => {
                self.nv_config.board_config.fw_crc = Crc::<u32>::new(&CRC_32_AUTOSAR).checksum(self.firmware.as_slice()) as u64;
                self.nv_config.board_config.fw_size = self.firmware.len() as u32;
                let mut nv_cfg = self.nv_config;
                let ptr = &nv_cfg as *const _;
                let mut nv_slice = unsafe{std::slice::from_raw_parts((ptr as *const u8), SIZE_OF_NVCONFIG)};
                let crc = Crc::<u32>::new(&CRC_32_AUTOSAR).checksum(&nv_slice[8..(SIZE_OF_NVCONFIG)]) as u64;

                println!("NEW_uavcan_node_id: {}",self.nv_config.board_config.uavcan_node_id);
                println!("tx crc 0x{:08x}",crc);
                nv_cfg.config_crc = crc;
                /*for i in 0..nv_slice.len(){
                    println!("tx_nv_sile[{}]: 0x{:02x}",i, nv_slice[i]);
                }*/
                //nv_slice = unsafe{std::slice::from_raw_parts((ptr as *const u8), SIZE_OF_NVCONFIG)};
                let chunks = nv_slice.chunks_exact(7);
                let rm = chunks.remainder();
                let mut tail_b = TailByte::new_multi_frame(TransferId::new(0).unwrap(), chunks.len() + if rm.is_empty(){ 0usize } else { 1usize });
                let mut can_data = [0u8; 8];
                for msg in chunks.into_iter(){
                    can_data[0..7].copy_from_slice(&msg[0..7]);
                    can_data[7] = u8::try_from(tail_b.next().unwrap()).unwrap();
                    self.send_arr.push(CANFrame::new(
                        unsafe{CanId::new_service_kind(FLASHER_NODE_ID, *dst_node_id, WRITE_CONFIG_SERVICE, true, Priority::High).into()},
                        can_data.as_slice(),
                        false, false
                    ).unwrap());

                }
                if !rm.is_empty(){
                    can_data[0..rm.len()].copy_from_slice(&rm[0..rm.len()]);
                    can_data[rm.len()] = u8::try_from(tail_b.next().unwrap()).unwrap();
                    self.send_arr.push(CANFrame::new(
                        unsafe{CanId::new_service_kind(FLASHER_NODE_ID, *dst_node_id, WRITE_CONFIG_SERVICE, true, Priority::High).into()},
                        &can_data[0..rm.len() + 1],
                        false, false
                    ).unwrap());

                }
                self.state = FlasherState::SendingNewNvConfig;
                None
            }
            FlasherState::SendingNewNvConfig => {
                if self.send_arr.len() - 1 >= self.send_idx as usize {
                    self.state = FlasherState::SendingNewNvConfig;
                    let p_i = self.send_idx;
                    self.send_idx += 1;
                    return Some(self.send_arr[p_i as usize]);
                }
                self.state = FlasherState::DoneSendingNewConfig;
                None
            }
            FlasherState::DoneSendingNewConfig=>{
                println!("NV config has been sent");
                self.state = FlasherState::WriteNewFirmware;
                None
            }
            FlasherState::WriteNewFirmware=>{
                self.send_arr = Vec::<CANFrame>::new();
                self.send_idx = 0;
                let chunks = self.firmware.as_slice().chunks_exact(7);
                let rm = chunks.remainder();
                let mut tail_b = TailByte::new_multi_frame(TransferId::new(0).unwrap(), chunks.len() + if rm.is_empty(){ 0usize } else { 1usize });
                let mut can_data = [0u8; 8];
                for msg in chunks.into_iter(){
                    can_data[0..7].copy_from_slice(&msg[0..7]);
                    can_data[7] = u8::try_from(tail_b.next().unwrap()).unwrap();
                    self.send_arr.push(CANFrame::new(
                        unsafe{CanId::new_service_kind(FLASHER_NODE_ID, *dst_node_id, WRITE_FIRMWARE_SERVICE, true, Priority::High).into()},
                        can_data.as_slice(),
                        false, false
                    ).unwrap());

                }
                if !rm.is_empty(){
                    can_data[0..rm.len()].copy_from_slice(&rm[0..rm.len()]);
                    can_data[rm.len()] = u8::try_from(tail_b.next().unwrap()).unwrap();
                    self.send_arr.push(CANFrame::new(
                        unsafe{CanId::new_service_kind(FLASHER_NODE_ID, *dst_node_id, WRITE_FIRMWARE_SERVICE, true, Priority::High).into()},
                        &can_data[0..rm.len() + 1],
                        false, false
                    ).unwrap());

                }
                self.state = FlasherState::SendingNewFirmware;
                None
            }
            FlasherState::SendingNewFirmware => {
                if self.send_arr.len() - 1 >= self.send_idx as usize {
                    self.state = FlasherState::SendingNewFirmware;
                    let p_i = self.send_idx;
                    let frame: CANFrame = self.send_arr[p_i as usize];
                    self.send_idx += 1;
                    self.tx_bytes_len += frame.data().len() as u64 - 1u64;
                    self.pb.set_position(self.tx_bytes_len );
                    return Some(frame);
                }
                self.pb.finish();
                self.state = FlasherState::DoneSendingNewFirmware;
                None
            }
            FlasherState::DoneSendingNewFirmware=>{
                println!("All sent!");
                None
            }
            _ => {None}
        };
        self.prev_state = self.state;
        res
    }
}

fn main() {

    let matches = App::new("vhrd-flasher")
        .version("0.1.0")
        .author("Matvei Klimov <klimatt.gu@gmail.com>")
        .about("CAN-BUS flasher stm32 bootloader")
        .arg(Arg::new("file.bin")
            .short("f".parse().unwrap())
            .long("file")
            .takes_value(true))
        .arg(Arg::new("can_bus_iface")
            .short("i".parse().unwrap())
            .long("iface")
            .takes_value(true))
        .arg(Arg::new("Working_node_id")
            .short("w".parse().unwrap())
            .long("node_id")
            .takes_value(true))
        .arg(Arg::new("Setup_NEW_node_id")
            .short("n".parse().unwrap())
            .long("new_uavcan_node_id")
            .takes_value(true))
        .get_matches();

    let can_iface = matches.value_of("can_bus_iface").unwrap();
    let can_socket = CANSocket::open(can_iface).unwrap();

    let working_node_id = NodeId::new(matches.value_of("Working_node_id").unwrap().parse::<u8>().expect("Working_node_id is wrong")).unwrap();
    let new_node_id = NodeId::new(matches.value_of("Setup_NEW_node_id").unwrap().parse::<u8>().expect("Setup_NEW_node_id is wrong")).unwrap();
    can_socket.set_read_timeout(Duration::from_millis(5000));

    let bin_path = Path::new(matches.value_of("file.bin").unwrap());
    let mut file = match File::open(&bin_path) {
        Err(why) => panic!("Couldn't open {}: {}", bin_path.display(), why),
        Ok(file) => {
            println!("Successfully opened {}", bin_path.display());
            file
        },
    };

    let mut image_byte_array: Vec<u8> = Vec::new();
    match file.read_to_end(image_byte_array.as_mut()) {
        Err(why) => panic!("Couldn't read {}: {}", bin_path.display(), why),
        Ok(_) => println!("Successfully read {:}[bytes]", image_byte_array.len()),
    }

    let ptr = &NV_CONFIG as *const _;
    let mut nv_slice = unsafe{std::slice::from_raw_parts((ptr as *const u8), SIZE_OF_NVCONFIG)};
    let crc = Crc::<u32>::new(&CRC_32_AUTOSAR).checksum(&nv_slice[8..(SIZE_OF_NVCONFIG - 8)]) as u64;

    let crc = Crc::<u32>::new(&CRC_32_AUTOSAR).checksum(&nv_slice[8..SIZE_OF_NVCONFIG - 8]) as u64;


    let mut flasher = Flasher::new(&image_byte_array, &NV_CONFIG, &new_node_id);

    let rb_frame = CANFrame::new(
        unsafe{CanId::new_service_kind(FLASHER_NODE_ID, working_node_id, REBOOT_DEVICE, true, Priority::High).into()},
        &[],
        false, false
    ).unwrap();

    can_socket.write_frame(&rb_frame).ok();

    loop {
        if let Ok(frame) = can_socket.read_frame() {
            if let Ok(uavcan_id) = CanId::try_from(frame.id()) {
                if uavcan_id.source_node_id == working_node_id{
                    let (uavcan_msg_type, data) = match frame.data().last() {
                        None => { (UavCanMsgType::NoData, [].as_slice()) }
                        Some(last_byte) => {
                            let tail_byte = TailByte::from(*last_byte);
                            let msg_type = if tail_byte.kind == Kind::SingleFrame {
                                UavCanMsgType::Single
                            } else {
                                if tail_byte.kind == Kind::MultiFrame {
                                    UavCanMsgType::Multi(DataTransferState::StartOfTransfer)
                                } else if tail_byte.kind == Kind::EndT0 || tail_byte.kind == Kind::EndT1
                                {
                                    UavCanMsgType::Multi(DataTransferState::EndOfTransfer)
                                } else {
                                    UavCanMsgType::Multi(DataTransferState::DataTransfer)
                                }
                            };
                            (msg_type, &frame.data()[0..frame.data().len() - 1])
                        }
                    };
                    match uavcan_id.transfer_kind {
                        TransferKind::Message(m) => {
                            flasher.rx_parser(RxEvent::Message(uavcan_id.source_node_id, m.subject_id, uavcan_msg_type), data);
                        }
                        TransferKind::Service(s) => {
                            //println!("Dest node_id: {:?}", s.destination_node_id);
                            if !s.is_request && s.destination_node_id == FLASHER_NODE_ID {
                                flasher.rx_parser(RxEvent::Response(uavcan_id.source_node_id, s.service_id, uavcan_msg_type), data);
                            }
                        }
                    }
                    match flasher.worker(&working_node_id){
                        None => {}
                        Some(f) => { can_socket.write_frame(&f).ok();}
                    }
                }
            }
        }
    }

}

const NV_CONFIG: NVConfig = NVConfig{
config_crc: 0,
board_config: BoardConfig {
hw_name: [0u8;32],
hw_variant: [0u8;4],
hw_version: Version {
major: 0,
minor: 0,
patch: 0
},
bootloader_size: 0,
bootloader_crc: 0,
bootloader_timeout_ms: 5_000,
fw_version: Version {
major: 0,
minor: 0,
patch: 0
},
fw_variant: [0u8;4],
fw_vcs_id: [0u8;8],
fw_size: 0,
fw_crc: 0,
canbus_mode: CANBusMode::Unknown,
canbus_speed: CANBusSpeed::Unknown,
uavcan_node_id: 0,
reserved: [0;156]
},
firmware_specific: [0;256],
vhl_bytecode: [0;1528]
};
