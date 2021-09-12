use socketcan::{CANSocket, CANFrame, CANFilter};
use clap::{Arg, App};
use std::{
    fs::File,
    path::Path,
    io::{Read},
    time::Duration,
    thread
};
use crc::{Crc, Algorithm, CRC_32_AUTOSAR};
use vhrd_module_nvconfig::{NVConfig, NV_CONFIG_START_ADDR, SIZE_OF_NVCONFIG, BoardConfig, CANBusMode, CANBusSpeed, Version};
use crate::State::DataTransfer;

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
enum State{
    WaitingForCommand,
    StartOfTransfer,
    DataTransfer,
    EndOfTransfer
}

const RX_NV_CONFIG: u32 = 0x0000_0001;
const RX_NEW_FIRMWARE: u32 = 0x0000_0010;

const RX_WAITING_CMD: u32 = 0x0000_0100;

const BOOTLOADER_KEY: u32 = 0xDEADBEEF;


fn draw_percent(p: u8){
    print!("{}", (8u8 as char));
}


fn main(){
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
        /*.arg(Arg::new("base_can_id")
            .short("id".parse().unwrap())
            .long("can_id")
            .takes_value(true))*/
        .get_matches();

    println!("Can_flasher starts!");
    let mut state = State::WaitingForCommand;
    let mut prev_state = state;

    let mut NV_CONFIG: NVConfig = NVConfig{
        config_crc: 0,
        board_config: BoardConfig {
            hw_name: [1u8;32],
            hw_variant: [2u8;4],
            hw_version: Version {
                major: 3,
                minor: 4,
                patch: 5
            },
            bootloader_size: 10230,
            bootloader_crc: 0x00000000_90cbd840,
            bootloader_timeout_ms: 10000,
            fw_version: Version {
                major: 6,
                minor: 7,
                patch: 8
            },
            fw_variant: [1u8;4],
            fw_vcs_id: [2u8;8],
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


    let ptr = &NV_CONFIG as *const _;

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

    NV_CONFIG.board_config.fw_size = image_byte_array.len()  as u32;
    NV_CONFIG.board_config.fw_crc = Crc::<u32>::new(&CRC_32_AUTOSAR).checksum(image_byte_array.as_slice()) as u64;



    let mut nv_slice = unsafe{std::slice::from_raw_parts((ptr as *const u8), SIZE_OF_NVCONFIG)};
    let crc = unsafe{ Crc::<u32>::new(&CRC_32_AUTOSAR).checksum(&nv_slice[8..SIZE_OF_NVCONFIG]) as u64};
    for i in 0..10{
        println!("nv_sile[{}]: {}",i, nv_slice[i]);
    }
    println!("CRC 0x{:032x}", crc);
    NV_CONFIG.config_crc = crc;

    let can_iface = matches.value_of("can_bus_iface").unwrap();
    let cs = match CANSocket::open(can_iface){
        Err(why) => panic!("Couldn't open {}: {}", can_iface, why),
        Ok(cs) => {
            println!("Successfully opened {}", can_iface);
            cs
        },
    };

    nv_slice = unsafe{std::slice::from_raw_parts((ptr as *const u8), SIZE_OF_NVCONFIG)};

    cs.set_read_timeout(Duration::from_millis(100));
    let mut index = 0;





    let mut can_frames = nv_slice.chunks_exact(8);
    let mut remaining = can_frames.remainder();
    loop {
        match cs.read_frame() {
            Ok(f) => {
                match f.id() {
                    RX_WAITING_CMD =>{
                        let mut cmd_frame = CANFrame::new(RX_NV_CONFIG,
                                                          &[]
                                                          , false, false).unwrap();
                        cmd_frame.force_extended();
                        cs.write_frame(&cmd_frame).ok();
                        break;
                    }
                    RX_NV_CONFIG => {
                        state = match state {
                            State::WaitingForCommand => {
                                println!("State {:?}", state);
                                can_frames = nv_slice.chunks_exact(8);
                                remaining = can_frames.remainder();
                                prev_state = state;
                                State::StartOfTransfer
                            }
                            State::StartOfTransfer => {
                                println!("State {:?}", state);

                                let mut cmd_frame = CANFrame::new(RX_NV_CONFIG,
                                                                  &BOOTLOADER_KEY.to_be_bytes()
                                                                  , false, false).unwrap();
                                cmd_frame.force_extended();
                                cs.write_frame(&cmd_frame).ok();
                                prev_state = state;
                                State::DataTransfer
                            }
                            State::DataTransfer => {
                                println!("State {:?}", state);
                                if prev_state == State::StartOfTransfer || prev_state == DataTransfer {
                                    let state = match can_frames.next() {
                                        None => {
                                            let mut cmd_frame = CANFrame::new(RX_NV_CONFIG,
                                                                              remaining
                                                                              , false, false).unwrap();
                                            cmd_frame.force_extended();
                                            cs.write_frame(&cmd_frame).ok();
                                            State::EndOfTransfer
                                        }
                                        Some(frame) => {
                                            let mut cmd_frame = CANFrame::new(RX_NV_CONFIG,
                                                                              frame
                                                                              , false, false).unwrap();
                                            cmd_frame.force_extended();
                                            cs.write_frame(&cmd_frame).ok();
                                            index += 1;
                                            println!("Send: {}", index);
                                            prev_state = state;
                                            State::DataTransfer
                                        }
                                    };
                                    state
                                } else {
                                    prev_state = state;
                                    State::WaitingForCommand
                                }
                            }
                            State::EndOfTransfer => {
                                println!("State {:?}", state);
                                let mut cmd_frame = CANFrame::new(RX_NV_CONFIG,
                                                                  &[]
                                                                  , false, false).unwrap();
                                cmd_frame.force_extended();
                                cs.write_frame(&cmd_frame).ok();
                                prev_state = state;
                                break;
                                State::WaitingForCommand
                            }
                        }
                    }
                    RX_NEW_FIRMWARE => {
                        state = match state {
                            State::WaitingForCommand => {
                                println!("State {:?}", state);
                                can_frames = image_byte_array.as_slice().chunks_exact(8);
                                remaining = can_frames.remainder();
                                prev_state = state;
                                State::StartOfTransfer
                            }
                            State::StartOfTransfer => {
                                println!("State {:?}", state);

                                let mut cmd_frame = CANFrame::new(RX_NEW_FIRMWARE,
                                                                  &BOOTLOADER_KEY.to_be_bytes()
                                                                  , false, false).unwrap();
                                cmd_frame.force_extended();
                                cs.write_frame(&cmd_frame).ok();
                                prev_state = state;
                                State::DataTransfer
                            }
                            State::DataTransfer => {
                                println!("State {:?}", state);
                                if prev_state == State::StartOfTransfer || prev_state == DataTransfer {
                                    let state = match can_frames.next() {
                                        None => {
                                            let mut cmd_frame = CANFrame::new(RX_NEW_FIRMWARE,
                                                                              remaining
                                                                              , false, false).unwrap();
                                            cmd_frame.force_extended();
                                            cs.write_frame(&cmd_frame).ok();
                                            State::EndOfTransfer
                                        }
                                        Some(frame) => {
                                            let mut cmd_frame = CANFrame::new(RX_NEW_FIRMWARE,
                                                                              frame
                                                                              , false, false).unwrap();
                                            cmd_frame.force_extended();
                                            cs.write_frame(&cmd_frame).ok();
                                            index += 1;
                                            println!("Send: {}", index);
                                            prev_state = state;
                                            State::DataTransfer
                                        }
                                    };
                                    state
                                } else {
                                    prev_state = state;
                                    State::WaitingForCommand
                                }
                            }
                            State::EndOfTransfer => {
                                println!("State {:?}", state);
                                let mut cmd_frame = CANFrame::new(RX_NEW_FIRMWARE,
                                                                  &[]
                                                                  , false, false).unwrap();
                                cmd_frame.force_extended();
                                cs.write_frame(&cmd_frame).ok();
                                prev_state = state;
                                break;
                                State::WaitingForCommand

                            }
                        }
                    }
                    _ => {}
                }
        }
            Err(_) => {}
        };
    }





/*
    let bin_path = Path::new(matches.value_of("file.bin").unwrap());
    let can_iface = matches.value_of("can_bus_iface").unwrap();
    // let can_id_base = matches.value_of("base_can_id").unwrap().parse::<u32>().expect("can_id_base is wrong");
    let can_id_base = 0x1234_0000;

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

    //let mut crc = CRC::crc64();
    crc.digest(image_byte_array.as_slice());

    println!("Calculated crc64 : {}", crc);



    println!("Wainting cmd from bootloader...");
    cs.set_read_timeout(Duration::from_millis(10_000));
    cs.set_filter(&[CANFilter::new(can_id_base, 0x1FFF_FFFF).unwrap()]);
    match cs.read_frame() {
        Ok(f) => { println!("Get start command from bootloader!");}
        Err(e) => { panic!("No Start command from bootloader");}
    };


    println!("Send erase command!");
    let mut cmd_frame = CANFrame::new(can_id_base + 1u32,
                                      &[]
                                      , false, false).unwrap();
    cmd_frame.force_extended();
    cs.set_filter(&[CANFilter::new(can_id_base + 1u32 , 0x1FFF_FFFF).unwrap()]);
    match cs.write_frame(&cmd_frame){
        Err(why) => panic!("Can't send erase to 0x{:08x} in : {}",can_id_base + 1u32 ,why),
        Ok(_) => println!("Successfully send erase cmd to 0x{:08x}", can_id_base + 1u32),
    }
    match cs.read_frame() {
        Ok(f) => { println!("Receive Erase done!");}
        Err(e) => { panic!("Error receiving erase done");}
    };


    println!("Send CRC!");
    let mut cmd_frame = CANFrame::new(can_id_base + 0x02u32,
                                      crc.get_crc().to_le_bytes().as_ref(),
                                      false, false).unwrap();
    cmd_frame.force_extended();
    cs.set_filter(&[CANFilter::new(can_id_base + 0x02u32, 0x1FFF_FFFF).unwrap()]);
    match cs.write_frame(&cmd_frame){
        Err(why) => panic!("Can't send CRC to 0x{:08x} in : {}",can_id_base + 0x02u32 ,why),
        Ok(_) => println!("Successfully send CRC to 0x{:08x}", can_id_base + 0x02u32),
    }
    match cs.read_frame() {
        Ok(f) => {
            println!("Receive CRC done!");
            let mut new_crc = 0u64;
            for i in 0..f.data().len(){
                new_crc = (f.data()[i] as u64) << 8 * i | new_crc;
            }
            if crc.get_crc() == new_crc{
                println!("Received CRC Match!");
            }
            else{
                panic!("Received CRC Wrong!");
            }
        }
        Err(e) => { panic!("Error receiving CRC!");}
    };


    println!("Send img to bootloader: 0x{:08x}", can_id_base + 0x03u32);
    let one_sec = Duration::from_millis(150);
    let ind:usize = 0;

    let chunks = image_byte_array.chunks(8);
    let mut ind = 0;
    cs.set_filter(&[CANFilter::new(can_id_base + 0x03u32, 0x1FFF_FFFF).unwrap()]);
    for pack in chunks{
        let mut cmd_frame = CANFrame::new(can_id_base + 0x03u32,pack ,false, false).unwrap();
        cmd_frame.force_extended();
        match cs.write_frame(&cmd_frame){
            Err(why) => panic!("Can't send cmd to 0x{:08x} in : {}",can_id_base + 0x03u32, why),
            Ok(_) => {}//println!("Successfully send  prepare cmd to 0x1234_5678"),

        }
        match cs.read_frame() {
            Ok(f) => {
                //println!("Receive ACK");
            }
            Err(e) => { panic!("Error IMG ACK");}
        };
        ind+=1;
    }
    println!("Send chunks {}", ind);
    println!("Send boot command!");
    cs.set_filter(&[CANFilter::new(can_id_base + 0x04u32, 0x1FFF_FFFF).unwrap()]);
    let mut cmd_frame = CANFrame::new(can_id_base + 0x04u32,
                                      &[]
                                      , false, false).unwrap();
    cmd_frame.force_extended();

    match cs.write_frame(&cmd_frame){
        Err(why) => panic!("Can't send cmd to 0x{:08x} in : {}",can_id_base + 0x04u32, why),
        Ok(_) => println!("Successfully send  prepare cmd to 0x{:08x}", can_id_base + 0x04u32),
    }
    match cs.read_frame() {
        Ok(f) => {
            if f.data()[0] == 1 {
                println!("Preparing for boot: OK!");
            }
            if f.data()[0] == 0{
                panic!("Preparing for boot: ERR!");
            }
        }
        Err(e) => { panic!("Preparing for boot: ERR!");}
    };

    println!("Done!");

 */
}
