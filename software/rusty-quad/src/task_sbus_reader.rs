use defmt::*;
use embassy_rp::{uart::{self, Async}, peripherals::UART1};
use embassy_sync::{signal::Signal, blocking_mutex::raw::CriticalSectionRawMutex};
use embassy_time::{Duration, Instant, with_timeout};
use sbus::SBusPacketParser;

use crate::sbus_cmd::{self, TriSwitch, SbusCmd};
use crate::signals;

static TASK_ID : &str = "SBUS_PARSER";

#[embassy_executor::task]
pub async fn sbus_reader(
    mut in_uart_rx_sbus: uart::UartRx<'static, UART1, Async>,
    out_sbus_cmd: signals::SbusCmdPub,
    parse_timeout_limit: Duration,
) {
    let mut parser = SBusPacketParser::new();
    let mut prev_parse_time = Instant::now();

    // A buffer length of 1 may be slower, but perhaps also reduces
    // the risk of the buffer coming out of sync with the message..
    let mut read_buffer = [0; 1];

    info!("{} : Entering main loop",TASK_ID);
    loop {

        match with_timeout(parse_timeout_limit, in_uart_rx_sbus.read(&mut read_buffer)).await {
            Ok(Ok(())) => {
                parser.push_bytes(&read_buffer);
                if let Some(packet) = parser.try_parse() {
                    let parse_time = Instant::now();
                    match sbus_cmd::convert(&packet) {
                        Some(cmd) => out_sbus_cmd.publish_immediate(Ok(cmd)),
                        None => out_sbus_cmd.publish_immediate(Err(SbusError::SbusFailsafe)),
                    }
                    prev_parse_time = parse_time;
                } else if Instant::now().duration_since(prev_parse_time) > parse_timeout_limit {
                    out_sbus_cmd.publish_immediate(Err(SbusError::ParseTimeout))
                }
            }
            Ok(Err(_)) => out_sbus_cmd.publish_immediate(Err(SbusError::SerialRead)),
            Err(_) => out_sbus_cmd.publish_immediate(Err(SbusError::SerialTimeout))
        }
    }
}

#[embassy_executor::task]
pub async fn sbus_switch_event(
    mut in_ch_sbus_cmd: crate::signals::SbusCmdSub,
    out_sg_sw_e: &'static Signal<CriticalSectionRawMutex,TriSwitch>,
    out_sg_sw_b: &'static Signal<CriticalSectionRawMutex,TriSwitch>,
    out_sg_sw_c: &'static Signal<CriticalSectionRawMutex,TriSwitch>,
    out_sg_sw_f: &'static Signal<CriticalSectionRawMutex,TriSwitch>,
) {

    let mut prev_sbus_msg = SbusCmd::default();

    info!("{} : Entering main loop",TASK_ID);
    loop {

        let sbus_result = in_ch_sbus_cmd.next_message_pure().await;

        if let Ok(sbus_msg) = sbus_result {
            if sbus_msg.sw_e != prev_sbus_msg.sw_e {
                out_sg_sw_e.signal(sbus_msg.sw_e);
            }

            if sbus_msg.sw_b != prev_sbus_msg.sw_b {
                out_sg_sw_b.signal(sbus_msg.sw_b);
            }

            if sbus_msg.sw_c != prev_sbus_msg.sw_c {
                out_sg_sw_c.signal(sbus_msg.sw_c);
            }

            if sbus_msg.sw_f != prev_sbus_msg.sw_f {
                out_sg_sw_f.signal(sbus_msg.sw_f);
            }

            prev_sbus_msg = sbus_msg;
        }
    }
}

#[derive(Clone,Format)]
pub enum SbusError {
    ParseTimeout,
    SerialRead,
    SerialTimeout,
    SbusFailsafe
}

