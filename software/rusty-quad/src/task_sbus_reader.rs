use defmt::*;
use embassy_rp::{uart::{self, Async}, peripherals::UART1};
use embassy_sync::{pubsub::Publisher, blocking_mutex::raw::CriticalSectionRawMutex};
use embassy_time::{Duration, Instant, with_timeout};
use sbus::SBusPacketParser;

use crate::sbus_cmd::{SbusCmd, self};

type CSMutex = CriticalSectionRawMutex;
type SbusDur = (Option<SbusCmd>, Duration);

#[embassy_executor::task]
pub async fn sbus_reader(
    mut rx: uart::UartRx<'static, UART1, Async>,
    ch_sbus_cmd: Publisher<'static, CSMutex, SbusDur, 1, 1, 1>,
    timeout: Duration,
) {
    let mut parser = SBusPacketParser::new();
    let mut prev_parse_time = Instant::now();

    // A buffer length of 1 may be slower, but perhaps also reduces
    // the risk of the buffer coming out of sync with the message..
    let mut read_buffer = [0; 1];

    info!("SBUS_PARSER : Entering main loop");
    loop {
        if Ok(Ok(())) == with_timeout(timeout, rx.read(&mut read_buffer)).await {
            parser.push_bytes(&read_buffer);
            if let Some(packet) = parser.try_parse() {
                let parse_time = Instant::now();
                let reformat = sbus_cmd::convert(&packet);
                ch_sbus_cmd
                    .publish_immediate((reformat, parse_time.duration_since(prev_parse_time)));
                prev_parse_time = parse_time;
            }
        } else {
            ch_sbus_cmd.publish_immediate((None, prev_parse_time.elapsed()));
        }
    }
}
