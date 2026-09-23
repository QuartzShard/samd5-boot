//! RS485 over a USB serial adapter: the transport that proves an update
//! works with no debugger in the loop, at the cost of assuming a
//! transceiver wired to PB02 / PB03 / PB00.

use std::io::{ErrorKind, Read, Write};
use std::time::Duration;

use anyhow::{Context, Result};
use serialport::{ClearBuffer, DataBits, FlowControl, Parity, SerialPort, StopBits};

use super::Transport;

pub const BAUD: u32 = 115_200;

pub struct Serial {
    port: Box<dyn SerialPort>,
    path: String,
    baud: u32,
}

impl Serial {
    pub fn open(path: &str, baud: u32) -> Result<Self> {
        let port = serialport::new(path, baud)
            .data_bits(DataBits::Eight)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .flow_control(FlowControl::None)
            // Poll granularity only; callers enforce their own deadlines.
            .timeout(Duration::from_millis(50))
            .open()
            .with_context(|| format!("opening {path}"))?;
        Ok(Self {
            port,
            path: path.to_string(),
            baud,
        })
    }
}

impl Transport for Serial {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize> {
        match self.port.read(buf) {
            Ok(n) => Ok(n),
            Err(e) if e.kind() == ErrorKind::TimedOut => Ok(0),
            Err(e) => Err(e).context("reading the serial port"),
        }
    }

    fn write(&mut self, bytes: &[u8]) -> Result<()> {
        self.port.write_all(bytes).context("writing the serial port")?;
        self.port.flush().context("flushing the serial port")
    }

    fn clear_input(&mut self) -> Result<()> {
        self.port
            .clear(ClearBuffer::Input)
            .context("clearing the serial input buffer")
    }

    fn describe(&self) -> String {
        format!("RS485 on {} @ {} 8N1", self.path, self.baud)
    }
}
