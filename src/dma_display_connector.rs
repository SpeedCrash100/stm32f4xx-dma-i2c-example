use core::{
    cell::RefCell,
    sync::atomic::{AtomicBool, Ordering},
};

use critical_section::Mutex;

use display_interface::{DataFormat, DisplayError, WriteOnlyDataCommand};
use hal::i2c::dma::I2CMasterWriteDMA;

const DISPLAY_BUFFER_SIZE: usize = 128 * 64 / 8 + 1; // Display 128x64 + 1 byte for DataByte
const COMMAND_BUFFER_SIZE: usize = 8;
const I2C_ADDRESS: u8 = 0x3C;

static COMMAND_SEND: AtomicBool = AtomicBool::new(false);
static DRAWING: AtomicBool = AtomicBool::new(false);

pub struct DMAI2cInterface<I2C: 'static> {
    display_buffer: [u8; DISPLAY_BUFFER_SIZE], // Display 128x64 + 1 byte for DataByte
    command_buffer: [u8; COMMAND_BUFFER_SIZE],

    i2c: &'static Mutex<RefCell<I2C>>,
}

impl<I2C: I2CMasterWriteDMA + 'static> DMAI2cInterface<I2C> {
    pub fn new(bus: &'static Mutex<RefCell<I2C>>) -> Self {
        Self {
            display_buffer: [0x40; DISPLAY_BUFFER_SIZE],
            command_buffer: [0x0; COMMAND_BUFFER_SIZE],
            i2c: bus,
        }
    }
}

impl<I2C: I2CMasterWriteDMA + 'static> WriteOnlyDataCommand for DMAI2cInterface<I2C> {
    fn send_commands(&mut self, cmd: DataFormat<'_>) -> Result<(), DisplayError> {
        while COMMAND_SEND.load(Ordering::SeqCst) {}

        match cmd {
            DataFormat::U8(slice) => {
                self.command_buffer[1..=slice.len()].copy_from_slice(&slice[0..slice.len()]);

                COMMAND_SEND.store(true, Ordering::SeqCst);
                let res = nb::block!(critical_section::with(|cs| {
                    let mut i2c = self.i2c.borrow(cs).borrow_mut();

                    unsafe {
                        i2c.write_dma(
                            I2C_ADDRESS,
                            &self.command_buffer[..=slice.len()],
                            Some(|_| COMMAND_SEND.store(false, Ordering::SeqCst)),
                        )
                    }
                }));

                if let Err(_) = res {
                    return Err(DisplayError::BusWriteError);
                }

                Ok(())
            }
            _ => Err(DisplayError::DataFormatNotImplemented),
        }
    }

    fn send_data(&mut self, buf: DataFormat<'_>) -> Result<(), DisplayError> {
        while DRAWING.load(Ordering::SeqCst) {}

        match buf {
            DataFormat::U8(slice) => {
                self.display_buffer[1..=slice.len()].copy_from_slice(&slice[0..slice.len()]);

                DRAWING.store(true, Ordering::SeqCst);
                let res = nb::block!(critical_section::with(|cs| {
                    let mut i2c = self.i2c.borrow(cs).borrow_mut();
                    unsafe {
                        i2c.write_dma(
                            I2C_ADDRESS,
                            &self.display_buffer[..=slice.len()],
                            Some(|_| DRAWING.store(false, Ordering::SeqCst)),
                        )
                    }
                }));

                if let Err(_) = res {
                    return Err(DisplayError::BusWriteError);
                }

                Ok(())
            }
            _ => Err(DisplayError::DataFormatNotImplemented),
        }
    }
}
