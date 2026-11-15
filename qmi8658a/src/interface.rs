//! Bus interfaces: I²C and SPI.

use qmi8658a_registers::{DEFAULT_DEVICE_ADDRESS, SPI_READ};

/// Register access over a bus.
///
/// Implemented by [`I2cInterface`] and [`SpiInterface`]. Transfers of more than one byte
/// address consecutive registers while `CTRL1.ADDR_AI` is set; the driver sets it in
/// `init`.
pub trait Interface {
    /// The bus error type.
    type Error;

    /// Reads consecutive registers, starting at `start`.
    fn read(&mut self, start: u8, buffer: &mut [u8]) -> Result<(), Self::Error>;

    /// Writes consecutive registers, starting at `start`.
    fn write(&mut self, start: u8, data: &[u8]) -> Result<(), Self::Error>;
}

/// Async register access over a bus.
///
/// Implemented by [`I2cInterface`] and [`SpiInterface`]. Transfers of more than one byte
/// address consecutive registers while `CTRL1.ADDR_AI` is set; the driver sets it in
/// `init`.
#[cfg(feature = "async")]
#[cfg_attr(docsrs, doc(cfg(feature = "async")))]
// The trait is meant for single-threaded executors; no `Send` bound on the futures.
#[allow(async_fn_in_trait)]
pub trait AsyncInterface {
    /// The bus error type.
    type Error;

    /// Reads consecutive registers, starting at `start`.
    async fn read(&mut self, start: u8, buffer: &mut [u8]) -> Result<(), Self::Error>;

    /// Writes consecutive registers, starting at `start`.
    async fn write(&mut self, start: u8, data: &[u8]) -> Result<(), Self::Error>;
}

/// The QMI8658A on an I²C bus (standard or fast mode, up to 400 kHz).
#[derive(Debug)]
pub struct I2cInterface<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C> I2cInterface<I2C> {
    /// Creates an interface for a device at [`DEFAULT_DEVICE_ADDRESS`].
    pub fn new(i2c: I2C) -> Self {
        Self::with_address(i2c, DEFAULT_DEVICE_ADDRESS)
    }

    /// Creates an interface for a device at the specified 7-bit address.
    pub fn with_address(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }

    /// Returns the I²C bus.
    pub fn release(self) -> I2C {
        self.i2c
    }
}

impl<I2C> Interface for I2cInterface<I2C>
where
    I2C: embedded_hal::i2c::I2c,
{
    type Error = I2C::Error;

    fn read(&mut self, start: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, &[start], buffer)
    }

    fn write(&mut self, start: u8, data: &[u8]) -> Result<(), Self::Error> {
        use embedded_hal::i2c::Operation;
        // Adjacent writes are sent as one transfer, without a repeated start.
        self.i2c.transaction(
            self.address,
            &mut [Operation::Write(&[start]), Operation::Write(data)],
        )
    }
}

#[cfg(feature = "async")]
impl<I2C> AsyncInterface for I2cInterface<I2C>
where
    I2C: embedded_hal_async::i2c::I2c,
{
    type Error = I2C::Error;

    async fn read(&mut self, start: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, &[start], buffer).await
    }

    async fn write(&mut self, start: u8, data: &[u8]) -> Result<(), Self::Error> {
        use embedded_hal_async::i2c::Operation;
        self.i2c
            .transaction(
                self.address,
                &mut [Operation::Write(&[start]), Operation::Write(data)],
            )
            .await
    }
}

/// The QMI8658A on an SPI bus (mode 0 or 3, up to 15 MHz).
///
/// The `CS` pin selects the interface: pulled low, the device talks SPI and ignores I²C.
/// The device starts in 4-wire mode. For 3-wire mode, set
/// [`Ctrl1::sim`](crate::registers::Ctrl1) and use a HAL that supports half-duplex transfers.
#[derive(Debug)]
pub struct SpiInterface<SPI> {
    spi: SPI,
}

impl<SPI> SpiInterface<SPI> {
    /// Creates an interface on the specified SPI device (bus plus chip select).
    pub fn new(spi: SPI) -> Self {
        Self { spi }
    }

    /// Returns the SPI device.
    pub fn release(self) -> SPI {
        self.spi
    }
}

impl<SPI> Interface for SpiInterface<SPI>
where
    SPI: embedded_hal::spi::SpiDevice,
{
    type Error = SPI::Error;

    fn read(&mut self, start: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        use embedded_hal::spi::Operation;
        self.spi.transaction(&mut [
            Operation::Write(&[start | SPI_READ]),
            Operation::Read(buffer),
        ])
    }

    fn write(&mut self, start: u8, data: &[u8]) -> Result<(), Self::Error> {
        use embedded_hal::spi::Operation;
        self.spi.transaction(&mut [
            Operation::Write(&[start & !SPI_READ]),
            Operation::Write(data),
        ])
    }
}

#[cfg(feature = "async")]
impl<SPI> AsyncInterface for SpiInterface<SPI>
where
    SPI: embedded_hal_async::spi::SpiDevice,
{
    type Error = SPI::Error;

    async fn read(&mut self, start: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        use embedded_hal_async::spi::Operation;
        self.spi
            .transaction(&mut [
                Operation::Write(&[start | SPI_READ]),
                Operation::Read(buffer),
            ])
            .await
    }

    async fn write(&mut self, start: u8, data: &[u8]) -> Result<(), Self::Error> {
        use embedded_hal_async::spi::Operation;
        self.spi
            .transaction(&mut [
                Operation::Write(&[start & !SPI_READ]),
                Operation::Write(data),
            ])
            .await
    }
}
