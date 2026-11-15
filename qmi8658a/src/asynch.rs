//! Async driver on `embedded-hal-async`.
//!
//! Mirrors the blocking [`Qmi8658a`](crate::Qmi8658a) on [`AsyncInterface`](crate::AsyncInterface):
//! both are generated from the same source.

crate::driver::define_driver!(
    crate::AsyncInterface,
    embedded_hal_async::delay::DelayNs,
    [async],
    [.await]
);
