use std::{
    marker::PhantomData,
    cell::RefCell,
    rc::Rc,
    ops::DerefMut,
};
use bilge::prelude::*;
use embedded_hal::i2c::I2c;


/// struct used to store the address of data in i2c slave and the type this data to serialize/deserialize it
pub struct Register<T:Bitsized> {
    pub address: u8,
    ty: PhantomData<T>,
}
impl<T:Bitsized> Register<T> {
    pub const fn new(address:u8) -> Self {
        Self{address, ty: PhantomData}
    }
}

/// high level representation of i2c slave, with some caching to optimize commands sending
pub struct Slave<B> {
    /// i2c driver for exchanging memory commands
    bus: Rc<RefCell<B>>,
    /// slave address
    slave: u8,
    /// pointer currently set in slave
    pointer: Option<u8>,
}
impl<B: I2c> Slave<B> {
    pub fn new(bus: Rc<RefCell<B>>, slave: u8) -> Self {
        Self{bus, slave, pointer:None}
    }

    /// slave address on i2c bus
    pub fn address(&self) -> u8 {self.slave}

    /// read given register's current value
    pub fn read<T>(&mut self, register: Register<T>) -> Result<T, B::Error>
    where
        T: Bitsized + From<T::ArbitraryInt> + Default,
        T::ArbitraryInt: From<T>
    {
        let mut dst = ReadBuffer{value: T::ArbitraryInt::from(T::default())};
        let mut bus = self.bus.borrow_mut();
        let bus = bus.deref_mut();
        if Some(register.address) == self.pointer {
            bus.read(self.slave, dst.as_slice())?;
        }
        else {
            self.pointer.replace(register.address);
            bus.write_read(self.slave,
                WriteBuffer{address: register.address, value: ()}.as_slice(),
                dst.as_slice(),
            )?;
        }
        Ok(T::from(dst.value))
    }
    /// write given register
    pub fn write<T>(&mut self, register: Register<T>, value: T) -> Result<(), B::Error>
    where T: Bitsized, T::ArbitraryInt: From<T>
    {
        let mut bus = self.bus.borrow_mut();
        let bus = bus.deref_mut();
        self.pointer.replace(register.address);
        bus.write(self.slave, WriteBuffer{address:register.address, value}.as_slice())?;
        Ok(())
    }
}


#[repr(packed)]
struct ReadBuffer<T: Sized> {
    value: T,
}
impl<T> ReadBuffer<T> {
    pub fn as_slice(&mut self) -> &mut [u8] {
        unsafe { core::slice::from_raw_parts_mut(
            core::mem::transmute::<&mut Self, *mut u8>(self),
            core::mem::size_of::<Self>(),
            ) }
    }
}

#[allow(dead_code)] // those attributes are actually written and used but not namely
#[repr(packed)]
struct WriteBuffer<T: Sized> {
    address: u8,
    value: T,
}
impl<T> WriteBuffer<T> {
    pub fn as_slice(&self) -> &[u8] {
        unsafe { core::slice::from_raw_parts(
            core::mem::transmute::<& Self, *const u8>(self),
            core::mem::size_of::<Self>(),
            ) }
    }
}
