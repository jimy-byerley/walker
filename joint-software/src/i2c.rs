use core::{
    marker::PhantomData,
    cell::RefCell,
    ops::DerefMut,
};
use embedded_hal_async::i2c::I2c;
use packbytes::{FromBytes, ToBytes, ByteArray};


/// struct used to store the address of data in i2c slave and the type this data to serialize/deserialize it
pub struct Register<T> {
    pub address: u8,
    ty: PhantomData<T>,
}
impl<T> Register<T> {
    pub const fn new(address:u8) -> Self {
        Self{address, ty: PhantomData}
    }
}

/// high level representation of i2c slave, with some caching to optimize commands sending
pub struct Slave<'b, B> {
    /// i2c driver for exchanging memory commands
    bus: &'b RefCell<B>,
    /// slave address
    slave: u8,
    /// pointer currently set in slave
    pointer: Option<u8>,
}
impl<'b, B: I2c> Slave<'b, B> {
    pub fn new(bus: &'b RefCell<B>, slave: u8) -> Self {
        Self{bus, slave, pointer:None}
    }

    /// slave address on i2c bus
    pub fn address(&self) -> u8 {self.slave}

    /// read given register's current value
    pub async fn read<T:FromBytes>(&mut self, register: Register<T>) -> Result<T, B::Error>
    {
        let mut dst = T::Bytes::zeroed();
        let mut bus = self.bus.borrow_mut();
        let bus = bus.deref_mut();
        if Some(register.address) == self.pointer {
            bus.read(self.slave, dst.as_mut()).await?;
        }
        else {
            self.pointer.replace(register.address);
            bus.write_read(self.slave,
                WriteBuffer{address: register.address, value: []}.as_slice(),
                dst.as_mut(),
            ).await?;
        }
        Ok(T::from_bytes(dst))
    }
    /// write given register
    pub async fn write<T:ToBytes>(&mut self, register: Register<T>, value: T) -> Result<(), B::Error>
    {
        let mut bus = self.bus.borrow_mut();
        let bus = bus.deref_mut();
        self.pointer.replace(register.address);
        bus.write(self.slave, WriteBuffer{address:register.address, value: value.to_be_bytes()}.as_slice()).await?;
        Ok(())
    }
}


#[allow(dead_code)] // those attributes are actually written and used but not namely
#[repr(packed)]
struct WriteBuffer<T: ByteArray> {
    address: u8,
    value: T,
}
impl<T: ByteArray> WriteBuffer<T> {
    pub fn as_slice(&self) -> &[u8] {
        unsafe { core::slice::from_raw_parts(
            core::mem::transmute::<& Self, *const u8>(self),
            core::mem::size_of::<Self>(),
            )}
    }
}


#[macro_export]
macro_rules! pack_bilge {
    ($t:ty) => {
    
        impl packbytes::ToBytes for $t {
            type Bytes = [u8; core::mem::size_of::<$t>()];
            
            fn to_le_bytes(self) -> Self::Bytes {
                self.value.value().to_le_bytes()
            }
            fn to_be_bytes(self) -> Self::Bytes {
                self.value.value().to_be_bytes()
            }
        }
        impl packbytes::FromBytes for $t {
            type Bytes = [u8; core::mem::size_of::<$t>()];
            
            fn from_le_bytes(bytes: Self::Bytes) -> Self {
                <$t>::from(<$t as bilge::Bitsized>::ArbitraryInt::from_be_bytes(bytes))
            }
            fn from_be_bytes(bytes: Self::Bytes) -> Self {
                <$t>::from(<$t as bilge::Bitsized>::ArbitraryInt::from_be_bytes(bytes))
            }
        }
    };
}
