use std::{
    marker::PhantomData,
    cell::RefCell,
    rc::Rc,
    ops::DerefMut,
};
use embedded_hal::i2c::I2c;
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
    pub fn read<T:FromBytes>(&mut self, register: Register<T>) -> Result<T, B::Error>
    {
        let mut dst = T::Bytes::zeroed();
        let mut bus = self.bus.borrow_mut();
        let bus = bus.deref_mut();
        if Some(register.address) == self.pointer {
            bus.read(self.slave, dst.as_mut())?;
        }
        else {
            self.pointer.replace(register.address);
            bus.write_read(self.slave,
                WriteBuffer{address: register.address, value: []}.as_slice(),
                dst.as_mut(),
            )?;
        }
        Ok(T::from_bytes(dst))
    }
    /// write given register
    pub fn write<T:ToBytes>(&mut self, register: Register<T>, value: T) -> Result<(), B::Error>
    {
        let mut bus = self.bus.borrow_mut();
        let bus = bus.deref_mut();
        self.pointer.replace(register.address);
        bus.write(self.slave, WriteBuffer{address:register.address, value: value.to_be_bytes()}.as_slice())?;
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


// pub trait Pack<const N: usize> {
//     fn to_bytes(self) -> [u8; N];
//     fn from_bytes(bytes: [u8; N]) -> Self;
// }

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
    
    
//         impl packbytes::ToBytes for $t {
//             type Bytes = [u8; core::mem::size_of::<$t>()];
//             
//             #[allow(unnecessary_transmutes)]
//             fn to_le_bytes(self) -> Self::Bytes {
// //                 <$t as bilge::Bitsized>::ArbitraryInt::from(self).value().to_be_bytes()
//                 let _ = <$t as Bitsized>::MAX;
//                 let mut bytes = unsafe {core::mem::transmute::<Self, Self::Bytes>(self)};
//                 if cfg!(target_endian = "big") {
//                     const N: usize = core::mem::size_of::<$t>();
//                     for i in 0 .. N/2 {
//                         (bytes[i], bytes[N-i]) = (bytes[N-i], bytes[i]);
//                     }
//                 }
//                 bytes
//             }
//             #[allow(unnecessary_transmutes)]
//             fn to_be_bytes(self) -> Self::Bytes {
// //                 <$t as bilge::Bitsized>::ArbitraryInt::from(self).value().to_be_bytes()
//                 let _ = <$t as Bitsized>::MAX;
//                 let mut bytes = unsafe {core::mem::transmute::<Self, Self::Bytes>(self)};
//                 if cfg!(target_endian = "little") {
//                     const N: usize = core::mem::size_of::<$t>();
//                     for i in 0 .. N/2 {
//                         (bytes[i], bytes[N-i]) = (bytes[N-i], bytes[i]);
//                     }
//                 }
//                 bytes
//             }
//         }
//         impl packbytes::FromBytes for $t {
//             #[allow(unnecessary_transmutes)]
//             fn from_le_bytes(mut bytes: [u8; core::mem::size_of::<$t>()]) -> $t {
// //                 use bilge::Bitsized;
// //                 <$t>::from(<$t as bilge::Bitsized>::ArbitraryInt::new_unchecked(<$t as bilge::Bitsized>::ArbitraryInt::T::from_be_bytes(bytes)))
//                 if cfg!(target_endian = "little") {
//                     for i in 0 .. N/2 {
//                         (bytes[i], bytes[N-i]) = (bytes[N-i], bytes[i]);
//                     }
//                 }
//                 unsafe {core::mem::transmute::<[u8; N], $t>(bytes)}
//             }
//         }
    
//         impl crate::i2c::Pack<{ core::mem::size_of::<$t>() }> for $t {
//             #[allow(unnecessary_transmutes)]
//             fn to_bytes(self) -> [u8; core::mem::size_of::<$t>()] {
// //                 <$t as bilge::Bitsized>::ArbitraryInt::from(self).value().to_be_bytes()
//                 let _ = <$t as Bitsized>::MAX;
//                 const N: usize = core::mem::size_of::<$t>();
//                 let mut bytes = unsafe {core::mem::transmute::<Self, [u8; N]>(self)};
//                 if cfg!(target_endian = "little") {
//                     for i in 0 .. N/2 {
//                         (bytes[i], bytes[N-i]) = (bytes[N-i], bytes[i]);
//                     }
//                 }
//                 bytes
//             }
//             #[allow(unnecessary_transmutes)]
//             fn from_bytes(mut bytes: [u8; core::mem::size_of::<$t>()]) -> $t {
// //                 use bilge::Bitsized;
// //                 <$t>::from(<$t as bilge::Bitsized>::ArbitraryInt::new_unchecked(<$t as bilge::Bitsized>::ArbitraryInt::T::from_be_bytes(bytes)))
//                 const N: usize = core::mem::size_of::<$t>();
//                 if cfg!(target_endian = "little") {
//                     for i in 0 .. N/2 {
//                         (bytes[i], bytes[N-i]) = (bytes[N-i], bytes[i]);
//                     }
//                 }
//                 unsafe {core::mem::transmute::<[u8; N], $t>(bytes)}
//             }
//         }
    };
}

// pack_int!(u1);
// pack_int!(u2);
// pack_int!(u3);
// pack_int!(u4);
// pack_int!(u5);
// pack_int!(u6);
// pack_int!(u7);
// pack_int!(u8);
// pack_int!(u9);
// pack_int!(u10);
// pack_int!(u11);
// pack_int!(u12);
// pack_int!(u13);
// pack_int!(u14);
// pack_int!(u15);
// pack_int!(u16);
