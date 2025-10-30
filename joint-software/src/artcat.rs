use core::marker::PhantomData;
use bilge::prelude::*;
use packbytes::{FromBytes, ToBytes, ByteArray};
use embedded_io_async::{Read, Write};
use crate::mutex::BusyMutex;
use crate::pack_bilge;


pub struct Register<T> {
    pub address: u16,
    ty: PhantomData<T>,
}
impl<T> Register<T> {
    pub const fn new(address:u16) -> Self {
        Self{address, ty: PhantomData}
    }
}

pub mod registers {
    use super::*;
    
    /// slave fixed address
    pub const address: Register<u16> = Register::new(0x0);
    /// first communication error raise by slave, write to to 0 to reset
    pub const error: Register<CommunicationError> = Register::new(0x2);
    /// protocol version
    pub const version: Register<u8> = Register::new(0x3);
//         /// error message, must be a UTF8 zero-terminated string
//         pub const message: Register<[u8; 32]> = Register::new(0x4);
    /// slave standard informations
    pub const device: Register::<Device> = Register::new(0x20);
    /// slave clock value when reading
    pub const clock: Register::<u64> = Register::new(0x100);
    /// mapping between registers and virtual memory
    pub const mapping: Register::<MappingTable> = Register::new(0x200);
    
    
    /// slave standard informations
    #[derive(Copy, Clone, FromBytes, ToBytes)]
    pub struct Device {
        /// model name, must be a UTF8 zero-terminated string
        model: [u8; 32],
        /// version of the slave's hardware, arbitrary format, must be a UTF8 zero-terminated string
        hardware_version: [u8; 32],
        /// version of the slave's software, arbitrary format, must be a UTF8 zero-terminated string
        software_version: [u8; 32],
    }
    #[derive(Copy, Clone, FromBytes, ToBytes)]
    pub struct MappingTable {
        size: u8,
        map: [Mapping; 128],
    }
    #[derive(Copy, Clone, FromBytes, ToBytes)]
    pub struct Mapping {
        mapped_start: u32,
        slave_start: u16,
        size: u16,
    }
    #[repr(u8)]
    #[derive(Copy, Clone, Default)]
    pub enum CommunicationError {
        #[default]
        None = 0,
        
        /// received command doesn't exist
        InvalidCommand = 1,
        /// requested read/write is not allowed for given register
        InvalidAccess = 2,
        /// requested register doesn't exist
        InvalidRegister = 3,
        /// register set in mapping doesn't exist
        InvalidMapping = 4,
    }
}


struct Slave<'d, B, const MEM: usize> {
    buffer: BusyMutex<[u8; MEM]>,
    control: BusyMutex<SlaveControl<B>>,
    receive: BusyMutex<[u8; MAX_COMMAND]>,
    send: BusyMutex<[u8; MAX_COMMAND]>,
}
struct SlaveControl<B> {
    bus: B,
    mapping: heapless::Vec<registers::Mapping, 128>,
    address: u16,
}
impl<'d, B: Read + Write, const MEM: usize> Slave<'d, B, MEM> {
    pub fn new(bus: B, device: registers::Device) -> Self {
        todo!()
    }
    pub async fn get<T: FromBytes>(&self, register: Register<T>) -> T {
        let mut dst = T::Bytes::zeroed();
        let src = self.buffer.lock().await;
        dst.as_mut().copy_from_slice(&src[usize::from(register.address) ..][.. T::Bytes::SIZE]);
        T::from_be_bytes(dst)
    }
    pub async fn set<T: ToBytes>(&self, register: Register<T>, value: T) {
        let src = value.to_be_bytes();
        let mut dst = self.buffer.lock().await;
        dst[usize::from(register.address) ..][.. T::Bytes::SIZE].copy_from_slice(src.as_ref());
    }
    pub async fn serve(&self) {
        let mut control = self.control.lock().await;
        let mut receive = self.receive.lock().await;
        let mut send = self.send.lock().await;
        
        let mut address;
        loop {
            // read header
            let size = control.bus.read(&mut receive[.. <Command as FromBytes>::Bytes::SIZE]).await;
            let header = Command::from_be_bytes(&receive);
            let local = SlaveRegister::from(header.address);
            
            // check command consistency
            if header.size > MAX_COMMAND {
                self.set(registers::error, registers::CommunicationError::InvalidAccess).await;
                continue;
            }
            if header.access.slave_fixed() && header.access.slave_topological() {
                self.set(registers::error, registers::CommunicationError::InvalidCommand).await;
                continue;
            }
            
            // logic for topologial addresses
            if header.access.slave_topological() {
                local.set_slave(local.slave().wrapping_sub(1));
                header.address = local.into();
            }
            // direct access to slave buffer
            if header.access.slave_fixed() && local.slave() == control.address
            || header.access.slave_topological() && local.slave() == 0 
            {
                // exchange requested chunk of data
                // mark the command executed
                header.executed += 1;
                control.bus.write(&header.to_be_bytes()).await;
                
                let size = control.bus.read(&mut receive[.. header.size]).await;
                assert_eq!(size, header.size);
                
                if header.access.read() {
                    send[.. header.size].copy_from_slice(
                        &self.buffer.lock().await
                        [usize::from(local.register()) ..][.. header.size]
                        );
                    control.bus.write(&send[.. header.size]).await;
                }
                else {
                    control.bus.write(&receive[.. header.size]).await;
                }
                if header.access.write() {
                    self.buffer.lock().await
                        [usize::from(local.register()) ..][.. header.size]
                        .copy_from_slice(&receive[.. header.size]);
                }
                
                // special actions for special registers
                let address = local.register();
                if address == registers::address.address {
                    control.address = self.get(registers::address).await;
                }
                else if address == registers::mapping.address {
                    let table = self.get(registers::mapping).await;
                    control.mapping.clear();
                    control.mapping.extend_from_slice(table.map[.. table.size]);
                    control.mapping.sort_by_key(|item| item.virtual_start);
                }
            }
            // access to bus virtual memory
            else if !header.access.slave_fixed() && !header.access.slave_topological() {
                // exchange data according to local mapping
                header.executed += 1;
                control.bus.write(&header.to_be_bytes()).await;
                
                todo!("iterate over mappings inside the requested area and exchange with registers");
            }
            // any other command
            else {
                // simply pass data
                control.bus.write(&header.to_be_bytes()).await;
                
                let size = control.bus.read(&mut receive[.. header.size]).await;
                assert_eq!(size, header.size);
                
                control.bus.write(receive[.. size]).await;
            }
        }
    }
}

pub const MAX_COMMAND: usize = 1024;

/// memory bus command header
#[derive(Copy, Clone, FromBytes, ToBytes, Debug)]
struct Command {
    /// type of memory access
    access: Access,
    /// counte the number of times this command has been executed by consecutive slaves
    executed: u8,
    /// address, its value depends on whether accessing a particular slave or the bus virtual memory
    address: u32,
    /// number of bytes to read/write, following this header
    size: u16,
}

/// type of memory access
#[bitsize(8)]
#[derive(Copy, Clone, FromBits, DebugBits, PartialEq, Default)]
struct Access {
    /// want to read memory
    read: bool,
    /// want to write memory, can be enabled along read
    write: bool,
    /** which memory to address
        - if False, the bus virtual memory is addressed, all slaves mixed, and a 32bit address is expected
        - if True, an individual slave's registers are addresses, the 32 bit addres concatenates 16bit address of slave and 16bit address of register in this slave
    */
    slave_fixed: bool,
    /// if set, the slave address is topological
    slave_topological: bool,
    _reserved: u5,
}
pack_bilge!(Access);

#[bitsize(32)]
#[derive(Copy, Clone, FromBits, DebugBits, PartialEq, Default)]
struct SlaveRegister {
    /// slave we are adressing the request to
    slave: u16,
    /// register we want to access
    register: u16,
}
pack_bilge!(SlaveRegister);
