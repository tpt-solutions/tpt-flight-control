//! seL4 capability / endpoint / protection-domain model.

use tpt_abstractions::{MemoryPool, PartitionChannel, Scheduler};

/// Errors surfaced by the seL4 backend.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Sel4Error {
    /// No capability (or insufficient rights) for the operation.
    NoCap,
    /// Endpoint buffer too small for the message.
    BufferTooSmall,
    /// Endpoint has no message pending.
    NoMessage,
    /// Protection domain id is unknown.
    UnknownDomain,
}

/// Capability rights mask (seL4-style Grant/Read/Write).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CapRights {
    read: bool,
    write: bool,
    grant: bool,
}

impl CapRights {
    /// No rights.
    pub const NONE: CapRights = CapRights {
        read: false,
        write: false,
        grant: false,
    };
    /// Read + write (typical endpoint send/recv).
    pub const RW: CapRights = CapRights {
        read: true,
        write: true,
        grant: false,
    };

    /// Construct rights from flags.
    pub const fn new(read: bool, write: bool, grant: bool) -> Self {
        Self { read, write, grant }
    }

    /// Whether `other` is a subset of `self`.
    pub fn covers(&self, other: &CapRights) -> bool {
        (!other.read || self.read) && (!other.write || self.write) && (!other.grant || self.grant)
    }
}

/// A seL4 endpoint providing synchronous IPC, modelled as a
/// [`PartitionChannel`]. A single pending message is held (synchronous
/// handshake semantics: a send blocks until a recv, modelled here by storing
/// the latest handed-over message).
#[derive(Debug, Clone)]
pub struct Endpoint {
    buf: [u8; 64],
    len: usize,
    has_message: bool,
}

impl Endpoint {
    /// Create an empty endpoint.
    pub const fn new() -> Self {
        Self {
            buf: [0u8; 64],
            len: 0,
            has_message: false,
        }
    }

    /// Construct a capability to this endpoint with the given rights.
    pub const fn cap(&self, rights: CapRights) -> EndpointCap {
        EndpointCap { rights }
    }
}

impl Default for Endpoint {
    fn default() -> Self {
        Self::new()
    }
}

/// A capability (rights wrapper) to an [`Endpoint`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EndpointCap {
    rights: CapRights,
}

impl EndpointCap {
    /// Rights carried by this capability.
    pub const fn rights(&self) -> CapRights {
        self.rights
    }
}

impl PartitionChannel for Endpoint {
    type Error = Sel4Error;

    fn write(&mut self, data: &[u8]) -> Result<(), Sel4Error> {
        if data.len() > self.buf.len() {
            return Err(Sel4Error::BufferTooSmall);
        }
        self.buf[..data.len()].copy_from_slice(data);
        self.len = data.len();
        self.has_message = true;
        Ok(())
    }

    fn read(&mut self, out: &mut [u8]) -> Result<usize, Sel4Error> {
        if !self.has_message {
            return Err(Sel4Error::NoMessage);
        }
        if out.len() < self.len {
            return Err(Sel4Error::BufferTooSmall);
        }
        out[..self.len].copy_from_slice(&self.buf[..self.len]);
        self.has_message = false;
        Ok(self.len)
    }

    fn fresh(&self) -> bool {
        self.has_message
    }
}

/// An isolated seL4 protection domain (address space).
#[derive(Debug, Clone, Copy)]
pub struct ProtectionDomain {
    id: u8,
    /// Bytes of memory this domain is authorised to use.
    mem_bytes: usize,
    used_bytes: usize,
}

impl ProtectionDomain {
    /// Create a domain with `mem_bytes` of authorised memory.
    pub const fn new(id: u8, mem_bytes: usize) -> Self {
        Self {
            id,
            mem_bytes,
            used_bytes: 0,
        }
    }

    /// Numeric domain id.
    pub const fn id(&self) -> u8 {
        self.id
    }
}

impl MemoryPool for ProtectionDomain {
    type Error = Sel4Error;

    fn capacity_bytes(&self) -> usize {
        self.mem_bytes
    }

    fn used_bytes(&self) -> usize {
        self.used_bytes
    }

    fn reset(&mut self) -> Result<(), Sel4Error> {
        self.used_bytes = 0;
        Ok(())
    }
}

impl ProtectionDomain {
    /// Internally account for an allocation (used by the domain's allocator).
    pub fn account_alloc(&mut self, bytes: usize) -> Result<(), Sel4Error> {
        if self.used_bytes + bytes > self.mem_bytes {
            return Err(Sel4Error::NoCap);
        }
        self.used_bytes += bytes;
        Ok(())
    }
}

/// A monotonic scheduler for the seL4 domain set.
#[derive(Debug, Clone, Copy)]
pub struct Sel4Scheduler {
    now_us: u64,
}

impl Sel4Scheduler {
    /// Create a scheduler at t = 0.
    pub const fn new() -> Self {
        Self { now_us: 0 }
    }

    /// Advance the clock.
    pub fn advance(&mut self, us: u64) {
        self.now_us += us;
    }
}

impl Default for Sel4Scheduler {
    fn default() -> Self {
        Self::new()
    }
}

impl Scheduler for Sel4Scheduler {
    type Error = Sel4Error;

    fn monotonic_micros(&self) -> Result<u64, Sel4Error> {
        Ok(self.now_us)
    }
}

/// The seL4 backend: a set of isolated protection domains plus the IPC
/// endpoints that connect them.
#[derive(Debug, Clone)]
pub struct Sel4Backend {
    domains: [ProtectionDomain; 3],
    endpoints: [Endpoint; 2],
    scheduler: Sel4Scheduler,
}

impl Sel4Backend {
    /// Build the default 3-domain topology (flight-core, fusion, comms) with
    /// two endpoints linking them.
    pub fn new() -> Self {
        Self {
            domains: [
                ProtectionDomain::new(0, 32 * 1024),
                ProtectionDomain::new(1, 32 * 1024),
                ProtectionDomain::new(2, 16 * 1024),
            ],
            endpoints: [Endpoint::new(), Endpoint::new()],
            scheduler: Sel4Scheduler::new(),
        }
    }

    /// A protection domain by index.
    pub fn domain(&self, idx: usize) -> &ProtectionDomain {
        &self.domains[idx]
    }

    /// Mutable protection domain by index.
    pub fn domain_mut(&mut self, idx: usize) -> &mut ProtectionDomain {
        &mut self.domains[idx]
    }

    /// An endpoint by index (owns the [`PartitionChannel`]).
    pub fn endpoint(&mut self, idx: usize) -> &mut Endpoint {
        &mut self.endpoints[idx]
    }

    /// The backend scheduler.
    pub const fn scheduler(&self) -> &Sel4Scheduler {
        &self.scheduler
    }

    /// Demo IPC: domain 0 sends `msg` to domain 1 over endpoint 0.
    pub fn ipc_send(&mut self, msg: &[u8]) -> Result<(), Sel4Error> {
        self.endpoints[0].write(msg)
    }

    /// Demo IPC: domain 1 receives over endpoint 0.
    pub fn ipc_recv(&mut self, out: &mut [u8]) -> Result<usize, Sel4Error> {
        self.endpoints[0].read(out)
    }
}

impl Default for Sel4Backend {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rights_subset_check() {
        assert!(CapRights::RW.covers(&CapRights::NONE));
        assert!(!CapRights::NONE.covers(&CapRights::RW));
        assert!(CapRights::new(true, true, true).covers(&CapRights::RW));
    }

    #[test]
    fn endpoint_ipc_handshake() {
        let mut ep = Endpoint::new();
        ep.write(b"hello seL4").unwrap();
        assert!(ep.fresh());
        let mut out = [0u8; 32];
        let n = ep.read(&mut out).unwrap();
        assert_eq!(&out[..n], b"hello seL4");
        assert!(!ep.fresh());
        assert_eq!(ep.read(&mut out), Err(Sel4Error::NoMessage));
    }

    #[test]
    fn memory_pool_bounds() {
        let mut dom = ProtectionDomain::new(0, 100);
        assert_eq!(dom.capacity_bytes(), 100);
        dom.account_alloc(60).unwrap();
        assert_eq!(dom.used_bytes(), 60);
        assert_eq!(dom.account_alloc(50), Err(Sel4Error::NoCap));
        dom.reset().unwrap();
        assert_eq!(dom.used_bytes(), 0);
    }

    #[test]
    fn backend_ipc_round_trip() {
        let mut be = Sel4Backend::new();
        be.ipc_send(b"nav update").unwrap();
        let mut out = [0u8; 64];
        let n = be.ipc_recv(&mut out).unwrap();
        assert_eq!(&out[..n], b"nav update");
    }

    #[test]
    fn backend_scheduler_monotonic() {
        let mut be = Sel4Backend::new();
        be.scheduler.advance(1234);
        assert_eq!(be.scheduler().monotonic_micros().unwrap(), 1234);
    }
}
