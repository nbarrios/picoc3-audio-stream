use minicbor::{CborLen, Decode, Encode};

#[derive(Encode, Decode, CborLen)]
pub struct AudioPacket {
    #[n(0)]
    pub magic: [u8; 3],
    #[n(1)]
    pub values: [u16; 4],
}
