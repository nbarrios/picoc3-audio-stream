use minicbor::decode::{ArrayIterWithCtx, Decoder, Error as DecoderError};
use minicbor::encode::{Encoder, Error, Write};
use minicbor::{CborLen, Decode, Encode};

#[derive(Encode, Decode, CborLen)]
pub struct AudioPacket {
    #[n(0)]
    pub magic: [u8; 3],
    #[cbor(n(1), with = "crate::audio_packet")]
    pub values: [u16; 480],
}

fn encode<Ctx, W: Write>(
    v: &[u16; 480],
    e: &mut Encoder<W>,
    ctx: &mut Ctx,
) -> Result<(), Error<W::Error>> {
    e.array(480)?;
    for x in v {
        x.encode(e, ctx)?
    }
    Ok(())
}

fn decode<'b, Ctx>(d: &mut Decoder<'b>, ctx: &mut Ctx) -> Result<[u16; 480], DecoderError> {
    let p = d.position();
    let iter: ArrayIterWithCtx<Ctx, u16> = d.array_iter_with(ctx)?;
    let mut a: [u16; 480] = [0; 480];
    let mut i = 0;
    for x in iter {
        if i >= a.len() {
            let msg = concat!("array has more than ", 480, " elements");
            return Err(DecoderError::message(msg).at(p));
        }
        a[i] = x?;
        i += 1;
    }
    if i < a.len() {
        let msg = concat!("array has less than ", 480, " elements");
        return Err(DecoderError::message(msg).at(p));
    }
    Ok(a)
}

fn cbor_len<Ctx>(val: &[u16; 480], ctx: &mut Ctx) -> usize {
    let n = val.len();
    n.cbor_len(ctx) + val.iter().map(|x| x.cbor_len(ctx)).sum::<usize>()
}
