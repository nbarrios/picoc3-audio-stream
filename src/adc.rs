use crate::bsp::hal::pac::{ADC, DMA, RESETS};
use picoc3_audio_stream::audio_packet::AudioPacket;
use rp2040_hal::dma::DREQ_ADC;

pub static mut ADC_BUFFER: [u16; 512] = [1; 512];
pub static mut AUDIO_PACKET_ONE: AudioPacket = AudioPacket {
    magic: *b"ADC",
    values: [0; 480],
};

pub struct DmaAdc {
    adc: ADC,
    dma: DMA,
}

impl DmaAdc {
    pub fn new(adc: ADC, dma: DMA, resets: &mut RESETS) -> Self {
        // Bring DMA Down
        resets.reset.modify(|_, w| w.dma().set_bit());
        // Bring DMA Up
        resets.reset.modify(|_, w| w.dma().clear_bit());
        // Wait for DMA up
        while resets.reset_done.read().dma().bit_is_clear() {}

        // Bring ADC Down
        resets.reset.modify(|_, w| w.adc().set_bit());
        // Bring ADC Up
        resets.reset.modify(|_, w| w.adc().clear_bit());
        // Wait for ADC up
        while resets.reset_done.read().adc().bit_is_clear() {}

        // Set DMA0 read addr to ADC FIFO addr
        dma.ch[0]
            .ch_read_addr
            .write(|w| unsafe { w.bits(adc.fifo.as_ptr() as u32) });

        let dest: *const [u16; 480] = unsafe { &AUDIO_PACKET_ONE.values };
        let dest_addr = dest as usize;

        // Set DMA0 write addr to ADC_BUFFER addr
        dma.ch[0]
            .ch_write_addr
            .write(|w| unsafe { w.bits(dest_addr as u32) });

        //DMA0 Config
        //Max Transfer Count
        dma.ch[0].ch_trans_count.write(|w| unsafe { w.bits(480) });

        dma.ch[0].ch_ctrl_trig.write(|w| unsafe {
            w.treq_sel()
                .bits(DREQ_ADC) // Pace using ADC DREQ
                .data_size()
                .bits(0x1) // 16-bit transfers
                .incr_write()
                .set_bit() // Increment write addr
                .ring_sel()
                .set_bit() // Wrap write addr
                .ring_size()
                .bits(10) // Wrap at 1024 bytes
                .en()
                .set_bit() //Enable DMA
        });

        //Enable DMA_IRQ_0 for channel 0
        dma.inte0.write(|w| unsafe { w.inte0().bits(1 << 0) });

        // Enable ADC
        adc.cs.write(|w| w.en().set_bit());

        while !adc.cs.read().ready().bit_is_set() {
            cortex_m::asm::nop();
        }

        // Sample at 48kS/s (48Mhz/1000)
        adc.div.modify(|_r, w| unsafe { w.int().bits(999) });

        adc.fcs
            .modify(|_r, w| unsafe { w.en().set_bit().thresh().bits(1).dreq_en().set_bit() });

        adc.cs
            .modify(|_r, w| unsafe { w.ainsel().bits(1).start_many().set_bit() });

        while !adc.cs.read().ready().bit_is_set() {
            cortex_m::asm::nop();
        }

        Self { adc, dma }
    }

    pub fn buffer_value(&self) -> [u16; 4] {
        let mut copy = [0u16; 4];
        copy.copy_from_slice(unsafe { &ADC_BUFFER[0..4] });

        copy
    }

    pub fn clear_interrupt(&self) {
        self.dma.ints0.write(|w| unsafe { w.ints0().bits(1 << 0) });
    }

    pub fn debug_latest_result(&self) -> u16 {
        let result: u16 = self.adc.result.read().result().bits();
        result
    }

    pub fn debug_print_fifo_control_status(&self) {
        let adc_fifo_control_status = self.adc.fcs.read().bits();
        defmt::info!(
            "ADC FIFO CS: EN {0=0..1} SHIFT {0=1..2} ERR {0=2..3} DREQ_EN {0=3..4} EMPTY {0=8..9} FULL {0=9..10} \
            UNDER {0=10..11} OVER {0=11..12} LEVEL {0=16..20} THRES {0=24..28}",
            adc_fifo_control_status
        );
    }

    pub fn debug_print_dma_control_status(&self) {
        let dma_control_status = self.dma.ch[0].ch_ctrl_trig.read().bits();
        defmt::info!(
            "DMA CS: EN {0=0..1} H_PRIO {0=1..2} DATA_SIZE {0=2..4} INCR_READ {0=4..5} INCR_WRITE {0=5..6} \
            RING_SIZE {0=6..10} RING_SEL {0=10..11} CHAIN_TO {0=11..15} TREQ_SEL {0=15..21} IRQ_QUIET {0=21..22} \
            BSWAP {0=22..23} SNIFF_EN {0=23..24} BUSY {0=24..25} WRITE_ERROR {0=29..30} READ_ERROR {0=30..31} \
            AHB_ERROR {0=31..32}",
            dma_control_status
        );
    }

    pub fn debug_dma_write_address(&self) -> u32 {
        self.dma.ch[0].ch_write_addr.read().bits()
    }

    pub fn debug_fifo_read(&self) -> u16 {
        self.adc.fifo.read().val().bits()
    }
}
