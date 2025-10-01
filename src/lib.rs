#![cfg_attr(not(test), no_std)]
//#![warn(missing_docs)]

const ADDRESS: u8 = 0x3F;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum Mp2722Error<E> {
    I2c(E),
}

pub struct Mp2722Interface<I2C: embedded_hal::i2c::I2c> {
    i2c: I2C,
}

impl<I2C: embedded_hal::i2c::I2c> Mp2722Interface<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Self { i2c }
    }

    pub fn free(self) -> I2C {
        self.i2c
    }
}

impl<I2C: embedded_hal::i2c::I2c> device_driver::RegisterInterface for Mp2722Interface<I2C> {
    type Error = Mp2722Error<I2C::Error>;
    type AddressType = u8;

    fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.i2c
            .write_read(ADDRESS, &[address], data)
            .map_err(Mp2722Error::I2c)
    }

    fn write_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        self.i2c
            .transaction(
                ADDRESS,
                &mut [
                    embedded_hal::i2c::Operation::Write(&[address]),
                    embedded_hal::i2c::Operation::Write(data),
                ],
            )
            .map_err(Mp2722Error::I2c)
    }
}

device_driver::create_device!(
    device_name: Mp2722Registers,
    dsl: {
        config {
            type DefaultByteOrder = LE;
            type RegisterAddressType = u8;
            type DefaultBitOrder = LSB0;
            type DefmtFeature = "defmt-03";
        }
        register Config0 {
            const ADDRESS = 0;
            const SIZE_BITS = 8;
            REG_RST: bool = 7,
            EN_STAT_IB: bool = 6,
            EN_PG_NTC2: bool = 5,
            LOCK_CHG: bool = 4,
            HOLDOFF_TMR: bool = 3,
            SW_FREQ: uint = 1..3,
            EN_VIN_TRK: bool = 0,
        },
        register Config1 {
            const ADDRESS = 1;
            const SIZE_BITS = 8;
            IIN_MODE: uint = 5..8,
            IIN_LIMIT: uint = 0..5
        },
        register Config2 {
            const ADDRESS = 2;
            const SIZE_BITS = 8;
            VPRE: uint = 6..8,
            ICC: uint = 0..6
        },
        register Config3 {
            const ADDRESS = 3;
            const SIZE_BITS = 8;
            IPRE: uint = 4..8,
            ITERM: uint = 0..4
        },
        register Config4 {
            const ADDRESS = 4;
            const SIZE_BITS = 8;
            VRECHG: bool = 7,
            ITRICKLE: uint = 4..7,
            VIN_LIM: uint = 0..4
        },
        register Config5 {
            const ADDRESS = 5;
            const SIZE_BITS = 8;
            TOPOFF_TM: uint = 6..8,
            VBATT: uint = 0..6
        },
        register Config6 {
            const ADDRESS = 6;
            const SIZE_BITS = 8;
            VIN_OVP: uint = 6..8,
            SYS_MIN: uint = 3..6,
            TREG: uint = 0..3
        },
        register Config7 {
            const ADDRESS = 7;
            const SIZE_BITS = 8;
            IB_EN: bool = 7,
            WATCHDOG_RST: bool = 6,
            WATCHDOG: uint = 4..6,
            EN_TERM: bool = 3,
            EN_TMR2X: bool = 2,
            CHG_TIMER: uint = 0..2
        },
        register Config8 {
            const ADDRESS = 8;
            const SIZE_BITS = 8;
            BATTFET_DIS: bool = 7,
            BATTFET_DLY: bool = 6,
            BATTFET_RST_EN: bool = 5,
            OLIM: uint = 3..5,
            VBOOST: uint = 0..3
        },
        register Config9 {
            const ADDRESS = 9;
            const SIZE_BITS = 8;
            CC_CFG: uint = 4..7,
            AUTOOTG: bool = 3,
            EN_BOOST: bool = 2,
            EN_BUCK: bool = 1,
            EN_CHG: bool = 0,
        },
        register Config10 {
            const ADDRESS = 10;
            const SIZE_BITS = 8;
            AUTODPDM: bool = 5,
            FORCEDPDM: bool = 4,
            RP_CFG: uint = 2..4,
            FORCE_CC: uint = 0..2,
        },
        register Config11 {
            const ADDRESS = 11;
            const SIZE_BITS = 8;
            HVEN: bool = 4,
            HVUP: bool = 3,
            HVDOWN: bool = 2,
            HVFREQ: uint = 0..2,
        },
        register Config12 {
            const ADDRESS = 12;
            const SIZE_BITS = 8;
            NTC1_ACTION: bool = 6,
            NTC2_ACTION: bool = 5,
            BATT_OVP_EN: bool = 4,
            BATT_LOW: uint = 2..4,
            BOOST_STP_EN: bool = 1,
            BOOST_OTP_EN: bool = 0,
        },
        register Config13 {
            const ADDRESS = 13;
            const SIZE_BITS = 8;
            WARM_ACT: uint = 7..8,
            COOL_ACT: uint = 4..6,
            JEITA_VSET: uint = 2..4,
            JEITA_ISET: uint = 0..2,
        },
        register Config14 {
            const ADDRESS = 14;
            const SIZE_BITS = 8;
            VHOT: uint = 7..8,
            VWARM: uint = 4..6,
            VCOOL: uint = 2..4,
            VCOLD: uint = 0..2,
        },
        register Config15 {
            const ADDRESS = 15;
            const SIZE_BITS = 8;
            VIN_SRC_EN: bool = 6,
            IVIN_SRC: uint = 2..6,
            VIN_TEST: uint = 0..1,
        },
        register Config16 {
            const ADDRESS = 16;
            const SIZE_BITS = 8;
            MASK_THERM: bool = 5,
            MASK_DPM: bool = 4,
            MASK_TOPOFF: bool = 3,
            MASK_CC_INT: bool = 2,
            MASK_BATT_LOW: bool = 1,
            MASK_DEBUG_AUDIO: bool = 0,
        },
        register Status17 {
            type Access = RO;
            const ADDRESS = 17;
            const SIZE_BITS = 8;
            DPDM_STAT: uint = 4..8,
            VINDPM_STAT: bool = 1,
            IINDPM_STAT: bool = 0,
        },
        register Status18 {
            type Access = RO;
            const ADDRESS = 18;
            const SIZE_BITS = 8;
            VIN_GD: bool = 6,
            VIN_RDY: bool = 5,
            LEGACYCABLE: bool = 4,
            THERM_STAT: bool = 3,
            VSYS_STAT: bool = 2,
            WATCHDOG_FAULT: bool = 1,
            WATCHDOG_BARK: bool = 0,
        },
        register Status19 {
            type Access = RO;
            const ADDRESS = 19;
            const SIZE_BITS = 8;
            CHG_STAT: uint = 5..8,
            BOOST_FAULT: uint = 2..5,
            CHG_FAULT: uint = 0..2,
        },
        register Status20 {
            type Access = RO;
            const ADDRESS = 20;
            const SIZE_BITS = 8;
            NTC_MISSING: bool = 7,
            BATT_MISSING: bool = 6,
            NTC1_FAULT: uint = 3..6,
            NTC2_FAULT: uint = 0..3,
        },
        register Status21 {
            type Access = RO;
            const ADDRESS = 21;
            const SIZE_BITS = 8;
            CC1_SNK_STAT: uint = 6..8,
            CC2_SNK_STAT: uint = 4..6,
            CC1_SRC_STAT: uint = 2..4,
            CC2_SRC_STAT: uint = 0..2,
        },
        register Status22 {
            type Access = RO;
            const ADDRESS = 22;
            const SIZE_BITS = 8;
            TOPOFF_ACTIVE: bool = 6,
            BFET_STAT: bool = 5,
            BATT_LOW_STAT: bool = 4,
            OTG_NEED: bool = 3,
            VIN_TEST_HIGH: bool = 2,
            DEBUGACC: bool = 1,
            AUDIOACC: bool = 0,
        }
    }
);

#[cfg(test)]
mod tests {}
