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
            /// Write 1 to reset registers to default value
            REG_RST: bool = 7,
            /// 0: STAT/IB is configured as status indicator, 1: STAT/IB is configured as current indicator
            EN_STAT_IB: bool = 6,
            /// 0: PG/NTC is power good 1: PG/NTC is thermistor input
            EN_PG_NTC2: bool = 5,
            /// Lock charge settings, further writes can only reduce values
            LOCK_CHG: bool = 4,
            /// Enable the hold-off timer
            HOLDOFF_TMR: bool = 3,
            /// Set buck / boost frequency
            SW_FREQ: uint as enum SwFreq {
                Freq750kHz = 0b00,
                Freq1000kHz = 0b01,
                Freq1250kHz = 0b10,
                Freq1500kHz = 0b11,
            } = 1..3,
            /// VIN tracking enable (forces VIN_LIM ≥ VBATT + 165 mV)
            EN_VIN_TRK: bool = 0,
        },
        register Config1 {
            const ADDRESS = 1;
            const SIZE_BITS = 8;
            /// Force limit input current
            IIN_MODE: uint as enum IinMode {
                Auto = 0b000,
                Force100m = 0b001,
                Force500m = 0b010,
                Force900m = 0b011,
                Force1500m = 0b100,
                Force2000m = 0b101,
                Force3000m = 0b110,
                Other = catch_all,
            } = 5..8,
            /// input current limit, updated by input source detection, can be overwritten
            IIN_LIMIT: uint as enum InputCurrentLimit {
                I100mA    = 0b00000,
                I200mA  = 0b00001,
                I300mA  = 0b00010,
                I400mA  = 0b00011,
                I500mA  = 0b00100,
                I600mA  = 0b00101,
                I700mA  = 0b00110,
                I800mA  = 0b00111,
                I900mA  = 0b01000,
                I1000mA  = 0b01001,
                I1100mA  = 0b01010,
                I1200mA  = 0b01011,
                I1300mA  = 0b01100,
                I1400mA  = 0b01101,
                I1500mA  = 0b01110,
                I1600mA  = 0b01111,
                I1700mA  = 0b10000,
                I1800mA  = 0b10001,
                I1900mA  = 0b10010,
                I2000mA  = 0b10011,
                I2100mA  = 0b10100,
                I2200mA  = 0b10101,
                I2300mA  = 0b10110,
                I2400mA  = 0b10111,
                I2500mA  = 0b11000,
                I2600mA  = 0b11001,
                I2700mA  = 0b11010,
                I2800mA  = 0b11011,
                I2900mA  = 0b11100,
                I3000mA  = 0b11101,
                I3100mA  = 0b11110,
                I3200mA  = 0b11111,
            } = 0..5
        },
        register Config2 {
            const ADDRESS = 2;
            const SIZE_BITS = 8;
            /// pre charge to fast charge threshold
            VPRE: uint as enum PreChargeToFastChargeThreshold { 
                V2400mV = 0b00,
                V2600mV = 0b01,
                V2800mV = 0b10,
                V3000mV = 0b11,
            } =  6..8,
            /// Fast charge current
            ICC: uint = 0..6
        },
        register Config3 {
            const ADDRESS = 3;
            const SIZE_BITS = 8;
            /// Precharge current
            IPRE: uint = 4..8,
            /// Termination current
            ITERM: uint = 0..4
        },
        register Config4 {
            const ADDRESS = 4;
            const SIZE_BITS = 8;
            /// Recharge threshold:  0: 100mV, 1: 200mV
            VRECHG: bool = 7,
            /// Trickle charge current
            ITRICKLE: uint = 4..7,
            /// Input voltage limit threshold
            VIN_LIM: uint = 0..4
        },
        register Config5 {
            const ADDRESS = 5;
            const SIZE_BITS = 8;
            /// Timer to stop charging after charge termination
            TOPOFF_TM: uint as enum TopOffTimer {
                Disabled = 0b00,
                T15min = 0b01,
                T30min = 0b10,
                T45min = 0b11,
            } = 6..8,
            /// Battery regulation voltage (max battery voltage)
            VBATT: uint = 0..6
        },
        register Config6 {
            const ADDRESS = 6;
            const SIZE_BITS = 8;
            /// Input over voltage protection threshold
            VIN_OVP: uint as enum VinOvp {
                V6_3 = 0b00,
                V11 = 0b01,
                V14 = 0b10,
                Disabled = 0b11,
            } = 6..8,
            /// Minimum system voltage
            SYS_MIN: uint as enum SysMin {
                V2975mV = 0b000,
                V3150mV = 0b001,
                V3325mV = 0b010,
                V3500mV = 0b011,
                V3588mV = 0b100,
                V3675mV = 0b101,
                V3763mV = 0b110,
                Other = 0b111,
            } = 3..6,
            /// Thermal threshold for charge regulation and boost mode protection
            TREG: uint as enum ThermanRegulationThreshold {
                T60DegreesCelsius = 0b000,
                T70DegreesCelsius = 0b001,
                T80DegreesCelsius = 0b010,
                T90DegreesCelsius = 0b011,
                T100DegreesCelsius = 0b100,
                T110DegreesCelsius = 0b101,
                T120DegreesCelsius = 0b110,
                Other = 0b111,
            } = 0..3
        },
        register Config7 {
            const ADDRESS = 7;
            const SIZE_BITS = 8;
            /// 0: IB outputs when the switcher is on, 1: IB outputs all the time
            IB_EN: bool = 7,
            /// Reset the watchdog timer
            WATCHDOG_RST: bool = 6,
            /// Watchdog timer configuration
            WATCHDOG: uint as enum Watchdog {
                Disabled = 0b00,
                T40s = 0b01,
                T80s = 0b10,
                T160s = 0b11,
            } = 4..6,
            /// Enable termination
            EN_TERM: bool = 3,
            /// Enable 2x timer
            EN_TMR2X: bool = 2,
            CHG_TIMER: uint as enum ChargeSafetyTimer {
                Disabled = 0b00,
                T5hrs = 0b01,
                T10hrs = 0b10,
                T15hrs = 0b11,
            } = 0..2
        },
        register Config8 {
            const ADDRESS = 8;
            const SIZE_BITS = 8;
            /// Turn off BATTFET
            BATTFET_DIS: bool = 7,
            /// Turn off BATTFET after a 10s delay
            BATTFET_DLY: bool = 6,
            /// Enable BATTFET reset function
            BATTFET_RST_EN: bool = 5,
            OLIM: uint = 3..5,
            VBOOST: uint = 0..3
        },
        register Config9 {
            const ADDRESS = 9;
            const SIZE_BITS = 8;
            CC_CFG: uint as enum CcCfg {
                SinkOnly = 0b000,
                SourceOnly = 0b001,
                DRP = 0b010,
                DRPTrySNK = 0b011,
                DRPTrySRC = 0b100,
                Disabled = 0b101,
                Other = catch_all,
            } = 4..7,
            /// OTG is automatically controlled by CC detection
            AUTOOTG: bool = 3,
            /// Boost enabled
            EN_BOOST: bool = 2,
            /// Buck allowed
            EN_BUCK: bool = 1,
            /// Charging allowed
            EN_CHG: bool = 0,
        },
        register Config10 {
            const ADDRESS = 10;
            const SIZE_BITS = 8;
            ///  D+/D- detection automatically starts after VIN_GD = 1 and the hold-off timer ends
            AUTODPDM: bool = 5,
            /// Force D+/D- detection
            FORCEDPDM: bool = 4,
            RP_CFG: uint as enum RpCfg {
                Rp80uA = 0b00,
                Rp180uA = 0b01,
                Rp330uA = 0b10,
                Other = catch_all,
            } = 2..4,
            FORCE_CC: uint as enum ForceCC {
                AutoCC_CFG = 0b00,
                ForceRd = 0b01,
                ForceRp = 0b10,
                HiZ = 0b11,
            } = 0..2,
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
            /// 0: Only generate INT, 1: NTC1 is fully functional
            NTC1_ACTION: bool = 6,
            /// 0: Only generate INT, 1: NTC2 is fully functional
            NTC2_ACTION: bool = 5,
            /// Battery OVP is enabled
            BATT_OVP_EN: bool = 4,
            BATT_LOW: uint as enum BatteryLowThreshold {
                Threshold_3000mV = 0,
                Threshold_3100mv = 1,
                Threshold_3200mV = 2,
                Threshold_3300mV = 3,
            } = 2..4,
            /// 0: Only generate INT on Battery Low, 1: Turns off boost on Battery Low
            BOOST_STP_EN: bool = 1,
            /// 0: Boost OTP is ignored, 1: Boost OTP occurs at TREG
            BOOST_OTP_EN: bool = 0,
        },
        register Config13_NtcActions {
            const ADDRESS = 13;
            const SIZE_BITS = 8;
            WARM_ACT: uint as enum WarmAction {
                NoAction = 0b00,
                ReduceVBATT_REG = 0b01,
                ReduceIcc = 0b10,
                ReduceBoth = 0b11,
            } = 6..8,
            COOL_ACT: uint as enum CoolAction {
                NoAction = 0b00,
                ReduceVBATT_REG = 0b01,
                ReduceIcc = 0b10,
                ReduceBoth = 0b11,
            } = 4..6,
            JEITA_VSET: uint as enum JeitaVset {
                VBATT_REG_100mV = 0b00,
                VBATT_REG_150mV = 0b01,
                VBATT_REG_200mV = 0b10,
                VBATT_REG_250mV = 0b11,
            } = 2..4,
            JEITA_ISET: uint as enum JeitaISet {
                Percent50 = 0b00,
                Percent33 = 0b01,
                Percent20 = 0b10,
                Other = 0b11,
            }= 0..2,
        },
        register Config14_NtcTemperatureThreshold {
            const ADDRESS = 14;
            const SIZE_BITS = 8;
            VHOT: uint = 7..8,
            VWARM: uint = 4..6,
            VCOOL: uint = 2..4,
            VCOLD: uint = 0..2,
        },
        register Config15_InputImpedanceTest {
            const ADDRESS = 15;
            const SIZE_BITS = 8;
            /// Enables the input impedance test. Source current to the IN pin.
            VIN_SRC_EN: bool = 6,
            /// Configures the input impedance test current source.
            IVIN_SRC: uint as enum InputImpedanceTestCurrent {
                Current_5uA = 0,
                Current_10uA = 1,
                Current_20uA = 2,
                Current_40uA = 3,
                Current_80uA = 4,
                Current_160uA = 5,
                Current_320uA = 6,
                Current_640uA = 7,
                Current_1280uA = 8,
                Other = catch_all,
            }= 2..6,
            /// Configures the input impedance test comparator threshold.
            VIN_TEST: uint as enum InputImpedanceThreshold {
                Threshold_300mV = 0,
                Threshold_500mV = 1,
                Threshold_1000mV = 2,
                Threshold_1500mV = 3,
            } = 0..2,
        },
        register Config16_InterruptMask {
            const ADDRESS = 16;
            const SIZE_BITS = 8;
            /// Mask the THERM_STAT INT pulse
            MASK_THERM: bool = 5,
            /// Mask the VINDPM and IINDPM INT pulse
            MASK_DPM: bool = 4,
            /// Mask the TOPOFF timer INT pulse
            MASK_TOPOFF: bool = 3,
            /// Mask the CC_SNK and CC_SRC INT pulse
            MASK_CC_INT: bool = 2,
            /// Mask the BATT_LOW INT pulse
            MASK_BATT_LOW: bool = 1,
            /// Mask DEBUGACC and AUDIOACC INT pulse
            MASK_DEBUG_AUDIO: bool = 0,
        },
        register Status17 {
            type Access = RO;
            const ADDRESS = 17;
            const SIZE_BITS = 8;
            /// Returns the input source D+/D- detection result.
            DPDM_STAT: uint as enum DPDM_Detection_Result {
                NotStarted = 0,
                USB_SDP_500mA = 1,
                USB_DCP_2000mA = 2,
                USB_CDP_1500mA = 3,
                Divider_1000mA = 4,
                Divider_2100mA = 5,
                Divider_2400mA = 6,
                Divider_2000mA = 7,
                Unknown_500mA = 8,
                HV_2000mA = 9,
                Divider_3000mA = 14,
                Other = catch_all
            } = 4..8,
            VINDPM_STAT: bool = 1,
            IINDPM_STAT: bool = 0,
        },
        register Status18 {
            type Access = RO;
            const ADDRESS = 18;
            const SIZE_BITS = 8;
            /// When VVIN_UV < VIN < VVIN_OV in buck mode, this bit is set to 1 and the PG pin is driven low (after a 15ms debounce time).
            VIN_GD: bool = 6,
            /// Indicates whether input source type detection has finished. IIN_LIM[4:0] is updated.
            VIN_RDY: bool = 5,
            /// Legacy cable is detected (not valid in DRP mode)
            LEGACYCABLE: bool = 4,
            /// In thermal regulation
            THERM_STAT: bool = 3,
            /// 0: VBATT < VSYS_MIN, 1: VBATT > VSYS_MIN
            VSYS_STAT: bool = 2,
            /// The watchdog timer has expired
            WATCHDOG_FAULT: bool = 1,
            /// The 3/4 watchdog timer has expired
            WATCHDOG_BARK: bool = 0,
        },
        register Status19 {
            type Access = RO;
            const ADDRESS = 19;
            const SIZE_BITS = 8;
            CHG_STAT: uint as enum ChargingState {
                NotCharging = 0,
                TrickleCharge = 1,
                PreCharge = 2,
                FastCharge = 3,
                ConstantVoltageCharge = 4,
                ChargingDone = 5,
                Other = catch_all
            } = 5..8,
            BOOST_FAULT: uint as enum BoostFault {
                Normal = 0,
                /// An IN overload or short (latch-off) has occurred
                InOverload = 1,
                /// Boost over-voltage protection (OVP) (not latch) has occurred
                BoostOverVoltage = 2,
                /// Boost over-temperature protection (latch-off) has occurred
                BoostOverTemperature = 3,
                /// The boost has stopped due to BATT_LOW (latch-off)
                BattLow = 4,
                Other = catch_all,
            } = 2..5,
            CHG_FAULT: uint as enum ChargeFault {
                Normal = 0,
                InputOverVoltage = 1,
                ChargeTimerExpired = 2,
                BatteryOverVoltage = 3,
            } = 0..2,
        },
        register Status20 {
            type Access = RO;
            const ADDRESS = 20;
            const SIZE_BITS = 8;
            /// NTC is missing (VNTC > 95% of VVRNTC)
            NTC_MISSING: bool = 7,
            /// The battery is missing (2 terminations detected within 3 seconds)
            BATT_MISSING: bool = 6,
            NTC1_FAULT: uint as enum NTC1_FAULT {
                Normal = 0,
                Warm = 1,
                Cool = 2,
                Cold = 3,
                Hot = 4,
                Other = catch_all
            } = 3..6,
            NTC2_FAULT: uint as enum NTC2_FAULT {
                Normal = 0,
                Warm = 1,
                Cool = 2,
                Cold = 3,
                Hot = 4,
                Other = catch_all
            } = 0..3,
        },
        register Status21 {
            type Access = RO;
            const ADDRESS = 21;
            const SIZE_BITS = 8;
            CC1_SNK_STAT: uint as enum CC1_SNK_STAT{
                /// CC1 detects vRa
                vRa = 0,
                /// CC1 detects vRd-USB
                VRd_USB = 1,
                /// CC1 detects vRd-1.5
                vRd_1_5 = 2,
                /// CC1 detects vRd-3.0
                vRd_3_0 = 3,
            } = 6..8,
            CC2_SNK_STAT: uint as enum CC2_SNK_STAT{
                /// CC2 detects vRa
                vRa = 0,
                /// CC2 detects vRd-USB
                VRd_USB = 1,
                /// CC2 detects vRd-1.5
                vRd_1_5 = 2,
                /// CC2 detects vRd-3.0
                vRd_3_0 = 3,
            } = 4..6,
            CC1_SRC_STAT: uint as enum CC1_SRC_STAT{
                /// CC1 detects vOPEN
                vOPEN = 0,
                /// CC1 detects vRd
                VRd = 1,
                /// CC1 detects vRa
                vRa= 2,
                Other = catch_all
            } = 2..4,
            CC2_SRC_STAT: uint as enum CC2_SRC_STAT{
                /// CC2 detects vOPEN
                vOPEN = 0,
                /// CC2 detects vRd
                VRd = 1,
                /// CC2 detects vRa
                vRa= 2,
                Other = catch_all
            }  = 0..2,
        },
        register Status22 {
            type Access = RO;
            const ADDRESS = 22;
            const SIZE_BITS = 8;
            /// the top-off timer is counting
            TOPOFF_ACTIVE: bool = 6,
            /// The battery is discharging
            BFET_STAT: bool = 5,
            /// VBATT is below BATT_LOW[1:0]. The hysteresis = 200mV
            BATT_LOW_STAT: bool = 4,
            /// If the boost needs to be turned on/off by CC detection, this bit is set/reset with an INT pulse followed.
            OTG_NEED: bool = 3,
            /// VIN has reached the VIN_TEST threshold
            VIN_TEST_HIGH: bool = 2,
            /// Enters DebugAccessory.SNK state
            DEBUGACC: bool = 1,
            /// Enters AudioAccessory state
            AUDIOACC: bool = 0,
        }
    }
);

#[cfg(test)]
mod tests {}
