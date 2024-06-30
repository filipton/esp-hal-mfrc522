// FROM: https://github.com/OSSLibraries/Arduino_MFRC522v2/blob/master/src/MFRC522Constants.h

pub struct PCDRegister;
pub struct PCDCommand;

#[allow(dead_code, non_upper_case_globals)]
impl PCDRegister {
    // Page 0: Command and status
    //              0x00                    // reserved for future use
    pub const CommandReg: u8 = 0x01; // starts and stops command execution
    pub const ComIEnReg: u8 = 0x02; // enable and disable interrupt request control bits
    pub const DivIEnReg: u8 = 0x03; // enable and disable interrupt request control bits
    pub const ComIrqReg: u8 = 0x04; // interrupt request bits
    pub const DivIrqReg: u8 = 0x05; // interrupt request bits
    pub const ErrorReg: u8 = 0x06; // error bits showing the error status of the last command executed
    pub const Status1Reg: u8 = 0x07; // communication status bits
    pub const Status2Reg: u8 = 0x08; // receiver and transmitter status bits
    pub const FIFODataReg: u8 = 0x09; // input and output of 64 byte FIFO buffer
    pub const FIFOLevelReg: u8 = 0x0A; // number of bytes stored in the FIFO buffer
    pub const WaterLevelReg: u8 = 0x0B; // level for FIFO underflow and overflow warning
    pub const ControlReg: u8 = 0x0C; // miscellaneous control registers
    pub const BitFramingReg: u8 = 0x0D; // adjustments for bit-oriented frames
    pub const CollReg: u8 = 0x0E; // bit position of the first bit-collision detected on the RF interface
                                  //              0x0F     // reserved for future use

    // Page 1: Command
    //               0x10     // reserved for future use
    pub const ModeReg: u8 = 0x11; // defines general modes for transmitting and receiving
    pub const TxModeReg: u8 = 0x12; // defines transmission data rate and framing
    pub const RxModeReg: u8 = 0x13; // defines reception data rate and framing
    pub const TxControlReg: u8 = 0x14; // controls the logical behavior of the antenna driver pins TX1 and TX2
    pub const TxASKReg: u8 = 0x15; // controls the setting of the transmission modulation
    pub const TxSelReg: u8 = 0x16; // selects the internal sources for the antenna driver
    pub const RxSelReg: u8 = 0x17; // selects internal receiver settings
    pub const RxThresholdReg: u8 = 0x18; // selects thresholds for the bit decoder
    pub const DemodReg: u8 = 0x19; // defines demodulator settings
                                   //               0x1A     // reserved for future use
                                   //               0x1B     // reserved for future use
    pub const MfTxReg: u8 = 0x1C; // controls some MIFARE communication transmit parameters
    pub const MfRxReg: u8 = 0x1D; // controls some MIFARE communication receive parameters
                                  //               0x1E     // reserved for future use
    pub const SerialSpeedReg: u8 = 0x1F; // selects the speed of the serial UART interface

    // Page 2: Configuration
    //               0x20        // reserved for future use
    pub const CRCResultRegH: u8 = 0x21; // shows the MSB and LSB values of the CRC calculation
    pub const CRCResultRegL: u8 = 0x22;
    //               0x23        // reserved for future use
    pub const ModWidthReg: u8 = 0x24; // controls the ModWidth setting?
                                      //               0x25        // reserved for future use
    pub const RFCfgReg: u8 = 0x26; // configures the receiver gain
    pub const GsNReg: u8 = 0x27; // selects the conductance of the antenna driver pins TX1 and TX2 for modulation
    pub const CWGsPReg: u8 = 0x28; // defines the conductance of the p-driver output during periods of no modulation
    pub const ModGsPReg: u8 = 0x29; // defines the conductance of the p-driver output during periods of modulation
    pub const TModeReg: u8 = 0x2A; // defines settings for the internal timer
    pub const TPrescalerReg: u8 = 0x2B; // the lower 8 bits of the TPrescaler value. The 4 high bits are in TModeReg.
    pub const TReloadRegH: u8 = 0x2C; // defines the 16-bit timer reload value
    pub const TReloadRegL: u8 = 0x2D;
    pub const TCounterValueRegH: u8 = 0x2E; // shows the 16-bit timer value
    pub const TCounterValueRegL: u8 = 0x2F;

    // Page 3: Test Registers
    //               0x30      // reserved for future use
    pub const TestSel1Reg: u8 = 0x31; // general test signal configuration
    pub const TestSel2Reg: u8 = 0x32; // general test signal configuration
    pub const TestPinEnReg: u8 = 0x33; // enables pin output driver on pins D1 to D7
    pub const TestPinValueReg: u8 = 0x34; // defines the values for D1 to D7 when it is used as an I/O bus
    pub const TestBusReg: u8 = 0x35; // shows the status of the internal test bus
    pub const AutoTestReg: u8 = 0x36; // controls the digital self-test
    pub const VersionReg: u8 = 0x37; // shows the software version
    pub const AnalogTestReg: u8 = 0x38; // controls the pins AUX1 and AUX2
    pub const TestDAC1Reg: u8 = 0x39; // defines the test value for TestDAC1
    pub const TestDAC2Reg: u8 = 0x3A; // defines the test value for TestDAC2
    pub const TestADCReg: u8 = 0x3B; // shows the value of ADC I and Q channels
                                     //               0x3C      // reserved for production tests
                                     //               0x3D      // reserved for production tests
                                     //               0x3E      // reserved for production tests
                                     //               0x3F      // reserved for production tests
}

#[allow(dead_code, non_upper_case_globals)]
impl PCDCommand {
    pub const Idle: u8 = 0x00; // no action, cancels current command execution
    pub const Mem: u8 = 0x01; // stores 25 bytes into the internal buffer
    pub const GenerateRandomID: u8 = 0x02; // generates a 10-byte random ID number
    pub const CalcCRC: u8 = 0x03; // activates the CRC coprocessor or performs a self-test
    pub const Transmit: u8 = 0x04; // transmits data from the FIFO buffer
    pub const NoCmdChange: u8 = 0x07; // no command change, can be used to modify the CommandReg register bits without affecting the command, for example, the PowerDown bit
    pub const Receive: u8 = 0x08; // activates the receiver circuits
    pub const Transceive: u8 = 0x0C; // transmits data from FIFO buffer to antenna and automatically activates the receiver after transmission
    pub const MFAuthent: u8 = 0x0E; // performs the MIFARE standard authentication as a reader
    pub const SoftReset: u8 = 0x0F; // resets the MFRC522
}