// Configuration switches; remove/add a double-slash at line-start to enable/disable a feature; to save space disable e.g. CAT, DIAG, KEYER
#define DIAG             1   // Hardware diagnostics on startup (only disable when your rig is working)
#define KEYER            1   // CW keyer
#define CAT              1   // CAT-interface
#define F_XTAL    27000030   // 27MHz SI5351 crystal
//#define F_XTAL  25004000   // 25MHz SI5351 crystal  (enable for WB2CBA-uSDX, SI5351 break-out board or uSDXDuO)
//#define F_XTAL  25000000   // 25MHz SI5351 crystal  (enable for 25MHz TCXO)
//#define SWAP_ROTARY    1   // Swap rotary direction (enable for WB2CBA-uSDX)
//#define QCX            1   // Supports older (non-SDR) QCX HW modifications (QCX, QCX-SSB, QCX-DSP with I/Q alignment-feature)
//#define OLED_SSD1306   1   // OLED display (SSD1306 128x32 or 128x64), connect SDA (PD2), SCL (PD3)
//#define OLED_SH1106    1   // OLED display (SH1106 1.3" inch display), connect SDA (PD2), SCL (PD3), NOTE that this display is pretty slow
//#define LCD_I2C        1   // LCD with I2C (PCF8574 module          ), connect SDA (PD2), SCL (PD3), NOTE that this display is pretty slow
#define LPF_SWITCHING_DL2MAN_USDX_REV3           1   // Enable 8-band filter bank switching:     latching relays wired to a TCA/PCA9555 GPIO extender on the PC4/PC5 I2C bus; relays are using IO0.0 as common (ground), IO1.0..7 used by the individual latches K0-7 switching respectively LPFs for 10m, 15m, 17m, 20m, 30m, 40m, 60m, 80m
//#define LPF_SWITCHING_DL2MAN_USDX_REV3_NOLATCH 1   // Enable 8-band filter bank switching: non-latching relays wired to a TCA/PCA9555 GPIO extender on the PC4/PC5 I2C bus; relays are using IO0.0 as common (ground), IO1.0..7 used by the individual latches K0-7 switching respectively LPFs for 10m, 15m, 17m, 20m, 30m, 40m, 60m, 80m. Enable this if you are using 8-band non-latching version for the relays, the radio will draw extra 15mA current but will work ity any relay (Tnx OH2UDS/TA7W Baris)
//#define LPF_SWITCHING_DL2MAN_USDX_REV2         1   // Enable 5-band filter bank switching:     latching relays wired to a TCA/PCA9555 GPIO extender on the PC4/PC5 I2C bus; relays are using IO0.1 as common (ground), IO0.3, IO0.5, IO0.7, IO1.1, IO1.3 used by the individual latches K1-5 switching respectively LPFs for 20m, 30m, 40m, 60m, 80m
//#define LPF_SWITCHING_DL2MAN_USDX_REV2_BETA    1   // Enable 5-band filter bank switching:     latching relays wired to a PCA9539PW   GPIO extender on the PC4/PC5 I2C bus; relays are using IO0.1 as common (ground), IO0.3, IO0.5, IO0.7, IO1.1, IO1.3 used by the individual latches K1-5 switching respectively LPFs for 20m, 30m, 40m, 60m, 80m
//#define LPF_SWITCHING_DL2MAN_USDX_REV1         1   // Enable 3-band filter bank switching:     latching relays wired to a PCA9536D    GPIO extender on the PC4/PC5 I2C bus; relays are using IO0 as common (ground), IO1-IO3 used by the individual latches K1-3 switching respectively LPFs for 20m, 40m, 80m
//#define LPF_SWITCHING_WB2CBA_USDX_OCTOBAND     1   // Enable 8-band filter bank switching: non-latching relays wired to a MCP23008    GPIO extender on the PC4/PC5 I2C bus; relays are using GND as common (ground), GP0..7 used by the individual latches K1-8 switching respectively LPFs for 80m, 60m, 40m, 30m, 20m, 17m, 15m, 10m
//#define LPF_SWITCHING_PE1DDA_USDXDUO           14  // Enable 2-band filter bank switching: non-latching relay  wired to pin PD5 (pin 11); specify as value the frequency in MHz for which (and above) the relay should be altered (e.g. put 14 to enable the relay at 14MHz and above to use the 20m LPF).
#define SI5351_ADDR   0x60   // SI5351A I2C address: 0x60 for SI5351A-B-GT, Si5351A-B04771-GT, MS5351M; 0x62 for SI5351A-B-04486-GT; 0x6F for SI5351A-B02075-GT; see here for other variants: https://www.silabs.com/TimingUtility/timing-download-document.aspx?OPN=Si5351A-B02075-GT&OPNRevision=0&FileType=PublicAddendum
//#define F_MCU   16000000   // 16MHz ATMEGA328P crystal (enable for unmodified Arduino Uno/Nano boards with 16MHz crystal). You may change this value to any other crystal frequency (up to 28MHz may work)

// Advanced configuration switches
//#define CONDENSED      1   // Display in 4 line mode (for OLED and LCD2004 modules)
//#define CAT_EXT        1   // Extended CAT support: remote button and screen control commands over CAT
//#define CAT_STREAMING  1   // Extended CAT support: audio streaming over CAT, once enabled and triggered with CAT cmd, samplerate 7812Hz, 8-bit unsigned audio is sent over UART. The ";" is omited in the data-stream, and only sent to indicate the beginning and end of a CAT cmd.
#define CW_DECODER       1   // CW decoder
#define TX_ENABLE        1   // Disable this for RX only (no transmit), e.g. to support uSDX for kids idea: https://groups.io/g/ucx/topic/81030243#6276
#define KEY_CLICK        1   // Reduce key clicks by envelope shaping
#define SEMI_QSK         1   // Just after keying the transmitter, keeps the RX muted for a short amount of time in the anticipation for continued keying
#define RIT_ENABLE       1   // Receive-In-Transit alternates the receiving frequency with an user-defined offset to compensate for any necessary tuning needed on receive
#define VOX_ENABLE       1   // Voice-On-Xmit which is switching the transceiver into transmit as soon audio is detected (above noise gate level)
//#define MOX_ENABLE     1   // Monitor-On-Xmit which is audio monitoring on speaker during transmit
//#define FAST_AGC       1   // Adds fast AGC option (good for CW)
//#define VSS_METER      1   // Supports Vss measurement (as s-meter option), requires resistor of 1M between 12V and pin 26 (PC3)
//#define SWR_METER      1   // Supports SWR meter with bridge on A6/A7 (LQPF ATMEGA328P) by Alain, K1FM, see: https://groups.io/g/ucx/message/6262 and https://groups.io/g/ucx/message/6361
//#define ONEBUTTON      1   // Use single (encoder) button to control full the rig; optionally use L/R buttons to completely replace rotory encoder function
//#define DEBUG          1   // for development purposes only (adds debugging features such as CPU, sample-rate measurement, additional parameters)
//#define TESTBENCH      1   // Tests RX chain by injection of sine wave, measurements results are sent over serial
//#define CW_FREQS_QRP   1   // Defaults to CW QRP   frequencies when changing bands
//#define CW_FREQS_FISTS 1   // Defaults to CW FISTS frequencies when changing bands

#define CW_MESSAGE       1   // Transmits pre-defined CW messages on-demand (left-click menu item 4.2)
//#define CW_MESSAGE_EXT 1   // Additional CW messages

//put your call and name here, it will be used to generate the predefined messages
#define MYCALL "PE1NNN"
#define MYNAME "GUIDO"

// predefined CW messages
#define CW_MSG1 "CQ " MYCALL " +"
// CW_MSG2-5 are used only when CW_MESSAGE_EXT is set
#define CW_MSG2 "CQ CQ DE PE1NNN PE1NNN +"
#define CW_MSG3 "GE TKS 5NN 5NN NAME IS " MYNAME " " MYNAME " HW?"
#define CW_MSG4 "FB RPTR TX 5W 5W ANT INV V 73 CUAGN"
#define CW_MSG5 "73 TU E E"
#define CW_MSG6 MYCALL

//#define TX_DELAY       1   // Enables a delay in the actual transmission to allow relay-switching to be completed before the power is applied (see also NTX, PTX definitions below for GPIO that can switch relay/PA)
//#define NTX            11  // Enables LOW  on TX, used as PTT out to enable external PAs (a value of 11 means PB3 is used)
//#define PTX            11  // Enables HIGH on TX, used as PTT out to enable external PAs (a value of 11 means PB3 is used)
//#define CLOCK          1   // Enables clock
#define CW_INTERMEDIATE  1   // CW decoder shows intermediate characters (only available for LCD and F_MCU at 20M), sequences like:  EIS[HV] EIUF EAW[JP] EARL TMO TMG[ZQ] TND[BX] TNK[YC], may be good to learn CW; a full list of possible sequences:  EISH5 EISV3 EIUF EIUU2 EAWJ1 EAWP EARL TMOO0 TMOO9 TMOO8 TMGZ7 TMGQ TNDB6 TNDX TNKY TNKC
//#define F_XTAL  20000000   // Enable this for uSDXDuO, 20MHz SI5351 crystal
//#define TX_CLK0_CLK1   1   // Enable this for uSDXDuO, i.e. when PA is driven by CLK0, CLK1 (not CLK2); NTX pin may be used for enabling the TX path (this is like RX pin, except that RX may also be used as attenuator)
//#define F_CLK2  12000000   // Enables a fixed CLK2 clock output of choice (only applicable when TX_CLK0_CLK1 is enabled), e.g. for up-converter or to clock UART USB device
