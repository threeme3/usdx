// Arduino Sketch of the QCX-SSB: SSB (+ SDR) with your QCX transceiver
//
// https://github.com/threeme3/QCX-SSB

#define VERSION   "1.01h"

// QCX pin defintion
#define LCD_D4  0
#define LCD_D5  1
#define LCD_D6  2
#define LCD_D7  3
#define LCD_EN  4
#define ROT_A   6
#define ROT_B   7
#define RX      8
#define SIDETONE 9
#define KEY_OUT 10
#define SIG_OUT 11
#define DAH     12
#define DIT     13
#define AUDIO1  PIN_A0
#define AUDIO2  PIN_A1
#define DVM     PIN_A2
#define BUTTONS PIN_A3
#define LCD_RS  18
#define SDA     18 //shared with LCD_RS
#define SCL     19

#include <LiquidCrystal.h>
class QCXLiquidCrystal : public LiquidCrystal {  // this class is used because QCX shares some of the LCD lines with the SI5351 lines, and these do have an absolute max rating of 3V6
public: // QCXLiquidCrystal extends LiquidCrystal library for pull-up driven LCD_RS, as done on QCX. LCD_RS needs to be set to LOW in advance of calling any operation.
  QCXLiquidCrystal(uint8_t rs, uint8_t en, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7) : LiquidCrystal(rs, en, d4, d5, d6, d7){ };
  virtual size_t write(uint8_t value){ // overwrites LiquidCrystal::write() and re-implements LCD data writes
    pinMode(LCD_RS, INPUT);  // pull-up LCD_RS
    write4bits(value >> 4);
    write4bits(value);
    pinMode(LCD_RS, OUTPUT); // restore LCD_RS as output, so that potential command write can be handled (LCD_RS pull-down is causing RFI, so better to prevent it if possible)
    return 1;
  };
  void write4bits(uint8_t value){
    digitalWrite(LCD_D4, (value >> 0) & 0x01);
    digitalWrite(LCD_D5, (value >> 1) & 0x01);
    digitalWrite(LCD_D6, (value >> 2) & 0x01);
    digitalWrite(LCD_D7, (value >> 3) & 0x01);
    digitalWrite(LCD_EN, LOW);  //delayMicroseconds(1);  // pulseEnable
    digitalWrite(LCD_EN, HIGH); //delayMicroseconds(1);  // enable pulse must be >450ns
    digitalWrite(LCD_EN, LOW);  //delayMicroseconds(50); // commands need > 37us to settle
  };
};
QCXLiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

#include <inttypes.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#undef F_CPU
#define F_CPU 20008440   // myqcx1:20008440   // Actual crystal frequency of XTAL1 20000000

#define I2C_DELAY   4    // Determines I2C Speed (2=939kb/s (too fast!!); 3=822kb/s; 4=731kb/s; 5=658kb/s; 6=598kb/s). Increase this value when you get I2C tx errors (E05); decrease this value when you get a CPU overload (E01). An increment eats ~3.5% CPU load; minimum value is 3 on my QCX, resulting in 84.5% CPU load
#define I2C_DDR DDRC     // Pins for the I2C bit banging
#define I2C_PIN PINC
#define I2C_PORT PORTC
#define I2C_SDA (1 << 4) // PC4
#define I2C_SCL (1 << 5) // PC5
#define DELAY(n) for(uint8_t i = 0; i != n; i++) asm("nop");

#define I2C_SDA_GET() I2C_PIN & I2C_SDA
#define I2C_SCL_GET() I2C_PIN & I2C_SCL
#define I2C_SDA_HI() I2C_DDR &= ~I2C_SDA;
#define I2C_SDA_LO() I2C_DDR |=  I2C_SDA;
#define I2C_SCL_HI() I2C_DDR &= ~I2C_SCL; DELAY(I2C_DELAY);
#define I2C_SCL_LO() I2C_DDR |=  I2C_SCL; DELAY(I2C_DELAY);

inline void i2c_start()
{
  i2c_resume();  //prepare for I2C
  I2C_SCL_LO();
  I2C_SDA_HI();
}

inline void i2c_stop()
{
  I2C_SCL_HI();
  I2C_SDA_HI();
  I2C_DDR &= ~(I2C_SDA | I2C_SCL); // prepare for a start: pull-up both SDA, SCL
  i2c_suspend();
}

#define i2c_SendBit(data, mask) \
  if(data & mask){ \
    I2C_SDA_HI();  \
  } else {         \
    I2C_SDA_LO();  \
  }                \
  I2C_SCL_HI();    \
  I2C_SCL_LO();

inline void i2c_SendByte(uint8_t data)
{
  i2c_SendBit(data, 1 << 7);
  i2c_SendBit(data, 1 << 6);
  i2c_SendBit(data, 1 << 5);
  i2c_SendBit(data, 1 << 4);
  i2c_SendBit(data, 1 << 3);
  i2c_SendBit(data, 1 << 2);
  i2c_SendBit(data, 1 << 1);
  i2c_SendBit(data, 1 << 0);
  I2C_SDA_HI();  // recv ACK
  DELAY(I2C_DELAY);
  I2C_SCL_HI();
  I2C_SCL_LO();
}

inline uint8_t i2c_RecvBit(uint8_t mask)
{
  I2C_SCL_HI();
  uint16_t i = 60000;
  for(;(!I2C_SCL_GET()) && i; i--);  // wait util slave release SCL to HIGH (meaning data valid), or timeout at 3ms
  if(!i){ lcd.setCursor(0, 1); lcd.print("E07 I2C timeout"); }
  uint8_t data = I2C_SDA_GET();
  I2C_SCL_LO();
  return (data) ? mask : 0;
}

inline uint8_t i2c_RecvByte(uint8_t last)
{
  uint8_t data = 0;
  data |= i2c_RecvBit(1 << 7);
  data |= i2c_RecvBit(1 << 6);
  data |= i2c_RecvBit(1 << 5);
  data |= i2c_RecvBit(1 << 4);
  data |= i2c_RecvBit(1 << 3);
  data |= i2c_RecvBit(1 << 2);
  data |= i2c_RecvBit(1 << 1);
  data |= i2c_RecvBit(1 << 0);
  if(last){
    I2C_SDA_HI();  // NACK
  } else {
    I2C_SDA_LO();  // ACK
  }
  DELAY(I2C_DELAY);
  I2C_SCL_HI();
  I2C_SDA_HI();    // restore SDA for read
  I2C_SCL_LO();
  return data;
}

void i2c_init()
{  
  I2C_PORT &= ~( I2C_SDA | I2C_SCL );
  I2C_SCL_HI();
  I2C_SDA_HI();
  i2c_suspend();
}

void i2c_deinit()
{
  I2C_PORT &= ~( I2C_SDA | I2C_SCL );
  I2C_DDR &= ~( I2C_SDA | I2C_SCL );
}

inline void i2c_resume()
{
#ifdef LCD_RS_PORTIO
  I2C_PORT &= ~I2C_SDA; // pin sharing SDA/LCD_RS mitigation
#endif
}

inline void i2c_suspend()
{
  I2C_SDA_LO();         // pin sharing SDA/LCD_RS: pull-down LCD_RS; QCXLiquidCrystal require this for any operation
}

#define SI_I2C_ADDR 96  // SI5351A I2C address
#define SI_CLK_OE 3     // Register definitions
#define SI_CLK0_CONTROL 16
#define SI_CLK1_CONTROL 17
#define SI_CLK2_CONTROL 18
#define SI_SYNTH_PLL_A 26
#define SI_SYNTH_PLL_B 34
#define SI_SYNTH_MS_0 42
#define SI_SYNTH_MS_1 50
#define SI_SYNTH_MS_2 58
#define SI_CLK0_PHOFF 165
#define SI_CLK1_PHOFF 166
#define SI_CLK2_PHOFF 167
#define SI_PLL_RESET 177

#define SI_MS_INT 0b01000000  // Clock control
#define SI_CLK_SRC_PLL_A 0b00000000
#define SI_CLK_SRC_PLL_B 0b00100000
#define SI_CLK_SRC_MS 0b00001100
#define SI_CLK_IDRV_8MA 0b00000011
#define SI_CLK_INV 0b00010000

#define SI_XTAL_FREQ 27004900  //myqcx1:27003847 myqcx2:27004900  Measured crystal frequency of XTAL2 for CL = 10pF (default), calibrate your QCX 27MHz crystal frequency here

#define log2(n) (log(n) / log(2))

volatile uint8_t si5351_prev_divider;
volatile int32_t si5351_raw_freq;
volatile uint8_t si5351_divider;  // note: because of int8 only freq > 3.6MHz can be covered for R_DIV=1
volatile uint8_t si5351_mult;
volatile uint8_t si5351_pll_data[8];
static int32_t si5351_prev_pll_freq = 0;

uint8_t si5351_RecvRegister(uint8_t reg)
{
  // Data write to set the register address
  i2c_start();
  i2c_SendByte(SI_I2C_ADDR << 1);
  i2c_SendByte(reg);
  i2c_stop();
  // Data read to retrieve the data from the set address
  i2c_start();
  i2c_SendByte((SI_I2C_ADDR << 1) | 1);
  uint8_t data = i2c_RecvByte(true);
  i2c_stop();
  return data;
}

void si5351_SendRegister(uint8_t reg, uint8_t data)
{
  i2c_start();
  i2c_SendByte(SI_I2C_ADDR << 1);
  i2c_SendByte(reg);
  i2c_SendByte(data);
  i2c_stop();
}

inline void si5351_SendPLLBRegisterBulk()  // fast freq change of PLLB, takes about [ 2 + 7*(8+1) + 2 ] / 840000 = 80 uS
{
  i2c_start();
  i2c_SendByte(SI_I2C_ADDR << 1);
  i2c_SendByte(SI_SYNTH_PLL_B + 3);  // Skip the first three pll_data bytes (first two always 0xFF and third not often changing
  i2c_SendByte(si5351_pll_data[3]);
  i2c_SendByte(si5351_pll_data[4]);
  i2c_SendByte(si5351_pll_data[5]);
  i2c_SendByte(si5351_pll_data[6]);
  i2c_SendByte(si5351_pll_data[7]);
  i2c_stop();
}

// Set up MultiSynth for register reg=MSNA, MNSB, MS0-5 with fractional divider, num and denom and R divider (for MSn, not for MSNA, MSNB)
// divider is 15..90 for MSNA, MSNB,  divider is 8..900 (and in addition 4,6 for integer mode) for MS[0-5]
// num is 0..1,048,575 (0xFFFFF)
// denom is 0..1,048,575 (0xFFFFF)
// num = 0, denom = 1 forces an integer value for the divider
// r_div = 1..128 (1,2,4,8,16,32,64,128)
void si5351_SetupMultisynth(uint8_t reg, uint8_t divider, uint32_t num, uint32_t denom, uint8_t r_div)
{
  uint32_t P1; // Synth config register P1
  uint32_t P2; // Synth config register P2
  uint32_t P3; // Synth config register P3

  P1 = (uint32_t)(128 * ((float)num / (float)denom));
  P1 = (uint32_t)(128 * (uint32_t)(divider) + P1 - 512);
  P2 = (uint32_t)(128 * ((float)num / (float)denom));
  P2 = (uint32_t)(128 * num - denom * P2);
  P3 = denom;

  si5351_SendRegister(reg + 0, (P3 & 0x0000FF00) >> 8);
  si5351_SendRegister(reg + 1, (P3 & 0x000000FF));
  si5351_SendRegister(reg + 2, (P1 & 0x00030000) >> 16 | ((int)log2(r_div) << 4) );
  si5351_SendRegister(reg + 3, (P1 & 0x0000FF00) >> 8);
  si5351_SendRegister(reg + 4, (P1 & 0x000000FF));
  si5351_SendRegister(reg + 5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
  si5351_SendRegister(reg + 6, (P2 & 0x0000FF00) >> 8);
  si5351_SendRegister(reg + 7, (P2 & 0x000000FF));
}

// this function relies on cached (global) variables: si5351_divider, si5351_mult, si5351_raw_freq, si5351_pll_data
inline void si5351_freq_calc_fast(int16_t freq_offset)
{ // freq_offset is relative to freq set in si5351_freq(freq)
  // uint32_t num128 = ((si5351_divider * (si5351_raw_freq + offset)) % SI_XTAL_FREQ) * (float)(0xFFFFF * 128) / SI_XTAL_FREQ;
  // Above definition (for SI_XTAL_FREQ=27.00491M) can be optimized by pre-calculating factor (0xFFFFF*128)/SI_XTAL_FREQ (=4.97) as integer constant (5) and
  // substracting the rest error factor (0.03). Note that the latter is shifted left (0.03<<6)=2, while the other term is shifted right (>>6)
  register int32_t z = ((si5351_divider * (si5351_raw_freq + freq_offset)) % SI_XTAL_FREQ);
  register int32_t z2 = -(z >> 5);
  int32_t num128 = (z * 5) + z2;

  // Set up specified PLL with si5351_mult, num and denom: si5351_mult is 15..90, num128 is 0..128*1,048,575 (128*0xFFFFF), denom is 0..1,048,575 (0xFFFFF)
  uint32_t P1 = 128 * si5351_mult + (num128 / 0xFFFFF) - 512;
  uint32_t P2 = num128 % 0xFFFFF;
  //si5351_pll_data[0] = 0xFF;
  //si5351_pll_data[1] = 0xFF;
  //si5351_pll_data[2] = (P1 >> 14) & 0x0C;
  si5351_pll_data[3] = P1 >> 8;
  si5351_pll_data[4] = P1;
  si5351_pll_data[5] = 0xF0 | (P2 >> 16);
  si5351_pll_data[6] = P2 >> 8;
  si5351_pll_data[7] = P2;
}

uint16_t si5351_div(uint32_t num, uint32_t denom, uint32_t* b, uint32_t* c)
{ // returns a + b / c = num / denom, where a is the integer part and b and c is the optional fractional part 20 bits each (range 0..1048575)
  uint16_t a = num / denom;
  if(b && c){
    uint64_t l = num % denom;
    l <<= 20; l--;  // l *= 1048575;
    l /= denom;     // normalize
    *b = l;
    *c = 0xFFFFF;    // for simplicity set c to the maximum 1048575
  }
  return a;
}

volatile int32_t pll_freq;   // temporary

void si5351_freq(uint32_t freq, uint8_t i, uint8_t q)
{ // Fout = Fvco / (R * [MSx_a + MSx_b/MSx_c]),  Fvco = Fxtal * [MSPLLx_a + MSPLLx_b/MSPLLx_c]; MSx as integer reduce spur
  uint8_t r_div = (freq > 4000000) ? 1 : (freq > 400000) ? 32 : 128; // helps si5351_divider to be in range
  freq *= r_div;  // take r_div into account, now freq is in the range 1MHz to 150MHz
  si5351_raw_freq = freq;   // cache frequency generated by PLL and MS stages (excluding R divider stage); used by si5351_freq_calc_fast()

  si5351_divider = 900000000 / freq;  // Calculate the division ratio. 900,000,000 is the maximum internal PLL freq (official range 600..900MHz but can be pushed to 300MHz..~1200Mhz)
  if(si5351_divider % 2) si5351_divider--;  // si5351_divider in range 8.. 900 (including 4,6 for integer mode), even numbers preferred. Note that uint8 datatype is used, so 254 is upper limit
  if( (si5351_divider * (freq - 5000) / SI_XTAL_FREQ) != (si5351_divider * (freq + 5000) / SI_XTAL_FREQ) ) si5351_divider -= 2; // Test if si5351_multiplier remains same for freq deviation +/- 5kHz, if not use different si5351_divider to make same
  /*int32_t*/ pll_freq = si5351_divider * freq; // Calculate the pll_freq: the si5351_divider * desired output freq
  uint32_t num, denom;
  si5351_mult = si5351_div(pll_freq, SI_XTAL_FREQ, &num, &denom); // Determine the mult to get to the required pll_freq (in the range 15..90)

  // Set up specified PLL with mult, num and denom: mult is 15..90, num is 0..1,048,575 (0xFFFFF), denom is 0..1,048,575 (0xFFFFF)
  // Set up PLL A and PLL B with the calculated  multiplication ratio
  si5351_SetupMultisynth(SI_SYNTH_PLL_A, si5351_mult, num, denom, 1);
  si5351_SetupMultisynth(SI_SYNTH_PLL_B, si5351_mult, num, denom, 1);
  //if(denom == 1) si5351_SendRegister(22, SI_MSx_INT); // FBA_INT: MSNA operates in integer mode
  //if(denom == 1) si5351_SendRegister(23, SI_MSx_INT); // FBB_INT: MSNB operates in integer mode
  // Set up MultiSynth 0,1,2 with the calculated divider, from 4, 6..1800.
  // The final R division stage can divide by a power of two, from 1..128
  // if you want to output frequencies below 1MHz, you have to use the final R division stage
  si5351_SetupMultisynth(SI_SYNTH_MS_0, si5351_divider, 0, 1, r_div);
  si5351_SetupMultisynth(SI_SYNTH_MS_1, si5351_divider, 0, 1, r_div);
  si5351_SetupMultisynth(SI_SYNTH_MS_2, si5351_divider, 0, 1, r_div);
  //if(si5351_prev_divider != si5351_divider){ lcd.setCursor(0, 0); lcd.print(si5351_divider); lcd.print(blanks); }
  // Set I/Q phase
  si5351_SendRegister(SI_CLK0_PHOFF, i * si5351_divider / 90); // one LSB equivalent to a time delay of Tvco/4 range 0..127
  si5351_SendRegister(SI_CLK1_PHOFF, q * si5351_divider / 90); // one LSB equivalent to a time delay of Tvco/4 range 0..127
  // Switch on the CLK0, CLK1 output to be PLL A and set si5351_multiSynth0, si5351_multiSynth1 input (0x0F = SI_CLK_SRC_MS | SI_CLK_IDRV_8MA)
  si5351_SendRegister(SI_CLK0_CONTROL, 0x0F | SI_MS_INT | SI_CLK_SRC_PLL_A);
  si5351_SendRegister(SI_CLK1_CONTROL, 0x0F | SI_MS_INT | SI_CLK_SRC_PLL_A);
  // Switch on the CLK2 output to be PLL B and set si5351_multiSynth2 input
  si5351_SendRegister(SI_CLK2_CONTROL, 0x0F | SI_MS_INT | SI_CLK_SRC_PLL_B);
  si5351_SendRegister(SI_CLK_OE, 0b11111100); // Enable CLK1|CLK0
  // Reset the PLL. This causes a glitch in the output. For small changes to
  // the parameters, you don't need to reset the PLL, and there is no glitch
  if((abs(pll_freq - si5351_prev_pll_freq) > 16000000L) || si5351_divider != si5351_prev_divider){
    si5351_prev_pll_freq = pll_freq;
    si5351_prev_divider = si5351_divider;
    si5351_SendRegister(SI_PLL_RESET, 0xA0);
  }
}

void si5351_alt_clk2(uint32_t freq)
{
  uint32_t num, denom;
  uint16_t mult = si5351_div(pll_freq, freq, &num, &denom);

  si5351_SetupMultisynth(SI_SYNTH_MS_2, mult, num, denom, 1);

  // Switch on the CLK2 output to be PLL A and set si5351_multiSynth2 input
  si5351_SendRegister(SI_CLK2_CONTROL, 0x0F | SI_CLK_SRC_PLL_A);

  si5351_SendRegister(SI_CLK_OE, 0b11111000); // Enable CLK2|CLK1|CLK0

  //si5351_SendRegister(SI_CLK0_PHOFF, 0 * si5351_divider / 90); // one LSB equivalent to a time delay of Tvco/4 range 0..127
  //si5351_SendRegister(SI_CLK1_PHOFF, 90 * si5351_divider / 90); // one LSB equivalent to a time delay of Tvco/4 range 0..127
  //si5351_SendRegister(SI_CLK2_PHOFF, 45 * si5351_divider / 90); // one LSB equivalent to a time delay of Tvco/4 range 0..127
  //si5351_SendRegister(SI_PLL_RESET, 0xA0);
}

void si5351_powerDown()
{
  si5351_SendRegister(SI_CLK0_CONTROL, 0b11000000);  // Conserve power when output is dsiabled
  si5351_SendRegister(SI_CLK1_CONTROL, 0b11000000);
  si5351_SendRegister(SI_CLK2_CONTROL, 0b11000000);
  si5351_SendRegister(19, 0b11000000);
  si5351_SendRegister(20, 0b11000000);
  si5351_SendRegister(21, 0b11000000);
  si5351_SendRegister(22, 0b11000000);
  si5351_SendRegister(23, 0b11000000);
  si5351_SendRegister(SI_CLK_OE, 0b11111111); // Disable all CLK outputs
}

const char* mode_label[] = { "LSB", "USB", "CW ", "AM ", "FM " };
enum mode_t { LSB, USB, CW, AM, FM };
volatile uint8_t mode = USB;
volatile bool change = true;
volatile int32_t freq = 7074000;

volatile bool ptt = false;
volatile uint8_t tx = 0;
volatile bool vox_enable = false;
enum dsp_cap_t { ANALOG, DSP, SDR };
static uint8_t dsp_cap = 0;
static uint8_t ssb_cap = 0;
volatile bool att_enable = false;

inline void txen(bool en)
{
  if(en){
      lcd.setCursor(15, 1); lcd.print("T");
      si5351_SendRegister(SI_CLK_OE, 0b11111011); // CLK2_EN=1, CLK1_EN,CLK0_EN=0
      digitalWrite(RX, LOW);  // TX
  } else {
      digitalWrite(RX, !(att_enable)); // RX (enable RX when attenuator not on)
      si5351_SendRegister(SI_CLK_OE, 0b11111100); // CLK2_EN=0, CLK1_EN,CLK0_EN=1
      lcd.setCursor(15, 1); lcd.print((vox_enable) ? "V" : "R");
  }
}

inline void vox(bool trigger)
{
  if(trigger){
    if(!tx) txen(true);
    tx = (vox_enable) ? 255 : 1; // hangtime = 255 / 4402 = 58ms (the time that TX at least stays on when not triggered again)
  } else {
    if(tx){
      tx--;
      if(!tx) txen(false);
    }
  }
}

volatile uint8_t drive = 4;
//#define F_SAMP_TX 4402
#define F_SAMP_TX 4810        //4810 // ADC sample-rate; is best a multiple of _UA and fits exactly in OCR0A = ((F_CPU / 64) / F_SAMP_TX) - 1 , should not exceed CPU utilization
#define _UA  (F_SAMP_TX)      //360  // unit angle; integer representation of one full circle turn or 2pi radials or 360 degrees, should be a integer divider of F_SAMP_TX and maximized to have higest precision
//#define MAX_DP  (_UA/1)  //(_UA/2) // the occupied SSB bandwidth can be further reduced by restricting the maximum phase change (set MAX_DP to _UA/2).

inline int16_t arctan3(int16_t q, int16_t i)  // error ~ 0.8 degree
{ // source: [1] http://www-labs.iro.umontreal.ca/~mignotte/IFT2425/Documents/EfficientApproximationArctgFunction.pdf
#define _atan2(z)  (_UA/8 - _UA/22 * z + _UA/22) * z  //derived from (5) [1]
  //#define _atan2(z)  (_UA/8 - _UA/24 * z + _UA/24) * z  //derived from (7) [1]
  int16_t r;
  if(abs(q) > abs(i))
    r = _UA / 4 - _atan2(abs(i) / abs(q));        // arctan(z) = 90-arctan(1/z)
  else
    r = (i == 0) ? 0 : _atan2(abs(q) / abs(i));   // arctan(z)
  r = (i < 0) ? _UA / 2 - r : r;                  // arctan(-z) = -arctan(z)
  return (q < 0) ? -r : r;                        // arctan(-z) = -arctan(z)
}

#define magn(i, q) (abs(i) > abs(q) ? abs(i) + abs(q) / 4 : abs(q) + abs(i) / 4) // approximation of: magnitude = sqrt(i*i + q*q); error 0.95dB

uint8_t lut[256];
volatile uint8_t amp;

inline int16_t ssb(int16_t in)
{
  static int16_t dc;

  int16_t i, q;
  uint8_t j;
  static int16_t v[16];
  for(j = 0; j != 15; j++) v[j] = v[j + 1];

  dc += (in - dc) / 2;
  v[15] = in - dc;     // DC decoupling
  //dc = in;  // this is actually creating a low-pass filter

  i = v[7];
  q = ((v[0] - v[14]) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 15) / 128 + (v[6] - v[8]) / 2; // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)

  uint16_t _amp = magn(i, q);

#define VOX_THRESHOLD (1 << 2)  // 2*6dB above ADC noise level
  if(vox_enable) vox(_amp > VOX_THRESHOLD);
  else vox(ptt);
//  else txen(ptt);
//  vox((ptt) || ((vox_enable) && (_amp > VOX_THRESHOLD)) );

  _amp = _amp << drive;
  _amp = ((_amp > 255) || (drive == 8)) ? 255 : _amp; // clip or when drive=8 use max output
  amp = (tx) ? lut[_amp] : 0;

  static int16_t prev_phase;
  int16_t phase = arctan3(q, i);
  int16_t dp = phase - prev_phase;  // phase difference and restriction
  prev_phase = phase;

  if(dp < 0) dp = dp + _UA; // make negative phase shifts positive: prevents negative frequencies and will reduce spurs on other sideband
#ifdef MAX_DP
  if(dp > MAX_DP){ // dp should be less than half unit-angle in order to keep frequencies below F_SAMP_TX/2
    prev_phase = phase - (dp - MAX_DP);  // substract restdp
    dp = MAX_DP;
  }
#endif
  if(mode == USB)
    return dp * ( F_SAMP_TX / _UA); // calculate frequency-difference based on phase-difference
  else
    return dp * (-F_SAMP_TX / _UA);
}

//#define PROFILING  1
#ifdef PROFILING
volatile uint32_t numSamples = 0;
#else
volatile uint16_t numSamples = 0;
#endif
#define MIC_ATTEN  0  // 0*6dB attenuation (note that the LSB bits are quite noisy)

// This is the ADC ISR, issued with sample-rate via timer1 compb interrupt.
// It performs in real-time the ADC sampling, calculation of SSB phase-differences, calculation of SI5351 frequency registers and send the registers to SI5351 over I2C.
void dsp_tx()
{ // jitter dependent things first
  ADCSRA |= (1 << ADSC);    // start next ADC conversion (trigger ADC interrupt if ADIE flag is set)
  OCR1BL = amp;                        // submit amplitude to PWM register (actually this is done in advance (about 140us) of phase-change, so that phase-delays in key-shaping circuit filter can settle)
  si5351_SendPLLBRegisterBulk();       // submit frequency registers to SI5351 over 731kbit/s I2C (transfer takes 64/731 = 88us, then PLL-loopfilter probably needs 50us to stabalize)
  //OCR1BL = amp;                        // submit amplitude to PWM register (takes about 1/32125 = 31us+/-31us to propagate) -> amplitude-phase-alignment error is about 30-50us
  int16_t adc = (ADCL | (ADCH << 8)) - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  int16_t df = ssb(adc >> MIC_ATTEN);  // convert analog input into phase-shifts (carrier out by periodic frequency shifts)
  si5351_freq_calc_fast(df);           // calculate SI5351 registers based on frequency shift and carrier frequency
  numSamples++;
}

int16_t dsp(int16_t in)
{
  static int16_t dc;

  int16_t i, q;
  uint8_t j;
  static int16_t v[16];
  for(j = 0; j != 15; j++) v[j] = v[j + 1];

  dc += (in - dc) / 2;
  v[15] = in - dc;     // DC decoupling

  i = v[7];
  q = ((v[0] - v[14]) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 15) / 128 + (v[6] - v[8]) / 2; // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)
  return abs(i) > abs(q) ? abs(i) + abs(q) / 4 : abs(q) + abs(i) / 4; // approximation of: amp = sqrt(i*i + q*q); error 0.95dB
}

static int32_t signal;
static int16_t avg = 0;
static int16_t maxpk=0;
static int16_t k0=0;
static int16_t k1=0;
static uint8_t sym;
static int16_t ta=0;
static char m2c[] = "##ETIANMSURWDKGOHVF#L#PJBXCYZQ##54S3###2##+###J16=/###H#7#G#8#90############?_####\"##.####@###'##-########;!#)#####,####:####";

char cw(int16_t in)
{
  char ch = 0;
  int i;
  signal += dsp(in);
  #define OSR 64 // (8*FS/1000)
  if((numSamples % OSR) == 0){   // process every 8 ms
    if(!signal) return ch;
    signal = signal / OSR;  //normalize
    maxpk = signal > maxpk ? signal : maxpk;
    #define RT  4
    if(signal>(maxpk/2) ){  // threshold: 3dB below max signal
      k1++;  // key on
      k0=0;
    } else {
      k0++;  // key off
      if(k0>0 && k1>0){        //symbol space
        if(k1>(ta/100)) ta=RT*ta/100+(100-RT)*k1;                   // go slower
        if(k1>(ta/600) && k1<(ta/300)) ta=(100-RT)*ta/100+RT*k1*3;  // go faster
        if(k1>(ta/600)) sym=(sym<<1)|(k1>(ta/200));        // dit (0) or dash (1)
        k1=0;
      }
      if(k0>=(ta/200) && sym>1){ //letter space
        if(sym<128) ch=m2c[sym];
        sym=1;
      }
      if(k0>=(ta/67)){        //word space (67=1.5/100)
        ch = ' ';
        k0=-1000*(ta/100);           //delay word spaces
      }
    }
    avg = avg*99/100 + signal*1/100;
    maxpk = maxpk*99/100 + signal*1/100;
    signal = 0;
  }
  return ch;
}

inline int16_t filt_cwn(int16_t v)
{ // 2nd Order Bandpass 650-840Hz (SR=8kHz) IIR in Direct Form I
  float zx0 = v;

  static float za1,za2;
  static float zb1,zb2;
  zx0=(zx0+-2.0*za1+za2+21.0*zb1+-11.6*zb2)*5.0/64.0;  //zx0=(5.0*zx0+-10.0*za1+5.0*za2+105.0*zb1+-58.0*zb2)/64.0;
  za2=za1;
  za1=v;
  zb2=zb1;
  zb1=zx0;

  static float zc1,zc2;
  zx0=(zx0+2.0*zb1+zb2+97.0*zc1+-57.0*zc2)/64.0;
  zc2=zc1;
  zc1=zx0;

  return zx0;
}

static float gain = 1.0;

static char out[] = "                ";
volatile bool cw_event = false;

volatile uint8_t volume = 8;
//#define F_SAMP_RX 156250
//#define F_SAMP_RX 78125
//#define F_SAMP_RX 62500  //overrun; sample rate of 55500 can be obtained
#define F_SAMP_RX 52083
//#define F_SAMP_RX 44643
//#define F_SAMP_RX 39062
//#define F_SAMP_RX 34722
//#define F_SAMP_RX 31250
//#define F_SAMP_RX 28409
#define F_ADC_CONV 192307

inline int16_t agc(int16_t in)
{  // source: Lyons Understanding Digital Signal Processing 3rd edition 13.30
  float out = in * gain;
  //#define ref_level 32 /* 32 */  // average reference level (volume)
  #define ref_level volume
  #define alpha 0.0000001 /* very_slow=0.00000005 slow=0.0000001 medium=0.0000005 fast=0.000001 */ // time constant
  gain = gain + (ref_level * ref_level - (out * out)) * alpha;
  return out;
}

#define JUNK  1
#ifdef JUNK
// This is here only for historical purpose and because of code alignment (makes sdr_rx faster)
void dsp_rx()
{ // jitter dependent things first
  ADCSRA |= (1 << ADSC);    // start next ADC conversion (trigger ADC interrupt if ADIE flag is set)
  int16_t adc = (ADCL | (ADCH << 8)) - 512; // ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH

  static int16_t dc;
  dc += (adc - dc) / 2;
  int16_t ac = adc - dc;     // DC decoupling
//#define RX_SIMPLE  1
#ifdef RX_SIMPLE
  ac = agc(ac);
  if(mode == CW) ac = filt_cwn(ac /* *64 */ * 16 );
  OCR1AL = ac + 128;
#endif

#define RX_CIC  1
#ifdef RX_CIC
  // Decimating 2nd Order CIC filter
  #define R 8  // Rate change from 62.5kSPS to 7812.5SPS
  static int16_t zi1, zi2, zd1, zd2;
  zi2 = zi1 + zi2;  // Integrator section
  zi1 = ac + zi1;
  if((numSamples % R) == 0){
    int16_t d1 = zi2 - zd1;  // Comb section
    ac = d1 - zd2;
    zd2 = d1;
    zd1 = zi2;
    
    if(mode == CW) filt_cwn(ac >> (16-volume));//filt_cwn(ac * volume/256 ); //ac = filt_cwn(ac /* *64 */ * 4 );
    else ac = (volume) ? ac >> (16-volume) : 0;//ac * volume/256;//ac >> drive; //ac >> 3; //ac = agc(ac);
    
    ac=min(max(ac, -128), 127);  // clip
    OCR1AL = ac + 128;
  }
#endif
//#define CW_DECODER  1
#ifdef CW_DECODER
  char ch = cw(adc >> 0);
  if(ch){
    for(int i=0; i!=15;i++) out[i]=out[i+1];
    out[15] = ch;
    cw_event = true;
  }
#endif
  numSamples++;
}
#endif

/*
iterate through modes always from begin, skipping the current one
code definitions and re-use for comb, integrator, dc decoupling, arctan
add sdr_rx capability to skip ADCMUX changing and Q processing, (while oversampling I branch?)
in func_ptr for different mode types
agc based on rms128
skip adc processing while smeter lcd update?
vox by sampling mic port in sdr_rx?

 */

volatile uint8_t admuxi, admuxq;
volatile uint32_t rms128 = 0;
//#define ADC_NR  1       // Stop CPU at ADC conversion, reduces noise but costs 30% CPU performance when ADC prescaler is 8

// sdr_rx() is sampling the ADC0 (I) and ADC1 (Q) inputs in alternating fashion, and generating a PWM output at OC1A. For both 
// I and Q samples, a DC-decoupling and CIC down-sampling is performed by integrating the samples and for each R samples performing 
// a comb and post-processing step where the I and Q results are demodulated into a single signal which is CIC upsampled again via 
// a comb; the upsampled signal is integrated and ouput via PWM.
// 
// The ADC sampling is governed by Timer2 compare match A interrupt. When the rate is too high for the CPU, sdr_rx() is still performing
// well: the integration of each sample takes longer than the sample-period, the next interrupt is scheduled later (once the current interrupt 
// finishes) or is missed due to a delay that has been build-up and wwhere an interrupt happens while an interrupt flag was already 
// set. As a result, the actual sample rate is a fraction of the timer rate, and the CPU is freed for the moments an interrupt 
// is missed. Since the time for sampling and integrating is roughly the same processing time for both the I and Q channel, 
// a constant sample-rate is obtained when next interrupts are delayed and scheduled. Missing samples however disturbs the 
// continuous sampling, and therefore cause a (random) phase shift errors for on the next series of samples. But, since these phase-shift 
// are instantly and and effective on both the I/Q inputs and PWM output, the resulting output do contain a constant average of 
// these phase-shift errors, and hence are not harmful. Hence, predicted sample rate = F_SAMP_RX / CPU_load_rx_avg
// [1] ATMEGA328P datasheet, chapter 6.7, "Reset and Interrupt Handling". Current performance figures:
// CPU load figures for numSamples[4..12]={250, 125, 125, 125, 125, 125, 125, 225} at R=4, Fs=62.5k, DUC on
// CPU load figures for numSamples[4..12]={187, 104, 104, 104, 104, 104, 104, 187} at R=4, Fs=52k, DUC on

void sdr_rx()
{
  static int16_t ozi1, ozi2, ozd1, ozd2, ocomb;
  static int16_t i, q, qh;
  #undef  R  // Decimating 2nd Order CIC filter
  //#define R 8  // Rate change from 62.5/2 kSPS to 3906.25SPS, providing 18dB gain
  //#define R 4  // Rate change from 62.5/2 kSPS to 7812.5SPS, providing 12dB gain
  #define R 4  // Rate change from 52.083/2 kSPS to 6510.4SPS, providing 12dB gain

  // process I for even samples
  if((numSamples % 2) == 0){  // [75% CPU@R=4;Fs=62.5k] (excluding the Comb branch and output stage)
    ADMUX = admuxq; //1 | (1 << REFS1) | (1 << REFS0); // prepare next Q conversion, 1v1 ref enabled
#ifdef ADC_NR
    interrupts();
    sleep_cpu();
//    noInterrupts();
#else
    ADCSRA |= (1 << ADSC);    // start next ADC conversion (trigger ADC interrupt if ADIE flag is set)
#endif
    int16_t adc = (ADCL | (ADCH << 8)) - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH

    // Correct I/Q sample delay by means of linear interpolation
    static int16_t prev_adc;
    int16_t corr_adc = (prev_adc + adc) / 2;
    prev_adc = adc;

    static int16_t dc;
    dc += (corr_adc - dc) / 2;
    int16_t ac = corr_adc - dc;    // DC decoupling

    static int16_t zi1, zi2, zd1, zd2;
    zi2 = zi1 + zi2;          // Integrator section
    zi1 = ac + zi1;
    if((numSamples % (R*2)) == 0){  // I-Comb branch [175% CPU@R=4;Fs=62.5k]
      int16_t d1 = zi2 - zd1; // Comb section
      static int16_t v[9];
      v[8] = d1 - zd2;
      zd2 = d1;
      zd1 = zi2;

      for(uint8_t j = 0; j != 8; j++) v[j] = v[j + 1]; // (25%CPU)
      i = v[0];  // Delay to match Hilbert transform on Q branch

      //i = i>>(drive-4);
      //q = q>>(drive-4);

      // post processing I and Q results
      ac = i + qh;
      if((numSamples % (R*2*128)) == 0) rms128 = 0; else rms128 += abs(ac);

      if(mode == AM) { // (12%CPU for the mode selection etc)
        { static int16_t dc;
          dc += (i - dc) / 2;
          i = i - dc; }  // DC decoupling
        { static int16_t dc;
          dc += (q - dc) / 2;
          q = q - dc; }  // DC decoupling
        ac = magn(i, q);  //(25%CPU)
        { static int16_t dc;
          dc += (ac - dc) / 2;
          ac = ac - dc; }  // DC decoupling
      } else if(mode == FM){
        static int16_t z1;
        int16_t z0 = arctan3(q, i);
        ac = z0 - z1;
        z1 = z0;
      }  // needs: p.12 https://www.veron.nl/wp-content/uploads/2014/01/FmDemodulator.pdf
      else { ; }  // USB, LSB, CW
      if(1)//(mode != LSB && mode != USB)
        ac >>= (16-volume);
      else 
        ac = agc(ac);
      if(mode == CW){
        ac = filt_cwn(ac << 4);
//#define CW_DECODER  1
#ifdef CW_DECODER
        char ch = cw(ac >> 0);
        if(ch){
          for(int i=0; i!=15;i++) out[i]=out[i+1];
          out[15] = ch;
          cw_event = true;
        }
#endif
      }
      //static int16_t dc;
      //dc += (ac - dc) / 2;
      //ac = ac - dc;    // DC decoupling

#define DUC  1
#ifdef DUC
      // Output stage
      if(numSamples == 0){ ozd1= 0; ozd2 = 0; ozi1 = 0; ozi2 = 0; } // hack: on first sample init accumlators of further stages (to prevent instability)
      int16_t od1 = ac - ozd1; // Comb section
      ocomb = od1 - ozd2;
      ozd2 = od1;
      ozd1 = ac;
#else
#ifndef PROFILING
      OCR1AL = min(max(ac + ICR1L/2, 0), ICR1L);  // center and clip wrt PWM working range
#endif
#endif
    }
  } 
  // process Q for odd samples
  else {  // [75% CPU@R=4;Fs=62.5k] (excluding the Comb branch and output stage)
    ADMUX = admuxi; //0 | (1 << REFS1) | (1 << REFS0); // prepare next I conversion, 1v1 ref enabled
#ifdef ADC_NR
    interrupts();
    sleep_cpu();
#else
    ADCSRA |= (1 << ADSC);    // start next ADC conversion (trigger ADC interrupt if ADIE flag is set)
#endif
    int16_t adc = (ADCL | (ADCH << 8)) - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH

    static int16_t dc;
    dc += (adc - dc) / 2;
    int16_t ac = adc - dc;    // DC decoupling
  
    static int16_t zi1, zi2, zd1, zd2;
    zi2 = zi1 + zi2;          // Integrator section
    zi1 = ac + zi1;
    if((numSamples % (R*2)) == ((R*2)-1) ){  // Q-Comb branch: [125% CPU@R=4;Fs=62.5k] - executed just 1 sample before executing I-Comb (and final) branch
      //interrupts();
      int16_t d1 = zi2 - zd1; // Comb section
      static int16_t v[16];
      v[15] = d1 - zd2;
      zd2 = d1;
      zd1 = zi2;

      for(uint8_t j = 0; j != 15; j++) v[j] = v[j + 1];
      q = v[7];
      qh = ((v[0] - v[14]) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 15) / 128 + (v[6] - v[8]) / 2; // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)
    }
  }
  
#ifdef DUC
  // Output stage [25% CPU@R=4;Fs=62.5k]
  ozi2 = ozi1 + ozi2;          // Integrator section 
  ozi1 = ocomb + ozi1;
#ifndef PROFILING
  if(volume) OCR1AL = min(max((ozi2>>5) + ICR1L/2, 0), ICR1L);  // center and clip wrt PWM working range
  numSamples++;
#endif
#endif
  //
}


typedef void (*func_t)(void);
static volatile func_t func_ptr = dsp_rx;  //referencing dsp_rx means it is used, and because it is there sdr_rx runs faster!

ISR(TIMER2_COMPA_vect)  // Timer2 COMPA interrupt
{
  func_ptr();
#ifdef PROFILING
  numSamples++;
#endif
}

void adc_start(uint8_t adcpin, bool ref1v1, uint32_t fs)
{
  // Setup ADC
  DIDR0 |= (1 << adcpin); // disable digital input
  
  ADCSRA = 0;             // clear ADCSRA register
  ADCSRB = 0;             // clear ADCSRB register
  ADMUX = 0;              // clear ADMUX register
  ADMUX |= (adcpin & 0x0f);    // set analog input pin
  ADMUX |= ((ref1v1) ? (1 << REFS1) : 0) | (1 << REFS0);  // set AREF=1.1V (Internal ref); otherwise AREF=AVCC=(5V)
  //ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // 128 prescaler for 9.6kHz*1.25
  //ADCSRA |= (1 << ADPS2) | (1 << ADPS1);    // 64 prescaler for 19.2kHz*1.25
  //ADCSRA |= (1 << ADPS2) | (1 << ADPS0);    // 32 prescaler for 38.5kHz*1.25  (this seems to contain more precision and gain compared to 153.8kHz*1.25
  //ADCSRA |= (1 << ADPS2);                   // 16 prescaler for 76.9kHz*1.25
  //ADCSRA |= (1 << ADPS1) | (1 << ADPS0);      // --> ADPS=011: 8 prescaler for 153.8kHz*1.25;  sampling rate is [ADC clock] / [prescaler] / [conversion clock cycles]  for Arduino Uno ADC clock is 20 MHz and a conversion takes 13 clock cycles: ADPS=011: 8 prescaler for 153.8 KHz, ADPS=100: 16 prescaler for 76.9 KHz; ADPS=101: 32 prescaler for 38.5 KHz; ADPS=110: 64 prescaler for 19.2kHz; // ADPS=111: 128 prescaler for 9.6kHz
  ADCSRA |= ((uint8_t)log2((uint8_t)(F_CPU / 13 / fs))) & 0x07;  // ADC Prescaler (for normal conversions non-auto-triggered): ADPS = log2(F_CPU / 13 / Fs) - 1
  //ADCSRA |= (1 << ADIE);  // enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN);  // enable ADC
  //ADCSRA |= (1 << ADSC);  // start ADC measurements
#ifdef ADC_NR
//  set_sleep_mode(SLEEP_MODE_ADC);  // ADC NR sleep destroys the timer2 integrity, therefore Idle sleep is better alternative (keeping clkIO as an active clock domain)
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
#endif
}

void adc_stop()
{
  //ADCSRA &= ~(1 << ADATE); // disable auto trigger
  ADCSRA &= ~(1 << ADIE);  // disable interrupts when measurement complete
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);    // 128 prescaler for 9.6kHz
#ifdef ADC_NR
  sleep_disable();
#endif
  ADMUX = (1 << REFS0);  // restore reference voltage AREF (5V)
}

void timer2_start(uint32_t fs)
{  // Timer 2: interrupt mode
  ASSR &= ~(1 << AS2);  // Timer 2 clocked from CLK I/O (like Timer 0 and 1)
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  TCCR2A |= (1 << WGM21); // WGM21: Mode 2 - CTC (Clear Timer on Compare Match)
  TCCR2B |= (1 << CS22);  // Set C22 bits for 64 prescaler
  TIMSK2 |= (1 << OCIE2A);  // enable timer compare interrupt TIMER2_COMPA_vect
  uint8_t ocr = (((float)F_CPU / (float)64) / (float)fs + 0.5) - 1;   // OCRn = (F_CPU / pre-scaler / fs) - 1;
  OCR2A = ocr;
}

void timer2_stop()
{ // Stop Timer 2 interrupt
  TIMSK2 &= ~(1 << OCIE2A);  // disable timer compare interrupt
  delay(1);  // wait until potential in-flight interrupts are finished
}

void timer1_start(uint32_t fs)
{  // Timer 1: OC1A and OC1B in PWM mode
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1); // Clear OC1A,OC1B on Compare Match when upcounting. Set OC1A,OC1B on Compare Match when downcounting.
  TCCR1B |= ((1 << CS10) | (1 << WGM13)); // WGM13: Mode 8 - PWM, Phase and Frequency Correct;  CS10: clkI/O/1 (No prescaling)
  ICR1H = 0x00;  // TOP. This sets the PWM frequency: PWM_FREQ=312.500kHz ICR=0x1F bit_depth=5; PWM_FREQ=156.250kHz ICR=0x3F bit_depth=6; PWM_FREQ=78.125kHz  ICR=0x7F bit_depth=7; PWM_FREQ=39.250kHz  ICR=0xFF bit_depth=8
  //ICR1L = 0xFF;  // Fpwm = F_CPU / (2 * Prescaler * TOP) :   PWM_FREQ = 39.25kHz, bit-depth=8
  //ICR1L = 160;   // Fpwm = F_CPU / (2 * Prescaler * TOP) :   PWM_FREQ = 62.500kHz, bit-depth=7.8
  //ICR1L = 0x7F;  // Fpwm = F_CPU / (2 * Prescaler * TOP) :   PWM_FREQ = 78.125kHz, bit-depth=7
  ICR1L = (float)F_CPU / (float)2 / (float)fs + 0.5;  // PWM value range (determines bit-depth and PWM frequency):  Fpwm = F_CPU / (2 * Prescaler * TOP)
  OCR1AH = 0x00;
  OCR1AL = 0x00;  // OC1A (SIDETONE) PWM duty-cycle (span defined by ICR).
  OCR1BH = 0x00;
  OCR1BL = 0x00;  // OC1B (KEY_OUT) PWM duty-cycle (span defined by ICR).

#ifdef PROFILING
  // TIMER1_COMPB with interrupt frequency 1.0000128001638422 Hz:
  cli(); // stop interrupts
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; // initialize counter value to 0
  // set compare match register for 1.0000128001638422 Hz increments
  OCR1A = 19530; // = 20000000 / (1024 * 1.0000128001638422) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12, CS11 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (0 << CS11) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); // allow interrupts
}
static uint32_t prev = 0;
ISR(TIMER1_COMPA_vect){
  uint32_t num = numSamples;
  uint32_t diff = num - prev;
  prev = num;
  lcd.setCursor(0, 0); lcd.print( diff ); lcd.print(" SPS   "); 
}
#else
}
#endif

void timer1_stop()
{
  OCR1AL = 0x00;
  OCR1BL = 0x00;
}

volatile int8_t encoder_val = 0;
static uint8_t last_state = 0b11;
ISR(PCINT2_vect){  // Interrupt on rotary encoder turn
  noInterrupts();
  uint8_t curr_state = (digitalRead(ROT_B) << 1) | digitalRead(ROT_A);
  switch((last_state << 2) | curr_state){ //transition  (see: https://www.allaboutcircuits.com/projects/how-to-use-a-rotary-encoder-in-a-mcu-based-project/  )
//#define ENCODER_ENHANCED_RESOLUTION  1
#ifdef ENCODER_ENHANCED_RESOLUTION // Option: enhance encoder from 24 to 96 steps/revolution, see: appendix 1, https://www.sdr-kits.net/documents/PA0KLT_Manual.pdf
    case 0b1101: case 0b0100: case 0b0010: case 0b1011: encoder_val++; break; //encoder_vect(1); break;
    case 0b1110: case 0b1000: case 0b0001: case 0b0111: encoder_val--; break; //encoder_vect(-1); break;
#else
    case 0b1101: encoder_val++; break; //encoder_vect(1); break;
    case 0b1110: encoder_val--; break; //encoder_vect(-1); break;
#endif
  }
  last_state = curr_state;
  interrupts();
}
void encoder_setup()
{
  pinMode(ROT_A, INPUT_PULLUP);
  pinMode(ROT_B, INPUT_PULLUP);
  *digitalPinToPCMSK(ROT_A) |= (1<<digitalPinToPCMSKbit(ROT_A));  // Arduino replacement for PCICR |= (1 << PCIE2); PCMSK2 |= (1 << PCINT22) | (1 << PCINT23); see https://github.com/EnviroDIY/Arduino-SDI-12/wiki/2b.-Overview-of-Interrupts
  *digitalPinToPCMSK(ROT_B) |= (1<<digitalPinToPCMSKbit(ROT_B));
  *digitalPinToPCICR(ROT_A) |= (1<<digitalPinToPCICRbit(ROT_A));
  *digitalPinToPCICR(ROT_B) |= (1<<digitalPinToPCICRbit(ROT_B));
  interrupts();
}

void qcx_setup()
{  
  // initialize
  digitalWrite(SIG_OUT, LOW);
  digitalWrite(RX, HIGH);
  digitalWrite(KEY_OUT, LOW);
  digitalWrite(SIDETONE, LOW);

  // pins
  pinMode(SIDETONE, OUTPUT);
  pinMode(SIG_OUT, OUTPUT);
  pinMode(RX, OUTPUT);
  pinMode(KEY_OUT, OUTPUT);
  pinMode(BUTTONS, INPUT);  // L/R/rotary button
  pinMode(DIT, INPUT_PULLUP);
  pinMode(DAH, INPUT);

  pinMode(AUDIO1, INPUT);
  pinMode(AUDIO2, INPUT);
}

char blanks[] = "        ";
byte font_run[] = {
  0b01000,
  0b00100,
  0b01010,
  0b00101,
  0b01010,
  0b00100,
  0b01000,
  0b00000
};

void customDelay(uint32_t _micros)  //_micros=100000 is 132052us delay
{
  uint32_t i; for(i = 0; i != _micros * 3; i++) wdt_reset();
}

uint32_t sample_amp(uint8_t pin, uint16_t osr)
{
  uint16_t avg = 1024 / 2;
  uint32_t rms = 0;
  uint16_t i;
  for(i = 0; i != 16; i++){
    uint16_t adc = analogRead(pin);
    avg = (avg + adc) / 2;
  }
  for(i = 0; i != osr; i++){ // 128 overampling is 42dB gain => with 10-bit ADC resulting in a total of 102dB DR
    uint16_t adc = analogRead(pin);
    avg = (avg + adc) / 2;  // average
    rms += ((adc > avg) ? 1 : -1) * (adc - avg);  // rectify based
    wdt_reset();
  }
  return rms;
}

float smeter(float ref = 0.0)
{
  float rms;
  if(dsp_cap == ANALOG) rms = ((float)sample_amp(AUDIO1, 128)) * 5.0 / (1024.0 * 128.0 * 100.0 * 120.0 / 1.750); // rmsV = ADC value * AREF / [ADC DR * processing gain * receiver gain * audio gain]
  if(dsp_cap == DSP) rms = (float)rms128 * 5.0 / (1024.0 * 128.0 * 100.0 * 120.0 / 1.750); // rmsV = ADC value * AREF / [ADC DR * processing gain * receiver gain * audio gain]
  if(dsp_cap == SDR) rms = (float)rms128 * 1.1 / (1024.0 * (float)R * 128.0 * 100.0 * 50.0); // rmsV = ADC value * AREF / [ADC DR * processing gain * receiver gain * audio gain]
  float dbm = (10.0 * log10((rms * rms) / 50.0) + 30.0) - ref; //from rmsV to dBm at 50R
  static float dbm_max;
  dbm_max = max(dbm_max, dbm);
  static uint8_t cnt;
  cnt++;
  if((cnt % 8) == 0){
#define DBM_METER  1
#ifdef DBM_METER
    lcd.setCursor(9, 0); lcd.print((int16_t)dbm_max); lcd.print((ref == 0.0) ? "dBm   " : "dB    ");
#else
    uint8_t s = (dbm_max < -63) ? ((dbm_max - -127) / 6) : (uint8_t)(dbm_max - -63 + 10) % 10;  // dBm to S
    lcd.setCursor(14, 0); if(s < 10){ lcd.print("S"); } lcd.print(s); 
#endif
    dbm_max = -174.0 + 34.0;
  }
  return dbm;
}

float dbmeter(float ref = 0.0)
{
  float rms = ((float)sample_amp(AUDIO1, 128)) * 5.0 / (1024.0 * 128.0 * 100.0); // rmsV = ADC value * AREF / [ADC DR * processing gain * receiver gain * audio gain]
  float dbm = (10.0 * log10((rms * rms) / 50.0) + 30.0) - ref; //from rmsV to dBm at 50R
  lcd.setCursor(9, 0); lcd.print(dbm); lcd.print("dB    ");
  delay(300);
  return dbm;
}

void test_tx_amp()
{
  lcd.setCursor(9, 0); lcd.print(blanks);
  TCCR1A = 0;   // Timer 1: PWM mode
  TCCR1B = 0;
  TCCR1A |= (1 << COM1B1); // Clear OC1A/OC1B on Compare Match when upcounting. Set OC1A/OC1B on Compare Match when downcounting.
  TCCR1B |= ((1 << CS10) | (1 << WGM13)); // WGM13: Mode 8 - PWM, Phase and Frequency Correct;  CS10: clkI/O/1 (No prescaling)
  ICR1H = 0x00;  // TOP. This sets the PWM frequency: PWM_FREQ=312.500kHz ICR=0x1freq bit_depth=5; PWM_FREQ=156.250kHz ICR=0x3freq bit_depth=6; PWM_FREQ=78.125kHz  ICR=0x7freq bit_depth=7; PWM_FREQ=39.250kHz  ICR=0xFF bit_depth=8
  ICR1L = 0xFF;  // Fpwm = F_CPU / (2 * Prescaler * TOP) :   PWM_FREQ = 39.25kHz, bit-depth=8
  OCR1BH = 0x00;
  OCR1BL = 0x00;  // PWM duty-cycle (span set by ICR).

  si5351_prev_pll_freq = 0;  // enforce PLL reset
  si5351_freq(freq, 0, 90);  // RX in USB
  si5351_alt_clk2(freq);
  si5351_SendRegister(SI_CLK_OE, 0b11111011); // CLK2_EN=1, CLK1_EN,CLK0_EN=0
  digitalWrite(RX, LOW);  // TX
  uint16_t i;
  for(i = 0; i != 256; i++)
  {
    OCR1BL = lut[i];
    si5351_alt_clk2(freq + i * 10);
    wdt_reset();
    lcd.setCursor(0, 1); lcd.print("SWEEP("); lcd.print(i); lcd.print(")="); lcd.print(lut[i]); lcd.print(blanks);
    delay(200);
  }

  OCR1BL = 0;
  delay(500);
  digitalWrite(RX, HIGH);  // RX
  si5351_SendRegister(SI_CLK_OE, 0b11111100); // CLK2_EN=0, CLK1_EN,CLK0_EN=1
  change = true;  //restore original frequency setting
}

void calibrate_predistortion()
{
  lcd.setCursor(9, 0); lcd.print(blanks);
  TCCR1A = 0;   // Timer 1: PWM mode
  TCCR1B = 0;
  TCCR1A |= (1 << COM1B1); // Clear OC1A/OC1B on Compare Match when upcounting. Set OC1A/OC1B on Compare Match when downcounting.
  TCCR1B |= ((1 << CS10) | (1 << WGM13)); // WGM13: Mode 8 - PWM, Phase and Frequency Correct;  CS10: clkI/O/1 (No prescaling)
  ICR1H = 0x00;  // TOP. This sets the PWM frequency: PWM_FREQ=312.500kHz ICR=0x1freq bit_depth=5; PWM_FREQ=156.250kHz ICR=0x3freq bit_depth=6; PWM_FREQ=78.125kHz  ICR=0x7freq bit_depth=7; PWM_FREQ=39.250kHz  ICR=0xFF bit_depth=8
  ICR1L = 0xFF;  // Fpwm = F_CPU / (2 * Prescaler * TOP) :   PWM_FREQ = 39.25kHz, bit-depth=8
  OCR1BH = 0x00;
  OCR1BL = 0x00;  // PWM duty-cycle (span set by ICR).

  si5351_prev_pll_freq = 0;  //enforce PLL reset
  si5351_freq(freq, 0, 90);  // RX in USB
  si5351_alt_clk2(freq + 1000); // si5351_freq_clk2(freq + 1000);
  si5351_SendRegister(SI_CLK_OE, 0b11111000); // CLK2_EN=1, CLK1_EN,CLK0_EN=1
  digitalWrite(RX, LOW);  // TX
  OCR1BL = 0xFF; // Max power to determine scale
  delay(200);
  float scale = sample_amp(AUDIO2, 128);
  wdt_reset();
  uint8_t amp[256];
  int16_t i, j;
  for(i = 127; i >= 0; i--) // determine amp for pwm value i
  {
    OCR1BL = i;
    wdt_reset();
    delay(100);

    amp[i] = min(255, (float)sample_amp(AUDIO2, 128) * 255.0 / scale);
    lcd.setCursor(0, 1); lcd.print("CALIB("); lcd.print(i); lcd.print(")="); lcd.print(amp[i]); lcd.print(blanks);
  }
  OCR1BL = 0xFF; // Max power to determine scale
  delay(200);
  for(j = 0; j != 256; j++){
    for(i = 0; i != 255 && amp[i] < j; i++) ; //lookup pwm i for amp[i] >= j
    if(j == 255) i = 255; // do not use PWM for peak RF
    lut[j] = i;
  }

  OCR1BL = 0x00;
  digitalWrite(RX, HIGH);  // RX
  si5351_SendRegister(SI_CLK_OE, 0b11111100); // CLK2_EN=0, CLK1_EN,CLK0_EN=1
  change = true;  //restore original frequency setting
}

// RX I/Q calibration procedure: terminate with 50 ohm, enable CW filter, adjust R27, R24, R17 subsequently to its minimum side-band rejection value in dB
void calibrate_iq()
{
  lcd.setCursor(9, 0); lcd.print(blanks);
  digitalWrite(SIG_OUT, true); // loopback on
  si5351_prev_pll_freq = 0;  //enforce PLL reset
  si5351_freq(freq, 0, 90);  // RX in USB
  float dbc;
  si5351_alt_clk2(freq + 700);
  dbc = dbmeter();
  si5351_alt_clk2(freq - 700);
  lcd.setCursor(0, 1); lcd.print("I-Q bal. (700 Hz)"); lcd.print(blanks);
  for(; !digitalRead(BUTTONS);){ wdt_reset(); dbmeter(dbc); } for(; digitalRead(BUTTONS);) wdt_reset();
  si5351_alt_clk2(freq + 600);
  dbc = dbmeter();
  si5351_alt_clk2(freq - 600);
  lcd.setCursor(0, 1); lcd.print("Phase Lo (600 Hz)"); lcd.print(blanks);
  for(; !digitalRead(BUTTONS);){ wdt_reset(); dbmeter(dbc); } for(; digitalRead(BUTTONS);) wdt_reset();
  si5351_alt_clk2(freq + 800);
  dbc = dbmeter();
  si5351_alt_clk2(freq - 800);
  lcd.setCursor(0, 1); lcd.print("Phase Hi (800 Hz)"); lcd.print(blanks);
  for(; !digitalRead(BUTTONS);){ wdt_reset(); dbmeter(dbc); } for(; digitalRead(BUTTONS);) wdt_reset();
/*
  for(uint32_t offset = 0; offset < 3000; offset += 100){
    si5351_alt_clk2(freq + offset);
    dbc = dbmeter();
    si5351_alt_clk2(freq - offset);
    lcd.setCursor(0, 1); lcd.print(offset); lcd.print(" Hz"); lcd.print(blanks);
    wdt_reset(); dbmeter(dbc); delay(500); wdt_reset(); 
  }
*/
  lcd.setCursor(9, 0); lcd.print(blanks);  // cleanup dbmeter
  digitalWrite(SIG_OUT, false); // loopback off
  si5351_SendRegister(SI_CLK_OE, 0b11111100); // CLK2_EN=0, CLK1_EN,CLK0_EN=1
  change = true;  //restore original frequency setting

}

void powermeter()
{
  lcd.setCursor(9, 0); lcd.print(blanks);
  si5351_prev_pll_freq = 0;  //enforce PLL reset
  si5351_freq(freq, 0, 90);  // RX in USB
  si5351_alt_clk2(freq + 1000); // si5351_freq_clk2(freq + 1000);
  si5351_SendRegister(SI_CLK_OE, 0b11111000); // CLK2_EN=1, CLK1_EN,CLK0_EN=1
  digitalWrite(KEY_OUT, HIGH); //OCR1BL = 0xFF;
  digitalWrite(RX, LOW);  // TX
  delay(100);

  float rms = ((float)sample_amp(AUDIO2, 128)) * 5.0 / (1024.0 * 128.0 * 100.0 * 50.0 / 100000.0); // rmsV = ADC value * AREF / [ADC DR * processing gain * receiver gain * rx/tx switch attenuation]
  float dbm = 10.0 * log10((rms * rms) / 50.0) + 30.0; //from rmsV to dBm at 50R

  lcd.setCursor(9, 0); lcd.print(" +"); lcd.print((int16_t)dbm); lcd.print("dBm   ");

  digitalWrite(KEY_OUT, LOW); //OCR1BL = 0x00;
  digitalWrite(RX, HIGH);  // RX
  si5351_SendRegister(SI_CLK_OE, 0b11111100); // CLK2_EN=0, CLK1_EN,CLK0_EN=1
  change = true;  //restore original frequency setting
  delay(1000);
}

void start_rx()
{
//if(!vox_enable) txen(false);
  timer2_stop();
  timer1_stop();
  adc_stop();
  if(dsp_cap){
    tx = 1;
    func_ptr = sdr_rx;  //enable RX DSP/SDR
    numSamples = 0;
    if(dsp_cap == SDR){
      adc_start(0, true, F_ADC_CONV); admuxi = ADMUX;
      adc_start(1, true, F_ADC_CONV); admuxq = ADMUX;
      timer2_start(F_SAMP_RX);
      timer1_start(F_SAMP_RX);
    } else { // ANALOG, DSP
      adc_start(0, false, F_ADC_CONV); admuxi = ADMUX; admuxq = ADMUX;
      timer2_start(F_SAMP_RX);
      timer1_start(F_SAMP_RX);
    }
  }
}

void start_tx()
{
  timer2_stop();
  timer1_stop();
  adc_stop();  //disable RX DSP
  tx = 0;
  func_ptr = dsp_tx;
  numSamples = 0;
  amp = 0;  // initialize
  adc_start(2, true, F_ADC_CONV);
  timer2_start(F_SAMP_TX);
  timer1_start(39250);
  //if(!vox_enable) txen(true);
}

int analogSafeRead(uint8_t pin)
{  // performs classical analogRead with default Arduino sample-rate and analog reference setting; restores previous settings
  noInterrupts();
  uint8_t adcsra = ADCSRA;
  uint8_t admux = ADMUX;
  ADCSRA &= ~(1 << ADIE);  // disable interrupts when measurement complete
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);    // 128 prescaler for 9.6kHz
  ADMUX = (1 << REFS0);  // restore reference voltage AREF (5V)
  delay(1);  // settle
  int val = analogRead(pin);
  ADCSRA = adcsra;
  ADMUX = admux;
  interrupts();
  return val;
}

static uint8_t bandval = 4; //2
#define N_BANDS 14 //7
uint32_t band[] = { 472000, 1840000, 3573000, 5357000, 7074000, 10136000, 14074000, 18100000, 21074000, 24915000, 28074000, 50313000, 70101000, 144125000 };  // { 3573000, 5357000, 7074000, 10136000, 14074000, 18100000, 21074000 };
enum step_t { STEP_10M, STEP_1M, STEP_500k, STEP_100k, STEP_10k, STEP_1k, STEP_500, STEP_100, STEP_10, STEP_1 };
int32_t stepsizes[] = { 10000000, 1000000, 500000, 100000, 10000, 1000, 500, 100, 10, 1 };
volatile int8_t stepsize = STEP_1k;

void process_encoder_tuning_step(int8_t steps)
{
  int32_t stepval = stepsizes[stepsize];
  //if(stepsize < STEP_100) freq %= 1000; // when tuned and stepsize > 100Hz then forget fine-tuning details
  freq += steps * stepval;
  //freq = max(1, min(99999999, freq));
  change = true;
}

void stepsize_showcursor()
{
  lcd.setCursor(stepsize, 1);  // display stepsize with cursor
  lcd.cursor();
}

void stepsize_change(int8_t val)
{
  stepsize += val;
  if(stepsize < STEP_1M) stepsize = STEP_10;
  if(stepsize > STEP_10) stepsize = STEP_1M;
  if(stepsize == STEP_10k || stepsize == STEP_500k) stepsize += val;
  stepsize_showcursor();
}

void(* resetFunc)(void) = 0; // declare reset function @ address 0

void powerDown()
{ // Reduces power from 110mA to 70mA (back-light on) or 30mA (back-light off), remaining current is probably opamp quiescent current
  lcd.setCursor(0, 1); lcd.print("Power-off 73 :-)"); lcd.print(blanks);

  MCUSR = ~(1<<WDRF);  // fix: wdt_disable() bug
  wdt_disable();

  timer2_stop();
  timer1_stop();
  adc_stop();

  si5351_powerDown();

  delay(1500);

  // Disable external interrupts INT0, INT1, Pin Change
  PCICR = 0;
  PCMSK0 = 0;
  PCMSK1 = 0;
  PCMSK2 = 0;
  // Disable internal interrupts
  TIMSK0 = 0;
  TIMSK1 = 0;
  TIMSK2 = 0;
  WDTCSR = 0;
  // Enable BUTTON Pin Change interrupt
  *digitalPinToPCMSK(BUTTONS) |= (1<<digitalPinToPCMSKbit(BUTTONS));
  *digitalPinToPCICR(BUTTONS) |= (1<<digitalPinToPCICRbit(BUTTONS));

  // Power-down sub-systems
  PRR = 0xff;

  lcd.noDisplay();

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_bod_disable();
  interrupts();
  sleep_cpu();  // go to sleep mode, wake-up by either INT0, INT1, Pin Change, TWI Addr Match, WDT, BOD
  sleep_disable();
  resetFunc();
}

/*
volatile boolean WDTalarm = false;
ISR(WDT_vect)
{
  wdt_disable();  // disable watchdog so the system does not restart
  WDTalarm = true;  // flag the event
}
float getTemp()
{
  WDTalarm = false;
  // Set the Watchdog timer                    from: https://www.gammon.com.au/power
  byte interval = 0b000110; // 1s=0b000110,  2s=0b000111, 4s=0b100000, 8s=0b10000
  //64ms= 0b000010, 128ms = 0b000011, 256ms= 0b000100, 512ms= 0b000101
  noInterrupts ();
  MCUSR = 0;
  WDTCSR |= 0b00011000;    // set WDCE, WDE
  WDTCSR = 0b01000000 | interval;    // set WDIE & delay interval
  wdt_reset();  // pat the dog
  interrupts ();
  unsigned long startTime = micros();
  while(!WDTalarm)  {    //sleep while waiting for the WDT
    set_sleep_mode (SLEEP_MODE_IDLE);
    noInterrupts ();  sleep_enable();  interrupts ();  sleep_cpu ();
    sleep_disable();  //processor starts here when any interrupt occurs
  }
  unsigned long WDTmicrosTime = micros() - startTime; // this is your measurement!
  return (float)WDTmicrosTime * 0.0026 - 3668.1;  //calibrate here
}
*/
#define SAFE  1

void setup()
{
  i2c_init();

#ifdef SAFE
  // Benchmark dsp_tx() ISR (this needs to be done in beginning of setup() otherwise when VERSION containts 5 chars, mis-alignment impact performance by a few percent)
  numSamples = 0;
  uint32_t t0, t1;
  func_ptr = dsp_tx;
  t0 = micros();
  TIMER2_COMPA_vect();
  //func_ptr();
  t1 = micros();
  float load_tx = (t1 - t0) * F_SAMP_TX * 100.0 / 1000000.0;
  // benchmark sdr_rx() ISR
  func_ptr = sdr_rx;
  numSamples = 8;
  float load_rx[8];
  float load_rx_avg = 0;
  uint16_t i;
  for(i = 0; i != 8; i++){
    t0 = micros();
    TIMER2_COMPA_vect();
    //func_ptr();
    t1 = micros();
    load_rx[i] = (t1 - t0) * F_SAMP_RX * 100.0 / 1000000.0;
    load_rx_avg += load_rx[i];
  }
  load_rx_avg /= 8;

  //adc_stop();  // recover general ADC settings so that analogRead is working again
  ADMUX = (1 << REFS0);  // restore reference voltage AREF (5V)

  wdt_enable(WDTO_2S);  // Enable watchdog, resolves QCX startup issue
#endif

  // disable external interrupts
  PCICR = 0;
  PCMSK0 = 0;
  PCMSK1 = 0;
  PCMSK2 = 0;

  encoder_setup();

  qcx_setup();

  lcd.begin(16, 2);
  lcd.createChar(1, font_run);

  //PCICR |= (1 << PCIE0);
  //PCMSK0 |= (1 << PCINT5) | (1 << PCINT4) | (1 << PCINT3);
  //interrupts();

  // initialize LUT
  //#define C31_IS_INSTALLED  1   // Uncomment this line when C31 is installed (shaping circuit will be driven with analog signal instead of being switched digitally with PWM signal)
#ifdef C31_IS_INSTALLED // In case of analog driven shaping circuit:
  #define PWM_MIN  29   // The PWM value where the voltage over L4 is approaching its minimum (~0.6V)
  #define PWM_MAX  96   // The PWM value where the voltage over L4 is approaching its maximum (~11V)
#else                   // In case of digital driven shaping circuit:
  #define PWM_MIN  0    // The PWM value where the voltage over L4 is its minimum (0V)
  #define PWM_MAX  255  // The PWM value where the voltage over L4 is its maximum (12V)
#endif
  for(i = 0; i != 256; i++)
    lut[i] = (float)i / ((float)255 / ((float)PWM_MAX - (float)PWM_MIN)) + PWM_MIN;

  // Test if QCX has DSP/SDR capability: SIDETONE output disconnected from AUDIO2
  si5351_SendRegister(SI_CLK_OE, 0b11111111); // Mute QSD: CLK2_EN=CLK1_EN,CLK0_EN=0  
  digitalWrite(RX, HIGH);  // generate pulse on SIDETONE and test if it can be seen on AUDIO2
  delay(100); // settle
  digitalWrite(SIDETONE, LOW);
  int16_t v1 = analogRead(AUDIO2);
  digitalWrite(SIDETONE, HIGH);
  int16_t v2 = analogRead(AUDIO2);
  digitalWrite(SIDETONE, LOW);
  dsp_cap = (v2 - v1) < (0.1 * 1024.0 / 5.0);  // DSP capability?
  
  // Test if QCX has SDR capability: AUDIO2 is disconnected from AUDIO1
  delay(400); wdt_reset(); // settle:  the following test only works well 400ms after startup
  v1 = analogRead(AUDIO1);
  pinMode(AUDIO2, OUTPUT);  // generate pulse on AUDIO2 and test if it can be seen on AUDIO1
  digitalWrite(AUDIO2, HIGH);
  delay(5);
  digitalWrite(AUDIO2, LOW);
  v2 = analogRead(AUDIO1);
  pinMode(AUDIO2, INPUT);
  digitalWrite(AUDIO2, LOW);
  if((dsp_cap) && (v2 - v1) < 8) dsp_cap = SDR;  // SDR capacility?
  // Test if QCX has SSB capability: DVM is biased
  ssb_cap = analogRead(DVM) > 369;

  const char* cap_label[] = { "SSB", "DSP", "SDR" };
  lcd.setCursor(0, 0); lcd.print("QCX"); if(ssb_cap || dsp_cap){ lcd.print("-"); lcd.print(cap_label[dsp_cap]); }
  lcd.setCursor(8, 0); lcd.print("R"); lcd.print(VERSION); lcd.print(blanks);
  //lcd.setCursor(0, 0); lcd.print(String("QCX-") +  (String[]){"SSB", "DSP", "SDR" }[dsp_cap] + F(" R") + F(VERSION) + blanks );

  //lcd.setCursor(0, 1); lcd.print("temp="); lcd.print(getTemp()); lcd.print("C");
  //for(;!digitalRead(BUTTONS);) wdt_reset();

#ifdef SAFE
  // Measure CPU loads
  if(!(load_tx < 100.0))
  {
    lcd.setCursor(0, 1); lcd.print("!!CPU_tx="); lcd.print(load_tx); lcd.print("%"); lcd.print(blanks);
    delay(1500); wdt_reset();
  }
  for(i = 0; i != 8; i++){
    if(!(load_rx[i] < 100.0))
    {
      lcd.setCursor(0, 1); lcd.print("!!CPU_rx"); lcd.print(i); lcd.print("="); lcd.print(load_rx[i]); lcd.print("%"); lcd.print(blanks);
      delay(1500); wdt_reset();
    }
  }
  //if(!(load_rx_avg < 100.0))
  {
    lcd.setCursor(0, 1); lcd.print("!!CPU_rx"); lcd.print("="); lcd.print(load_rx_avg); lcd.print("%"); lcd.print(blanks);
    delay(1500); wdt_reset();
  }

  // Measure VDD (+5V); should be ~5V
  si5351_SendRegister(SI_CLK_OE, 0b11111111); // Mute QSD: CLK2_EN=CLK1_EN,CLK0_EN=0
  digitalWrite(KEY_OUT, LOW);
  digitalWrite(RX, LOW);  // mute RX
  delay(100); // settle
  float vdd = 2.0 * (float)analogRead(AUDIO2) * 5.0 / 1024.0;
  digitalWrite(RX, HIGH);
  if(!(vdd > 4.8 && vdd < 5.2))
  {
    lcd.setCursor(0, 1); lcd.print("!!V5.0="); lcd.print(vdd); lcd.print("V"); lcd.print(blanks);
    delay(1500); wdt_reset();
  }

  // Measure VEE (+3.3V); should be ~3.3V
  float vee = (float)analogRead(SCL) * 5.0 / 1024.0;
  if(!(vee > 3.2 && vee < 3.8))
  {
    lcd.setCursor(0, 1); lcd.print("!!V3.3="); lcd.print(vee); lcd.print("V"); lcd.print(blanks);
    delay(1500); wdt_reset();
  }

  // Measure AVCC via AREF and using internal 1.1V reference fed to ADC; should be ~5V
  analogRead(6); // setup almost proper ADC readout
  bitSet(ADMUX, 3); // Switch to channel 14 (Vbg=1.1V)
  delay(1); // delay improves accuracy
  bitSet(ADCSRA, ADSC);
  for(; bit_is_set(ADCSRA, ADSC););
  float avcc = 1.1 * 1023.0 / ADC;
  if(!(avcc > 4.6 && avcc < 5.1))
  {
    lcd.setCursor(0, 1); lcd.print("!!Vavcc="); lcd.print(avcc); lcd.print("V"); lcd.print(blanks);
    delay(1500); wdt_reset();
  }

  // Measure DVM bias; should be ~VAREF/2
  float dvm = (float)analogRead(DVM) * 5.0 / 1024.0;
  if((ssb_cap) && !(dvm > 1.8 && dvm < 3.2))
  {
    lcd.setCursor(0, 1); lcd.print("!!Vadc2="); lcd.print(dvm); lcd.print("V"); lcd.print(blanks);
    delay(1500); wdt_reset();
  }

  // Measure I2C Bus speed for Bulk Transfers
  si5351_freq(freq, 0, 90);
  wdt_reset();
  t0 = micros();
  for(i = 0; i != 1000; i++) si5351_SendPLLBRegisterBulk();
  t1 = micros();
  uint32_t i2c_speed = (1000000 * 8 * 7) / (t1 - t0); // speed in kbit/s
  if(false)
  {
    lcd.setCursor(0, 1); lcd.print("i2cspeed="); lcd.print(i2c_speed); lcd.print("kbps"); lcd.print(blanks);
    delay(1500); wdt_reset();
  }

  // Measure I2C Bit-Error Rate (BER); should be error free for a thousand random bulk PLLB writes
  si5351_freq(freq, 0, 90);
  wdt_reset();
  uint16_t i2c_error = 0;  // number of I2C byte transfer errors
  for(i = 0; i != 1000; i++){
    si5351_freq_calc_fast(i);
    //for(int j = 3; j != 8; j++) si5351_pll_data[j] = rand();
    si5351_SendPLLBRegisterBulk();
    for(int j = 3; j != 8; j++) if(si5351_RecvRegister(SI_SYNTH_PLL_B + j) != si5351_pll_data[j]) i2c_error++;
  }
  if(i2c_error){
    lcd.setCursor(0, 1); lcd.print("!!BER_i2c="); lcd.print(i2c_error); lcd.print(""); lcd.print(blanks);
    delay(1500); wdt_reset();
  }
#endif

  // Display banner
  //delay(800); wdt_reset();
  lcd.setCursor(7, 0); lcd.print("\001"); lcd.print(blanks);

  volume = (dsp_cap) ? ((dsp_cap == SDR) ? 8 : 14) : 0;
  mode = (dsp_cap || ssb_cap) ? USB : CW;
  start_rx();
}
  
void loop()
{
  delay(100);

  smeter();
  if(mode == CW && cw_event){
    cw_event = false;
    lcd.setCursor(0, 1); lcd.print(out);
  }
  
  if(!digitalRead(DIT)){
    ptt = true;
    start_tx();
    for(; !digitalRead(DIT);){ //until released
      wdt_reset();
    }
    ptt = false;
    delay(1);
    start_rx();
  }
  if(digitalRead(BUTTONS)){ // Left-/Right-/Rotary-button
    uint16_t v = analogSafeRead(BUTTONS);
    enum event_t { BL=0x00, BR=0x10, BE=0x20, SC=0x00, DC=0x01, PL=0x02, PT=0x03 }; // button-left, button-right and button-encoder; single-click, double-click, press-long, press-and-turn
    uint8_t event = SC;
    int32_t t0 = millis();
    for(; digitalRead(BUTTONS);){ // until released or long-press
      if((millis() - t0) > 300){ event = PL; break; }
      wdt_reset();
    }
    delay(10); //debounce
    for(; (event != PL) && ((millis() - t0) < 500);){ // until 2nd press or timeout
      if(digitalRead(BUTTONS)){ event = DC; break; }
      wdt_reset();
    }
    for(; digitalRead(BUTTONS);){ // until released, or encoder is turned while longpress
      if(encoder_val && event == PL){ event = PT; break; }
      wdt_reset();
    }
    event |= (v < 862) ? BL : (v < 1023) ? BR : BE; // determine which button pressed based on threshold levels
    switch(event){
      case BL|SC:
        //calibrate_iq();
        if(dsp_cap){
          att_enable = !att_enable;
          lcd.setCursor(0, 1); lcd.print("Attenuate: "); lcd.print(att_enable ? "ON" : "OFF"); lcd.print(blanks);
          txen(false); // submit attenuator setting
        }
        break;
      case BL|DC: powerDown(); break;
      case BL|PL:
        //calibrate_predistortion();
        //powermeter();
        test_tx_amp();
        break;
      case BL|PT: break;
      case BR|SC:
        mode++;  // mode change
        if(mode != CW) stepsize = STEP_1k; else stepsize = STEP_100;
        if(mode > FM) mode = LSB;
        si5351_prev_pll_freq = 0;  // enforce PLL reset
        change = true;
        break;
      case BR|DC:
        if(drive == 0) drive = 1;
        else drive += 1;
        if(drive > 8) drive = 0;
        lcd.setCursor(0, 1); lcd.print("Drive: "); lcd.print(drive); lcd.print(blanks);
        break;
      case BR|PL:
        vox_enable = true;
        start_tx();
        for(; !digitalRead(BUTTONS);){ // while in VOX mode
          wdt_reset();  // until 2nd press
        }
        vox_enable = false;
        delay(100);
        start_rx();
        delay(100);
        break;
      case BR|PT: break;
      case BE|SC: stepsize_change(+1); break;
      case BE|DC:
        delay(100);
        bandval++;
        if(bandval >= N_BANDS) bandval = 0;
        freq = band[bandval];
        stepsize = STEP_1k;
        change = true;
        break;
      case BE|PL: stepsize_change(-1); break;
      case BE|PT:
          for(; digitalRead(BUTTONS);){ // process encoder changes until released
          wdt_reset();
          if(encoder_val != 0 && (dsp_cap)){
            int16_t tmp = volume;
            tmp += encoder_val;
            encoder_val = 0;
            volume = max(0, min(tmp, 255));
            lcd.setCursor(0, 1); lcd.print("Volume "); lcd.print(volume); lcd.print(blanks);
          }
        }
        change = true; // refresh display
        break;
    }
  }
  if(encoder_val){  // process encoder tuning steps
    process_encoder_tuning_step(encoder_val);
    encoder_val = 0;
  }
  if(change){
    change = false;
    uint32_t n = freq / 1000000;  // lcd.print(f) with commas
    uint32_t n2 = freq % 1000000;
    uint32_t scale = 1000000;
    char buf[16];
    sprintf(buf, "%2u", n); lcd.setCursor(0, 1); lcd.print(buf);
    while(scale != 1){
      scale /= 1000;
      n = n2 / scale;
      n2 = n2  % scale;
      if(scale == 1) sprintf(buf, ",%02u", n / 10); else // leave last digit out
      sprintf(buf, ",%03u", n);
      lcd.print(buf);
    }
    lcd.print(" "); lcd.print(mode_label[mode]); lcd.print("  ");
    lcd.setCursor(15, 1); lcd.print("R");

    if(mode == LSB)
      si5351_freq(freq, 90, 0);  // RX in LSB
    else
      si5351_freq(freq, 0, 90);  // RX in USB
  }
  stepsize_showcursor();
  wdt_reset();
}
