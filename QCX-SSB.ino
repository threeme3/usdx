// Arduino Sketch of the QCX-SSB: SSB with your QCX transceiver (modification)
//
// https://github.com/threeme3/QCX-SSB

#define VERSION   "1.01g"

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
#define AUDIO1  14 //A0
#define AUDIO2  15 //A1
#define DVM     16 //A2
#define BUTTONS 17 //A3
#define LCD_RS  18
#define SDA     18 //shared with LCD_RS
#define SCL     19

#include <LiquidCrystal.h>
class QCXLiquidCrystal : public LiquidCrystal {
public: // QCXLiquidCrystal extends LiquidCrystal library for pull-up driven LCD_RS, as done on QCX. LCD_RS needs to be set to LOW in advance of calling any operation.
  QCXLiquidCrystal(uint8_t rs, uint8_t en, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7) : LiquidCrystal(rs, en, d4, d5, d6, d7){ };
  virtual size_t write(uint8_t value){ // overwrites LiquidCrystal::write() and re-implements LCD data writes
    pinMode(LCD_RS, INPUT);  // pull-up LCD_RS
    write4bits(value >> 4);
    write4bits(value);
    pinMode(LCD_RS, OUTPUT); // pull-down LCD_RS
    return 1;
  };
  void write4bits(uint8_t value){
    digitalWrite(LCD_D4, (value >> 0) & 0x01);
    digitalWrite(LCD_D5, (value >> 1) & 0x01);
    digitalWrite(LCD_D6, (value >> 2) & 0x01);
    digitalWrite(LCD_D7, (value >> 3) & 0x01);
    digitalWrite(LCD_EN, LOW);  // pulseEnable
    delayMicroseconds(1);
    digitalWrite(LCD_EN, HIGH);
    delayMicroseconds(1);    // enable pulse must be >450ns
    digitalWrite(LCD_EN, LOW);
    delayMicroseconds(100);   // commands need > 37us to settle
  };
};
QCXLiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

#include <inttypes.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#undef F_CPU
#define F_CPU 20008440   //20000000   // Actual crystal frequency of XTAL1

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

#define SI_XTAL_FREQ 27003847  //27003847 27004452  Measured crystal frequency of XTAL2 for CL = 10pF (default), calibrate your QCX 27MHz crystal frequency here

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

enum mode_t { LSB, USB, CW };
volatile uint8_t mode = USB;
volatile bool change = true;
volatile int32_t freq = 7074000;

volatile uint8_t tx = 0;
volatile bool vox_enable = false;

inline void vox(bool trigger)
{
  if(trigger){
    if(!tx){
      lcd.setCursor(15, 1); lcd.print("T");
      si5351_SendRegister(SI_CLK_OE, 0b11111011); // CLK2_EN=1, CLK1_EN,CLK0_EN=0
      digitalWrite(RX, LOW);  // TX
    }
    tx = 255; // hangtime = 255 / 4402 = 58ms (the time that TX at least stays on when not triggered again)
  } else {
    if(tx){
      tx--;
      if(!tx){
        digitalWrite(RX, HIGH); // RX
        si5351_SendRegister(SI_CLK_OE, 0b11111100); // CLK2_EN=0, CLK1_EN,CLK0_EN=1
        lcd.setCursor(15, 1); lcd.print("V");
      }
    }
  }
}

volatile uint8_t drive = 4;
//#define F_SAMP 4402
#define F_SAMP 4810        //4810 // ADC sample-rate; is best a multiple of _UA and fits exactly in OCR0A = ((F_CPU / 64) / F_SAMP) - 1 , should not exceed CPU utilization
#define _UA  (F_SAMP)      //360  // unit angle; integer representation of one full circle turn or 2pi radials or 360 degrees, should be a integer divider of F_SAMP and maximized to have higest precision
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

uint8_t lut[256];
volatile uint8_t amp;

inline int16_t ssb(int16_t in)
{
  static int16_t dc;

  int16_t i, q;
  uint8_t j;
  static int16_t v[16];
  for(j = 0; j != 15; j++) v[j] = v[j + 1];

  //dc += (in - dc) / 2;
  v[15] = in - dc;     // DC decoupling
  dc = in;  // this is actually creating a low-pass filter

  i = v[7];
  q = ((v[0] - v[14]) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 15) / 128 + (v[6] - v[8]) / 2; // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)

  uint16_t _amp = abs(i) > abs(q) ? abs(i) + abs(q) / 4 : abs(q) + abs(i) / 4; // approximation of: amp = sqrt(i*i + q*q); error 0.95dB

#define VOX_THRESHOLD (1 << 2)  // 2*6dB above ADC noise level
  if(vox_enable) vox((_amp > VOX_THRESHOLD));

  _amp = _amp << drive;
  _amp = ((_amp > 255) || (drive == 8)) ? 255 : _amp; // clip or when drive=8 use max output
  amp = (tx) ? lut[_amp] : 0;

  static int16_t prev_phase;
  int16_t phase = arctan3(q, i);
  int16_t dp = phase - prev_phase;  // phase difference and restriction
  prev_phase = phase;

  if(dp < 0) dp = dp + _UA; // make negative phase shifts positive: prevents negative frequencies and will reduce spurs on other sideband
#ifdef MAX_DP
  if(dp > MAX_DP){ // dp should be less than half unit-angle in order to keep frequencies below F_SAMP/2
    prev_phase = phase - (dp - MAX_DP);  // substract restdp
    dp = MAX_DP;
  }
#endif
  if(mode == USB)
    return dp * ( F_SAMP / _UA); // calculate frequency-difference based on phase-difference
  else
    return dp * (-F_SAMP / _UA);
}

volatile uint16_t numSamples = 0;
#define MIC_ATTEN  0  // 0*6dB attenuation (note that the LSB bits are quite noisy)

// This is the ADC ISR, issued with sample-rate via timer1 compb interrupt.
// It performs in real-time the ADC sampling, calculation of SSB phase-differences, calculation of SI5351 frequency registers and send the registers to SI5351 over I2C.
void dsp_tx()
{ // jitter dependent things first
  uint8_t low  = ADCL;                 // ADC sample 10-bits analog input, first ADCL, then ADCH
  uint8_t high = ADCH;  
  OCR1BL = amp;                        // submit amplitude to PWM register (actually this is done in advance (about 140us) of phase-change, so that phase-delays in key-shaping circuit filter can settle)
  si5351_SendPLLBRegisterBulk();       // submit frequency registers to SI5351 over 731kbit/s I2C (transfer takes 64/731 = 88us, then PLL-loopfilter probably needs 50us to stabalize)
  //OCR1BL = amp;                        // submit amplitude to PWM register (takes about 1/32125 = 31us+/-31us to propagate) -> amplitude-phase-alignment error is about 30-50us
  int16_t adc = ((high << 8) | low) - 512;
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

inline int16_t agc(int16_t in)
{  // source: Lyons Understanding Digital Signal Processing 3rd edition 13.30
  float out = in * gain;
  #define ref_level 16 /* 32 */  // average reference level (volume)
  #define alpha 0.000001  // time constant
  gain = gain + (ref_level * ref_level - (out * out)) * alpha;
  return out;
}

static char out[] = "                ";
volatile bool cw_event = false;


// This is the ADC ISR, issued with sample-rate via timer1 compb interrupt.
void dsp_rx()
{ // jitter dependent things first
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
    
    if(mode == CW) ac = filt_cwn(ac /* *64 */ * 4 );
    else ac = ac >> drive; //ac >> 3; //ac = agc(ac);
    
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

//#define ORIG  1
typedef void (*func_t)(void);
static volatile func_t func_ptr = dsp_tx;
ISR(ADC_vect)                          // ADC conversion interrupt
{
#ifdef ORIG
  func_ptr();
#endif
}

//#define ADC_NR  1       // Stop CPU at ADC conversion, reduces noise but costs 30% CPU performance when ADC prescaler is 8

#ifdef ORIG
ISR(TIMER2_COMPB_vect)  // Timer2 COMPB interrupt
{
#ifdef ADC_NR
  interrupts();
  sleep_cpu();
#else
  ADCSRA |= (1 << ADSC); // start ADC conversion (triggers ADC interrupt)
#endif
}
#else

#define IQ_MOD  1

ISR(TIMER2_COMPB_vect)  // Timer2 COMPB interrupt
{
  static int16_t i, q;
  #undef  R  // Decimating 2nd Order CIC filter
  #define R 4 //8  // Rate change from 62.5/2 kSPS to 7812.5SPS per chan, providing 12dB gain

  // process I for even samples
  if((numSamples % 2) == 0){  // at 62.5kSPS about 75% CPU load (excluding the Comb branch)
#ifdef IQ_MOD
    ADMUX = 1 | (1 << REFS1) | (1 << REFS0); // prepare next Q conversion, 1v1 ref enabled
#else
    ADMUX = 2 | (1 << REFS0); // prepare next Q conversion (use dummy)
#endif
    ADCSRA |= (1 << ADSC);    // start next ADC conversion (trigger ADC interrupt if ADIE flag is set)
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
    if((numSamples % (R*2)) == 0){  // I-Comb branch: about 7% CPU load
      int16_t d1 = zi2 - zd1; // Comb section
      static int16_t v[16];
      /* i = */ v[15] = d1 - zd2;
      zd2 = d1;
      zd1 = zi2;

      for(uint8_t j = 0; j != 15; j++) v[j] = v[j + 1];
      i = v[7];  // Delay to match Hilbert transform on Q branch

      // post processing I and Q results
      ac = i + q;
      if(mode == CW) ac = filt_cwn(ac /* *64 */ * 4 );
      else ac = ac >> drive; //ac = agc(ac);
      i=min(max(ac, -128), 127);  // clip
      OCR1AL = ac + 128;
    }
  } 
  // process Q for odd samples
  else {
#ifdef IQ_MOD
    ADMUX = 0 | (1 << REFS1) | (1 << REFS0); // prepare next I conversion, 1v1 ref enabled
#else
    ADMUX = 0 | (1 << REFS0); // prepare next I conversion
#endif
    ADCSRA |= (1 << ADSC);    // start next ADC conversion (trigger ADC interrupt if ADIE flag is set)
    int16_t adc = (ADCL | (ADCH << 8)) - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  
    static int16_t dc;
    dc += (adc - dc) / 2;
    int16_t ac = adc - dc;    // DC decoupling
  
    static int16_t zi1, zi2, zd1, zd2;
    zi2 = zi1 + zi2;          // Integrator section
    zi1 = ac + zi1;
    if((numSamples % (R*2)) == ((R*2)-1) ){  // Q-Comb branch: about 17% CPU load - executed just 1 sample before executing I-Comb (and final) branch
      int16_t d1 = zi2 - zd1; // Comb section
      static int16_t v[16];
      /* q = */ v[15] = d1 - zd2;
      zd2 = d1;
      zd1 = zi2;

      for(uint8_t j = 0; j != 15; j++) v[j] = v[j + 1];
      q = ((v[0] - v[14]) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 15) / 128 + (v[6] - v[8]) / 2; // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)
    }
  }
  numSamples++;
}
#endif

void adc_start(uint8_t adcpin, uint8_t ref1v1, uint32_t fs)
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
  ADCSRA |= (1 << ADPS1) | (1 << ADPS0);      // ADPS=011: 8 prescaler for 153.8kHz*1.25;  sampling rate is [ADC clock] / [prescaler] / [conversion clock cycles]  for Arduino Uno ADC clock is 20 MHz and a conversion takes 13 clock cycles: ADPS=011: 8 prescaler for 153.8 KHz, ADPS=100: 16 prescaler for 76.9 KHz; ADPS=101: 32 prescaler for 38.5 KHz; ADPS=110: 64 prescaler for 19.2kHz; // ADPS=111: 128 prescaler for 9.6kHz
#ifdef ORIG
  ADCSRA |= (1 << ADIE);  // enable interrupts when measurement complete
#else
#ifdef ADC_NR
  ADCSRA |= (1 << ADIE);  // enable interrupts when measurement complete
#endif
#endif
  ADCSRA |= (1 << ADEN);  // enable ADC
  //ADCSRA |= (1 << ADSC);  // start ADC measurements
#ifdef ADC_NR
//  set_sleep_mode(SLEEP_MODE_ADC);  // ADC NR sleep destroys the timer2 integrity, therefore Idle sleep is better alternative (keeping clkIO as an active clock domain)
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
#endif

  // Timer 2: interrupt mode
  ASSR &= ~(1 << AS2);  // Timer 2 clocked from CLK I/O (like Timer 0 and 1)
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  TCCR2A |= (1 << WGM01); // WGM01: Mode 2 - CTC
  TCCR2B |= (1 << CS22);  // Set C22 bits for 64 prescaler
  TIMSK2 |= (1 << OCIE2B);  // enable timer compare interrupt TIMER2_COMPB_vect
  uint8_t ocr = (((float)F_CPU / (float)64) / (float)fs + 0.5) - 1;   // ((F_CPU / 64) / F_SAMP) - 1;
  OCR2A = ocr; // Set the value that you want to count to :   OCRn = [clock_speed / (Prescaler_value * Fs)] - 1  : sampling frequency

  // Timer 1: OC1A and OC1B in PWM mode
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1); // Clear OC1A,OC1B on Compare Match when upcounting. Set OC1A,OC1B on Compare Match when downcounting.
  TCCR1B |= ((1 << CS10) | (1 << WGM13)); // WGM13: Mode 8 - PWM, Phase and Frequency Correct;  CS10: clkI/O/1 (No prescaling)
  ICR1H = 0x00;  // TOP. This sets the PWM frequency: PWM_FREQ=312.500kHz ICR=0x1F bit_depth=5; PWM_FREQ=156.250kHz ICR=0x3F bit_depth=6; PWM_FREQ=78.125kHz  ICR=0x7F bit_depth=7; PWM_FREQ=39.250kHz  ICR=0xFF bit_depth=8
  ICR1L = 0xFF;  // Fpwm = F_CPU / (2 * Prescaler * TOP) :   PWM_FREQ = 39.25kHz, bit-depth=8
  OCR1AH = 0x00;
  OCR1AL = 0x00;  // OC1A (SIDETONE) PWM duty-cycle (span defined by ICR).
  OCR1BH = 0x00;
  OCR1BL = 0x00;  // OC1B (KEY_OUT) PWM duty-cycle (span defined by ICR).

  amp = 0;
}

//#define F_SAMP_RX 8000
#define F_SAMP_RX 62500

void adc_stop()
{
  // Stop Timer 2 interrupt
  TIMSK2 &= ~(1 << OCIE2B);  // disable timer compare interrupt TIMER2_COMPB_vect

  OCR1AL = 0x00;
  OCR1BL = 0x00;

  //ADCSRA &= ~(1 << ADATE); // disable auto trigger
  ADCSRA &= ~(1 << ADIE);  // disable interrupts when measurement complete
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);    // 128 prescaler for 9.6kHz

#ifdef ADC_NR
  sleep_disable();
#endif

  ADMUX = 0;              // clear ADMUX register
  ADMUX |= (1 << REFS0);  // restore reference voltage AREF (5V)
}

static uint8_t bandval = 4; //2
#define N_BANDS 14 //7
uint32_t band[] = { 472000, 1840000, 3573000, 5357000, 7074000, 10136000, 14074000, 18100000, 21074000, 24915000, 28074000, 50313000, 70101000, 144125000 };  // { 3573000, 5357000, 7074000, 10136000, 14074000, 18100000, 21074000 };
enum step_t { STEP_10M, STEP_1M, STEP_500k, STEP_100k, STEP_10k, STEP_1k, STEP_500, STEP_100, STEP_10, STEP_1 };
volatile int8_t stepsize = STEP_1k;

void encoder_vect(int8_t sign)
{
  int32_t stepval;
  switch(stepsize){
    case STEP_10M:  stepval = 10000000; break;
    case STEP_1M:   stepval = 1000000; break;
    case STEP_500k: stepval = 500000; break;
    case STEP_100k: stepval = 100000; break;
    case STEP_10k:  stepval = 10000; break;
    case STEP_1k:   stepval = 1000; break;
    case STEP_500:  stepval = 500; break;
    case STEP_100:  stepval = 100; break;
    case STEP_10:   stepval = 10; break;
    case STEP_1:    stepval = 1; break;
  }
  if(stepval < STEP_100) freq %= 1000; // when tuned and stepsize > 100Hz then forget fine-tuning details
  freq += sign * stepval;
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

ISR(PCINT2_vect){  // Interrupt on rotary encoder turn
  noInterrupts();
  static uint8_t last_state;
  uint8_t curr_state = (digitalRead(ROT_B) << 1) | digitalRead(ROT_A);
  switch((last_state << 2) | curr_state){ //transition
#ifdef ENCODER_ENHANCED_RESOLUTION // Option: enhance encoder from 24 to 96 steps/revolution, see: https://www.sdr-kits.net/documents/PA0KLT_Manual.pdf
    case 0b1101: case 0b0100: case 0b0010: case 0b1011: encoder_vect(1); break;
    case 0b1110: case 0b1000: case 0b0001: case 0b0111: encoder_vect(-1); break;
#else
    case 0b1101: encoder_vect(1); break;
    case 0b1110: encoder_vect(-1); break;
#endif
  }
  last_state = curr_state;
  interrupts();
}
void encoder_setup()
{
  pinMode(ROT_A, INPUT_PULLUP);
  pinMode(ROT_B, INPUT_PULLUP);
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT22) | (1 << PCINT23);
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
  float rms = ((float)sample_amp(AUDIO1, 128)) * 5.0 / (1024.0 * 128.0 * 100.0 * 120.0 / 1.750); // rmsV = ADC value * AREF / [ADC DR * processing gain * receiver gain * audio gain]
  float dbm = (10.0 * log10((rms * rms) / 50.0) + 30.0) - ref; //from rmsV to dBm at 50R
  static float dbm_max;
  dbm_max = max(dbm_max, dbm);
  static uint8_t cnt;
  cnt++;
  if((cnt % 8) == 0){
//#define DBM_METER  1
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

void toggle_rxdsp()
{
  if(func_ptr != dsp_rx){
    func_ptr = dsp_rx;  //enable RX DSP
    adc_start(0, false, F_SAMP_RX);
  } else {
    adc_stop();  //disable RX DSP
    func_ptr = dsp_tx;
  }
  change = true;
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
#ifdef SAFE
  // Benchmark ADC_vect() ISR (this needs to be done in beginning of setup() otherwise when VERSION containts 5 chars, mis-alignment impact performance by a few percent)
  numSamples = 0;
  uint32_t t0, t1;
  t0 = micros();
  TIMER2_COMPB_vect();
  ADC_vect();
  t1 = micros();
  float load = (t1 - t0) * F_SAMP * 100.0 / 1000000.0;

  wdt_enable(WDTO_2S);  // Enable watchdog, resolves QCX startup issue
#endif

  lcd.begin(16, 2);
  lcd.createChar(1, font_run);
  lcd.setCursor(0, 0); lcd.print("QCX-SSB R"); lcd.print(VERSION); lcd.print(blanks);
  delay(200);

  //lcd.setCursor(0, 1); lcd.print("temp="); lcd.print(getTemp()); lcd.print("C");
  //for(;!digitalRead(BUTTONS);) wdt_reset();

  i2c_init();

  encoder_setup();

  qcx_setup();

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
  uint16_t i;
  for(i = 0; i != 256; i++)
    lut[i] = (float)i / ((float)255 / ((float)PWM_MAX - (float)PWM_MIN)) + PWM_MIN;

#ifdef SAFE
  // Measure CPU loads
  if(!(load < 100.0)){
    lcd.setCursor(0, 1); lcd.print("E01 CPU overload");
    delay(1500);
    wdt_reset();
  }

  // Measure VDD (+5V); should be ~5V
  digitalWrite(RX, LOW);  // mute RX
  digitalWrite(KEY_OUT, LOW);
  si5351_SendRegister(SI_CLK_OE, 0b11111111); // Mute QSD: CLK2_EN=CLK1_EN,CLK0_EN=0
  float vdd = 2.0 * (float)analogRead(AUDIO2) * 5.0 / 1024.0;
  digitalWrite(RX, HIGH);
  if(!(vdd > 4.5 && vdd < 5.2)){
    lcd.setCursor(0, 1); lcd.print("E02 +5V not OK");
    delay(1500);
    wdt_reset();
  }

  // Measure VEE (+3.3V); should be ~3.3V
  float vee = (float)analogRead(SCL) * 5.0 / 1024.0;
  if(!(vee > 3.2 && vee < 3.8)){
    lcd.setCursor(0, 1); lcd.print("E03 +3.3V not OK");
    delay(1500);
    wdt_reset();
  }

  // Measure AVCC via AREF and using internal 1.1V reference fed to ADC; should be ~5V
  analogRead(6); // setup almost proper ADC readout
  bitSet(ADMUX, 3); // Switch to channel 14 (Vbg=1.1V)
  delay(1); // delay improves accuracy
  bitSet(ADCSRA, ADSC);
  for(; bit_is_set(ADCSRA, ADSC););
  float avcc = 1.1 * 1023.0 / ADC;
  if(!(avcc > 4.6 && avcc < 5.1)){
    lcd.setCursor(0, 1); lcd.print("E04 AVCC not OK");
    delay(1500);
    wdt_reset();
  }

  // Measure DVM bias; should be ~VAREF/2
  float dvm = (float)analogRead(DVM) * 5.0 / 1024.0;
  if(!(dvm > 1.8 && dvm < 3.2)){
    lcd.setCursor(0, 1); lcd.print("E05 DVM bias err");
    delay(1500);
    wdt_reset();
  }

  // Measure I2C Bus speed for Bulk Transfers
  t0 = micros();
  for(i = 0; i != 1000; i++) si5351_SendPLLBRegisterBulk();
  t1 = micros();
  uint32_t i2c_speed = (1000000 * 8 * 7) / (t1 - t0); // speed in kbit/s

  // Measure I2C Bit-Error Rate (BER); should be error free for a thousand random bulk PLLB writes
  uint16_t i2c_error = 0;  // number of I2C byte transfer errors
  for(i = 0; i != 1000; i++){
    for(int j = 3; j != 8; j++) si5351_pll_data[j] = rand();
    si5351_SendPLLBRegisterBulk();
    for(int j = 3; j != 8; j++) if(si5351_RecvRegister(SI_SYNTH_PLL_B + j) != si5351_pll_data[j]) i2c_error++;
  }
  if(i2c_error){
    lcd.setCursor(0, 1); lcd.print("E06 I2C tx error");
    delay(1500);
    wdt_reset();
  }

  lcd.setCursor(0, 1); lcd.print("CPU_tx="); lcd.print(load); lcd.print("%"); lcd.print(blanks);
#endif
  delay(800);
  lcd.setCursor(7, 0); lcd.print("\001"); lcd.print(blanks); // Ready: display initialization complete normally
}

void loop()
{
  delay(100);

  if(func_ptr != dsp_rx) smeter();
  if(mode == CW && cw_event){
    cw_event = false;
    lcd.setCursor(0, 1); lcd.print(out);
  }
  
  if(!digitalRead(DIT)){
    lcd.setCursor(15, 1); lcd.print("T");
    lcd.setCursor(9, 0); lcd.print(blanks);
    si5351_SendRegister(SI_CLK_OE, 0b11111011); // CLK2_EN=1, CLK1_EN,CLK0_EN=0
    adc_start(2, true, F_SAMP);
    customDelay(1000);  //allow setup time
    digitalWrite(RX, LOW);  // TX
    tx = 1;
    for(; !digitalRead(DIT);){ //until released
      wdt_reset();
    }
    digitalWrite(RX, HIGH); // RX
    customDelay(1000);  //allow setup time
    tx = 0;
    adc_stop();
    si5351_SendRegister(SI_CLK_OE, 0b11111100); // CLK2_EN=0, CLK1_EN,CLK0_EN=1
    lcd.setCursor(15, 1); lcd.print("R");
  }
  if(digitalRead(BUTTONS)){ // Left-/Right-/Rotary-button
    uint16_t val;
    if(func_ptr != dsp_rx)  //hack to prevent corrupting RX DSP ADC settings
      val = analogRead(BUTTONS);
    else { toggle_rxdsp(); return; }  //hack
    bool longpress = false;
    bool doubleclick = false;
    int32_t t0 = millis();
    for(; digitalRead(BUTTONS) && !longpress;){ // until released or long-press
      longpress = ((millis() - t0) > 300);
      wdt_reset();
    }
    delay(10); //debounce
    for(; !longpress && ((millis() - t0) < 500) && !doubleclick;){ //until 2nd press or timeout
      doubleclick = digitalRead(BUTTONS);
      wdt_reset();
    }
    for(; digitalRead(BUTTONS);) wdt_reset(); // until released
    if(val < 862){       // LEFT-button   ADC=780
      if(doubleclick){
        calibrate_predistortion();
        return;
      }
      if(longpress){
        test_tx_amp();
        //powermeter();
        return;
      } //single-click
      //calibrate_iq();
      toggle_rxdsp();  // enable/disable digital RX
    } else if(val < 1023){ // RIGHT-button  ADC=943
        if(doubleclick){
        if(drive == 0) drive = 1;
        else drive += 1;
        if(drive > 8) drive = 0;
        lcd.setCursor(0, 1); lcd.print("Drive "); lcd.print(drive); lcd.print(blanks);
        return;
      }
      if(longpress){
        lcd.setCursor(15, 1); lcd.print("V");
        lcd.setCursor(9, 0); lcd.print(blanks);
        vox_enable = true;
        adc_start(2, true, F_SAMP);
        for(; !digitalRead(BUTTONS);){ // while in VOX mode
          wdt_reset();  // until 2nd press
        }
        adc_stop();
        vox_enable = false;
        lcd.setCursor(15, 1); lcd.print("R");
        for(; digitalRead(BUTTONS);) wdt_reset(); // until released
        delay(100);
        return;
      } //single-click
      mode++;  // mode change
      if(mode != CW) stepsize = STEP_1k; else stepsize = STEP_100;
      if(mode > CW) mode = LSB;
      si5351_prev_pll_freq = 0;  // enforce PLL reset
      change = true;
    } else {               // ROTARY-button ADC=1023
      if(doubleclick){
        delay(100);
        bandval++;
        if(bandval >= N_BANDS) bandval = 0;
        freq = band[bandval];
        stepsize = STEP_1k;
        change = true;
        return;
      }
      if(longpress){
        stepsize_change(-1);
        return;
      } //single-click
      stepsize_change(+1);
    }
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
    lcd.print(" ");
    if(mode == LSB) lcd.print("LSB");
    if(mode == USB) lcd.print("USB");
    if(mode ==  CW) lcd.print("CW ");
    lcd.print(" ");
    if(func_ptr != dsp_rx){ lcd.setCursor(15, 1); lcd.print("R"); }
    if(func_ptr == dsp_rx){ lcd.setCursor(15, 1); lcd.print("D"); }

    if(mode == LSB)
      si5351_freq(freq, 90, 0);  // RX in LSB
    else
      si5351_freq(freq, 0, 90);  // RX in USB
  }
  wdt_reset();
  stepsize_showcursor();
}
