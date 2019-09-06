// Arduino Sketch of the QCX-SSB: SfSB (+ SDR) with your QCX transceiver
//
// https://github.com/threeme3/QCX-SSB

#define VERSION   "1.01i"

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

#include <inttypes.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#undef F_CPU
#define F_CPU 20008440   // myqcx1:20008440   // Actual crystal frequency of XTAL1 20000000
#define log2(n) (log(n) / log(2))

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
  }
  void write4bits(uint8_t value){
    digitalWrite(LCD_D4, (value >> 0) & 0x01);
    digitalWrite(LCD_D5, (value >> 1) & 0x01);
    digitalWrite(LCD_D6, (value >> 2) & 0x01);
    digitalWrite(LCD_D7, (value >> 3) & 0x01);
    digitalWrite(LCD_EN, LOW);  //delayMicroseconds(1);  // pulseEnable
    digitalWrite(LCD_EN, HIGH); //delayMicroseconds(1);  // enable pulse must be >450ns
    digitalWrite(LCD_EN, LOW);  //delayMicroseconds(50); // commands need > 37us to settle
  }
};
QCXLiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

class I2C {
public:
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

  I2C(){
    I2C_PORT &= ~( I2C_SDA | I2C_SCL );
    I2C_SCL_HI();
    I2C_SDA_HI();
    suspend();
  }
  ~I2C(){
    I2C_PORT &= ~( I2C_SDA | I2C_SCL );
    I2C_DDR &= ~( I2C_SDA | I2C_SCL );
  }  
  inline void start(){
    resume();  //prepare for I2C
    I2C_SCL_LO();
    I2C_SDA_HI();
  }
  inline void stop(){
    I2C_SCL_HI();
    I2C_SDA_HI();
    I2C_DDR &= ~(I2C_SDA | I2C_SCL); // prepare for a start: pull-up both SDA, SCL
    suspend();
  }
  #define SendBit(data, mask) \
    if(data & mask){ \
      I2C_SDA_HI();  \
    } else {         \
      I2C_SDA_LO();  \
    }                \
    I2C_SCL_HI();    \
    I2C_SCL_LO();
  inline void SendByte(uint8_t data){
    SendBit(data, 1 << 7);
    SendBit(data, 1 << 6);
    SendBit(data, 1 << 5);
    SendBit(data, 1 << 4);
    SendBit(data, 1 << 3);
    SendBit(data, 1 << 2);
    SendBit(data, 1 << 1);
    SendBit(data, 1 << 0);
    I2C_SDA_HI();  // recv ACK
    DELAY(I2C_DELAY);
    I2C_SCL_HI();
    I2C_SCL_LO();
  }
  inline uint8_t RecvBit(uint8_t mask){
    I2C_SCL_HI();
    uint16_t i = 60000;
    for(;(!I2C_SCL_GET()) && i; i--);  // wait util slave release SCL to HIGH (meaning data valid), or timeout at 3ms
    if(!i){ lcd.setCursor(0, 1); lcd.print("E07 I2C timeout"); }
    uint8_t data = I2C_SDA_GET();
    I2C_SCL_LO();
    return (data) ? mask : 0;
  }
  inline uint8_t RecvByte(uint8_t last){
    uint8_t data = 0;
    data |= RecvBit(1 << 7);
    data |= RecvBit(1 << 6);
    data |= RecvBit(1 << 5);
    data |= RecvBit(1 << 4);
    data |= RecvBit(1 << 3);
    data |= RecvBit(1 << 2);
    data |= RecvBit(1 << 1);
    data |= RecvBit(1 << 0);
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
  inline void resume(){
  #ifdef LCD_RS_PORTIO
    I2C_PORT &= ~I2C_SDA; // pin sharing SDA/LCD_RS mitigation
  #endif
  }
  inline void suspend(){
    I2C_SDA_LO();         // pin sharing SDA/LCD_RS: pull-down LCD_RS; QCXLiquidCrystal require this for any operation
  }
};

class SI5351 : public I2C {
public:
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

  volatile uint8_t prev_divider;
  volatile int32_t raw_freq;
  volatile uint8_t divider;  // note: because of int8 only freq > 3.6MHz can be covered for R_DIV=1
  volatile uint8_t mult;
  volatile uint8_t pll_data[8];
  volatile int32_t prev_pll_freq;
  volatile int32_t pll_freq;   // temporary
  
  SI5351(){
    init();
    prev_pll_freq = 0;
  }
  uint8_t RecvRegister(uint8_t reg)
  {
    // Data write to set the register address
    start();
    SendByte(SI_I2C_ADDR << 1);
    SendByte(reg);
    stop();
    // Data read to retrieve the data from the set address
    start();
    SendByte((SI_I2C_ADDR << 1) | 1);
    uint8_t data = RecvByte(true);
    stop();
    return data;
  }
  void SendRegister(uint8_t reg, uint8_t data)
  {
    start();
    SendByte(SI_I2C_ADDR << 1);
    SendByte(reg);
    SendByte(data);
    stop();
  }
  inline void SendPLLBRegisterBulk()  // fast freq change of PLLB, takes about [ 2 + 7*(8+1) + 2 ] / 840000 = 80 uS
  {
    start();
    SendByte(SI_I2C_ADDR << 1);
    SendByte(SI_SYNTH_PLL_B + 3);  // Skip the first three pll_data bytes (first two always 0xFF and third not often changing
    SendByte(pll_data[3]);
    SendByte(pll_data[4]);
    SendByte(pll_data[5]);
    SendByte(pll_data[6]);
    SendByte(pll_data[7]);
    stop();
  }
  // Set up MultiSynth for register reg=MSNA, MNSB, MS0-5 with fractional divider, num and denom and R divider (for MSn, not for MSNA, MSNB)
  // divider is 15..90 for MSNA, MSNB,  divider is 8..900 (and in addition 4,6 for integer mode) for MS[0-5]
  // num is 0..1,048,575 (0xFFFFF)
  // denom is 0..1,048,575 (0xFFFFF)
  // num = 0, denom = 1 forces an integer value for the divider
  // r_div = 1..128 (1,2,4,8,16,32,64,128)
  void SetupMultisynth(uint8_t reg, uint8_t divider, uint32_t num, uint32_t denom, uint8_t r_div)
  {
    uint32_t P1; // Synth config register P1
    uint32_t P2; // Synth config register P2
    uint32_t P3; // Synth config register P3
  
    P1 = (uint32_t)(128 * ((float)num / (float)denom));
    P1 = (uint32_t)(128 * (uint32_t)(divider) + P1 - 512);
    P2 = (uint32_t)(128 * ((float)num / (float)denom));
    P2 = (uint32_t)(128 * num - denom * P2);
    P3 = denom;
  
    SendRegister(reg + 0, (P3 & 0x0000FF00) >> 8);
    SendRegister(reg + 1, (P3 & 0x000000FF));
    SendRegister(reg + 2, (P1 & 0x00030000) >> 16 | ((int)log2(r_div) << 4) );
    SendRegister(reg + 3, (P1 & 0x0000FF00) >> 8);
    SendRegister(reg + 4, (P1 & 0x000000FF));
    SendRegister(reg + 5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
    SendRegister(reg + 6, (P2 & 0x0000FF00) >> 8);
    SendRegister(reg + 7, (P2 & 0x000000FF));
  }
  // this function relies on cached (global) variables: divider, mult, raw_freq, pll_data
  inline void freq_calc_fast(int16_t freq_offset)
  { // freq_offset is relative to freq set in freq(freq)
    // uint32_t num128 = ((divider * (raw_freq + offset)) % SI_XTAL_FREQ) * (float)(0xFFFFF * 128) / SI_XTAL_FREQ;
    // Above definition (for SI_XTAL_FREQ=27.00491M) can be optimized by pre-calculating factor (0xFFFFF*128)/SI_XTAL_FREQ (=4.97) as integer constant (5) and
    // substracting the rest error factor (0.03). Note that the latter is shifted left (0.03<<6)=2, while the other term is shifted right (>>6)
    register int32_t z = ((divider * (raw_freq + freq_offset)) % SI_XTAL_FREQ);
    register int32_t z2 = -(z >> 5);
    int32_t num128 = (z * 5) + z2;
  
    // Set up specified PLL with mult, num and denom: mult is 15..90, num128 is 0..128*1,048,575 (128*0xFFFFF), denom is 0..1,048,575 (0xFFFFF)
    uint32_t P1 = 128 * mult + (num128 / 0xFFFFF) - 512;
    uint32_t P2 = num128 % 0xFFFFF;
    //pll_data[0] = 0xFF;
    //pll_data[1] = 0xFF;
    //pll_data[2] = (P1 >> 14) & 0x0C;
    pll_data[3] = P1 >> 8;
    pll_data[4] = P1;
    pll_data[5] = 0xF0 | (P2 >> 16);
    pll_data[6] = P2 >> 8;
    pll_data[7] = P2;
  }
  uint16_t div(uint32_t num, uint32_t denom, uint32_t* b, uint32_t* c)
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
  void freq(uint32_t freq, uint8_t i, uint8_t q)
  { // Fout = Fvco / (R * [MSx_a + MSx_b/MSx_c]),  Fvco = Fxtal * [MSPLLx_a + MSPLLx_b/MSPLLx_c]; MSx as integer reduce spur
    uint8_t r_div = (freq > 4000000) ? 1 : (freq > 400000) ? 32 : 128; // helps divider to be in range
    freq *= r_div;  // take r_div into account, now freq is in the range 1MHz to 150MHz
    raw_freq = freq;   // cache frequency generated by PLL and MS stages (excluding R divider stage); used by freq_calc_fast()
  
    divider = 900000000 / freq;  // Calculate the division ratio. 900,000,000 is the maximum internal PLL freq (official range 600..900MHz but can be pushed to 300MHz..~1200Mhz)
    if(divider % 2) divider--;  // divider in range 8.. 900 (including 4,6 for integer mode), even numbers preferred. Note that uint8 datatype is used, so 254 is upper limit
    if( (divider * (freq - 5000) / SI_XTAL_FREQ) != (divider * (freq + 5000) / SI_XTAL_FREQ) ) divider -= 2; // Test if multiplier remains same for freq deviation +/- 5kHz, if not use different divider to make same
    /*int32_t*/ pll_freq = divider * freq; // Calculate the pll_freq: the divider * desired output freq
    uint32_t num, denom;
    mult = div(pll_freq, SI_XTAL_FREQ, &num, &denom); // Determine the mult to get to the required pll_freq (in the range 15..90)
  
    // Set up specified PLL with mult, num and denom: mult is 15..90, num is 0..1,048,575 (0xFFFFF), denom is 0..1,048,575 (0xFFFFF)
    // Set up PLL A and PLL B with the calculated  multiplication ratio
    SetupMultisynth(SI_SYNTH_PLL_A, mult, num, denom, 1);
    SetupMultisynth(SI_SYNTH_PLL_B, mult, num, denom, 1);
    //if(denom == 1) SendRegister(22, SI_MSx_INT); // FBA_INT: MSNA operates in integer mode
    //if(denom == 1) SendRegister(23, SI_MSx_INT); // FBB_INT: MSNB operates in integer mode
    // Set up MultiSynth 0,1,2 with the calculated divider, from 4, 6..1800.
    // The final R division stage can divide by a power of two, from 1..128
    // if you want to output frequencies below 1MHz, you have to use the final R division stage
    SetupMultisynth(SI_SYNTH_MS_0, divider, 0, 1, r_div);
    SetupMultisynth(SI_SYNTH_MS_1, divider, 0, 1, r_div);
    SetupMultisynth(SI_SYNTH_MS_2, divider, 0, 1, r_div);
    //if(prev_divider != divider){ lcd.setCursor(0, 0); lcd.print(divider); lcd.print(blanks); }
    // Set I/Q phase
    SendRegister(SI_CLK0_PHOFF, i * divider / 90); // one LSB equivalent to a time delay of Tvco/4 range 0..127
    SendRegister(SI_CLK1_PHOFF, q * divider / 90); // one LSB equivalent to a time delay of Tvco/4 range 0..127
    // Switch on the CLK0, CLK1 output to be PLL A and set multiSynth0, multiSynth1 input (0x0F = SI_CLK_SRC_MS | SI_CLK_IDRV_8MA)
    SendRegister(SI_CLK0_CONTROL, 0x0F | SI_MS_INT | SI_CLK_SRC_PLL_A);
    SendRegister(SI_CLK1_CONTROL, 0x0F | SI_MS_INT | SI_CLK_SRC_PLL_A);
    // Switch on the CLK2 output to be PLL B and set multiSynth2 input
    SendRegister(SI_CLK2_CONTROL, 0x0F | SI_MS_INT | SI_CLK_SRC_PLL_B);
    SendRegister(SI_CLK_OE, 0b11111100); // Enable CLK1|CLK0
    // Reset the PLL. This causes a glitch in the output. For small changes to
    // the parameters, you don't need to reset the PLL, and there is no glitch
    if((abs(pll_freq - prev_pll_freq) > 16000000L) || divider != prev_divider){
      prev_pll_freq = pll_freq;
      prev_divider = divider;
      SendRegister(SI_PLL_RESET, 0xA0);
    }
  }
  void alt_clk2(uint32_t freq)
  {
    uint32_t num, denom;
    uint16_t mult = div(pll_freq, freq, &num, &denom);
  
    SetupMultisynth(SI_SYNTH_MS_2, mult, num, denom, 1);
  
    // Switch on the CLK2 output to be PLL A and set multiSynth2 input
    SendRegister(SI_CLK2_CONTROL, 0x0F | SI_CLK_SRC_PLL_A);
  
    SendRegister(SI_CLK_OE, 0b11111000); // Enable CLK2|CLK1|CLK0
  
    //SendRegister(SI_CLK0_PHOFF, 0 * divider / 90); // one LSB equivalent to a time delay of Tvco/4 range 0..127
    //SendRegister(SI_CLK1_PHOFF, 90 * divider / 90); // one LSB equivalent to a time delay of Tvco/4 range 0..127
    //SendRegister(SI_CLK2_PHOFF, 45 * divider / 90); // one LSB equivalent to a time delay of Tvco/4 range 0..127
    //SendRegister(SI_PLL_RESET, 0xA0);
  }
  void powerDown()
  {
    SendRegister(SI_CLK0_CONTROL, 0b11000000);  // Conserve power when output is dsiabled
    SendRegister(SI_CLK1_CONTROL, 0b11000000);
    SendRegister(SI_CLK2_CONTROL, 0b11000000);
    SendRegister(19, 0b11000000);
    SendRegister(20, 0b11000000);
    SendRegister(21, 0b11000000);
    SendRegister(22, 0b11000000);
    SendRegister(23, 0b11000000);
    SendRegister(SI_CLK_OE, 0b11111111); // Disable all CLK outputs
  }
};

static SI5351 si5351;
enum mode_t { LSB, USB, CW, AM, FM };
volatile uint8_t mode = USB;
const char* mode_label[] = { "LSB", "USB", "CW ", "AM ", "FM " };
volatile bool change = true;
volatile int32_t freq = 7074000;
volatile bool ptt = false;
volatile uint8_t tx = 0;
volatile bool vox_enable = false;
enum dsp_cap_t { ANALOG, DSP, SDR };
static uint8_t dsp_cap = 0;
static uint8_t ssb_cap = 0;
volatile bool att_enable = false;
//#define PROFILING  1
#ifdef PROFILING
volatile uint32_t numSamples = 0;
#else
volatile uint8_t numSamples = 0;
#endif

inline void txen(bool en)
{
  if(en){
      lcd.setCursor(15, 1); lcd.print("T");
      si5351.SendRegister(SI_CLK_OE, 0b11111011); // CLK2_EN=1, CLK1_EN,CLK0_EN=0
      digitalWrite(RX, LOW);  // TX
  } else {
      digitalWrite(RX, !(att_enable)); // RX (enable RX when attenuator not on)
      si5351.SendRegister(SI_CLK_OE, 0b11111100); // CLK2_EN=0, CLK1_EN,CLK0_EN=1
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

#define MIC_ATTEN  0  // 0*6dB attenuation (note that the LSB bits are quite noisy)

// This is the ADC ISR, issued with sample-rate via timer1 compb interrupt.
// It performs in real-time the ADC sampling, calculation of SSB phase-differences, calculation of SI5351 frequency registers and send the registers to SI5351 over I2C.
void dsp_tx()
{ // jitter dependent things first
  ADCSRA |= (1 << ADSC);    // start next ADC conversion (trigger ADC interrupt if ADIE flag is set)
  OCR1BL = amp;                        // submit amplitude to PWM register (actually this is done in advance (about 140us) of phase-change, so that phase-delays in key-shaping circuit filter can settle)
  si5351.SendPLLBRegisterBulk();       // submit frequency registers to SI5351 over 731kbit/s I2C (transfer takes 64/731 = 88us, then PLL-loopfilter probably needs 50us to stabalize)
  //OCR1BL = amp;                        // submit amplitude to PWM register (takes about 1/32125 = 31us+/-31us to propagate) -> amplitude-phase-alignment error is about 30-50us
  int16_t adc = (ADCL | (ADCH << 8)) - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  int16_t df = ssb(adc >> MIC_ATTEN);  // convert analog input into phase-shifts (carrier out by periodic frequency shifts)
  si5351.freq_calc_fast(df);           // calculate SI5351 registers based on frequency shift and carrier frequency
  numSamples++;
}

static int32_t signal;
static int16_t avg = 0;
static int16_t maxpk=0;
static int16_t k0=0;
static int16_t k1=0;
static uint8_t sym;
static int16_t ta=0;
static char m2c[] = "##ETIANMSURWDKGOHVF#L#PJBXCYZQ##54S3###2##+###J16=/###H#7#G#8#90############?_####\"##.####@###'##-########;!#)#####,####:####";
static uint8_t nsamp=0;

char cw(int16_t in)
{
  char ch = 0;
  int i;
  signal += abs(in);
  #define OSR 64 // (8*FS/1000)
  if((nsamp % OSR) == 0){   // process every 8 ms
    nsamp=0;
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
  nsamp++;
  return ch;
}

static char out[] = "                ";
volatile bool cw_event = false;

//#define F_SAMP_RX 78125
#define F_SAMP_RX 62500  //overrun; sample rate of 55500 can be obtained
//#define F_SAMP_RX 52083
//#define F_SAMP_RX 44643
//#define F_SAMP_RX 39062
//#define F_SAMP_RX 34722
//#define F_SAMP_RX 31250
//#define F_SAMP_RX 28409
#define F_ADC_CONV 192307

volatile uint8_t volume = 8;
volatile bool agc = true;
volatile bool nr = false;

//static uint32_t gain = 1024;
static int16_t gain = 1024;
inline int16_t process_agc(int16_t in)
{
  //int16_t out = ((uint32_t)(gain) >> 20) * in;
  //gain = gain + (1024 - abs(out) + 512);
  int16_t out = (gain >= 1024) ? (gain >> 10) * in : in;
  //if(gain >= 1024) out = (gain >> 10) * in;  // net gain >= 1
  //else if(gain >= 16) out = ((gain >> 4) * in) >> 6;  // net gain < 1
  //else out = (gain * in) >> 10;
  int16_t accum = (1 - abs(out >> 10));
  if((INT16_MAX - gain) > accum) gain = gain + accum;
  if(gain < 1) gain = 1;
  return out;
}

inline int16_t process_nr(int16_t ac)
{
  ac = ac >> (6-abs(ac));  // non-linear below amp of 6; to reduce noise (switchoff agc and tune-up volume until noise dissapears, todo:extra volume control needed)
  ac = ac << 3;
  return ac;
}

#define JUNK  1
#ifdef JUNK
// Having this function included here and referenced makes sdr_rx faster 15% faster (normally including filt_cwn() in sdr_rx() makes the thing slower for an unknown reason)
void junk()
{ 
    filt_var(0);
}
#endif

volatile int8_t filt = -1;

inline int16_t filt_var(int16_t v)
{ 
  int16_t zx0 = v;

  static int16_t za1,za2;
  if(filt < 3){
    // 1st Order (SR=8kHz) IIR in Direct Form I, 8x8:16
    static int16_t zz1,zz2;
    zx0=(29*(zx0-zz1)+50*za1)/64;                               //300-Hz
    zz2=zz1;
    zz1=v;
  }
  za2=za1;
  za1=zx0;

  #define N_FILT 6
  // 4th Order (SR=8kHz) IIR in Direct Form I, 8x8:16
  //static int16_t za1,za2;
  static int16_t zb1,zb2;
  switch(filt){
    case 0: break; //0-4000Hz (pass-through)
    //case 1: zx0=(13*(zx0+2*za1+za2)-30*zb1-13*zb2)/64; break; //0-2500Hz 1st-order butterworth
    //case 1: zx0=(12*(zx0+2*za1+za2)-26*zb1-5*zb2)/64; break;    //0-2500Hz butterworth
    //case 1: zx0=(12*(zx0+2*za1+za2)+2*zb1-11*zb2)/64; break;    //0-2500Hz  elliptic
    //case 1: zx0=(10*(zx0+2*za1+za2)+7*zb1-11*zb2)/32; break;    //0-2500Hz  elliptic slower roll-off but deep
    case 1: zx0=(10*(zx0+2*za1+za2)+16*zb1-17*zb2)/32; break;    //0-2500Hz  elliptic -60dB@3kHz
    //case 2: zx0=(7*(zx0+2*za1+za2)+18*zb1-12*zb2)/64; break;  //0-1700Hz 1st-order
    //case 2: zx0=(7*(zx0+2*za1+za2)+16*zb1-3*zb2)/64; break;     //0-1700Hz butterworth
    case 2: zx0=(7*(zx0+2*za1+za2)+48*zb1-18*zb2)/32; break;     //0-1700Hz  elliptic
    case 3: zx0=(5*(zx0-2*za1+za2)+105*zb1-58*zb2)/64; break;   //650-840Hz
    case 4: zx0=(3*(zx0-2*za1+za2)+108*zb1-61*zb2)/64; break;   //650-750Hz
    case 5: zx0=((2*zx0-3*za1+2*za2)+111*zb1-62*zb2)/64; break; //630-680Hz
  }
  //za2=za1;
  //za1=v;
  zb2=zb1;
  zb1=zx0;

  static int16_t zc1,zc2;
  switch(filt){
    case 0: break; //0-4000Hz (pass-through)
    //case 1: break;
    //case 1: zx0=(16*(zx0+2*zb1+zb2)-36*zc1-31*zc2)/64; break;   //0-2500Hz butterworth
    //case 1: zx0=(8*(zx0+zb2)+13*zb1-46*zc1-48*zc2)/64; break;   //0-2500Hz  elliptic
    //case 1: zx0=(8*(zx0+2*zb1+zb2)-46*(zc1+zc2))/64; break;   //0-2500Hz  elliptic slower roll-off but deep
    case 1: zx0=(8*(zx0+zb2)+13*zb1-43*zc1-52*zc2)/64; break;   //0-2500Hz  elliptic -60dB@3kHz
    //case 2: break;
    //case 2: zx0=(16*(zx0+2*zb1+zb2)+22*zc1-29*zc2)/64; break;   //0-1700Hz butterworth
    case 2: zx0=(4*(zx0+zb1+zb2)+22*zc1-47*zc2)/64; break;   //0-1700Hz  elliptic
    case 3: zx0=((zx0+2*zb1+zb2)+97*zc1-57*zc2)/64; break;      //650-840Hz
    case 4: zx0=((zx0+zb1+zb2)+104*zc1-60*zc2)/64; break;       //650-750Hz
    case 5: zx0=((zb1)+109*zc1-62*zc2)/64; break;               //630-680Hz
  }
  zc2=zc1;
  zc1=zx0;
  return zx0;
}


/*
iterate through modes always from begin, skipping the current one
code definitions and re-use for comb, integrator, dc decoupling, arctan
add sdr_rx capability to skip ADCMUX changing and Q processing, (while oversampling I branch?)
in func_ptr for different mode types
agc based on rms256
skip adc processing while smeter lcd update?
vox by sampling mic port in sdr_rx?

 */

typedef void (*func_t)(void);
#ifdef JUNK
volatile func_t func_ptr = junk;
#else
volatile func_t func_ptr;
#endif

static uint32_t absavg256 = 0;
volatile uint32_t _absavg256 = 0;
volatile uint8_t admux[2];
volatile uint8_t _init;
volatile int16_t ocomb, i, q, qh;
#undef  R  // Decimating 2nd Order CIC filter
#define R 4  // Rate change from 52.083/2 kSPS to 6510.4SPS, providing 12dB gain

// Non-recursive CIC Filter (M=2, R=4) implementation, so two-stages of (followed by down-sampling with factor 2):
// H1(z) = (1 + z^-1)^2 = 1 + 2*z^-1 + z^-2 = (1 + z^-2) + (2) * z^-1 = FA(z) + FB(z) * z^-1;
// with down-sampling before stage translates into poly-phase components: FA(z) = 1 + z^-1, FB(z) = 2
// source: Lyons Understanding Digital Signal Processing 3rd edition 13.24.1
void sdr_rx()
{
  static int16_t zi1, zi2, zd1, zd2;

  // process I for even samples  [75% CPU@R=4;Fs=62.5k] (excluding the Comb branch and output stage)
  ADMUX = admux[1];  // set MUX for next conversion
  ADCSRA |= (1 << ADSC);    // start next ADC conversion
  func_ptr = sdr_rx_2;    // processing function for next conversion
  int16_t adc = (ADCL | (ADCH << 8)) - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  sdr_rx_common();

  // Only for I: correct I/Q sample delay by means of linear interpolation
  static int16_t prev_adc;
  int16_t corr_adc = (prev_adc + adc) / 2;
  prev_adc = adc;
  adc = corr_adc;

  static int16_t dc;
  dc += (adc - dc) / 2;
  int16_t ac = adc - dc;     // DC decoupling

  int16_t ac2;
  static int16_t z1;
  if(numSamples == 0 || numSamples == 4){  // 1st stage: down-sample by 2
    static int16_t za1;
    int16_t _ac = ac + za1 + z1;           // 1st stage: FA + FB
    za1 = ac;
    static int16_t _z1;
    if(numSamples == 0){                   // 2nd stage: down-sample by 2
      static int16_t _za1;
      ac2 = _ac + _za1 + _z1;              // 2nd stage: FA + FB
      _za1 = _ac;
      {
        // post processing I and Q (down-sampled) results
        static int16_t v[8];
        v[7] = ac2;
        
        i = v[0]; v[0] = v[1]; v[1] = v[2]; v[2] = v[3]; v[3] = v[4]; v[4] = v[5]; v[5] = v[6]; v[6] = v[7];  // Delay to match Hilbert transform on Q branch
            
        int16_t ac = i + qh;
        static uint8_t absavg256cnt;
        if(!(absavg256cnt--)){ _absavg256 = absavg256; absavg256 = 0; } else absavg256 += abs(ac);
    
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
          ac = z0 - z1; // Differentiator
          z1 = z0;
          //ac = ac * (F_SAMP_RX/R) / _UA;  // =ac*3.5 -> skip
        }  // needs: p.12 https://www.veron.nl/wp-content/uploads/2014/01/FmDemodulator.pdf
        else { ; }  // USB, LSB, CW
        if(agc) ac = process_agc(ac);
        ac = ac >> (16-volume);
        if(nr) ac = process_nr(ac);
        if(mode == USB || mode == LSB){
            if(filt != -1) ac = filt_var(ac << 0);
        }
        if(mode == CW){
          if(filt != -1) ac = filt_var(ac << 6);
#define CW_DECODER  1
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
    
        // Output stage
        static int16_t ozd1, ozd2;
        if(_init){ ac = 0; ozd1 = 0; ozd2 = 0; _init = 0; } // hack: on first sample init accumlators of further stages (to prevent instability)
        int16_t od1 = ac - ozd1; // Comb section
        ocomb = od1 - ozd2;
        ozd2 = od1;
        ozd1 = ac;
      }
    } else _z1 = _ac * 2;
  } else z1 = ac * 2;

  numSamples++;
}

void sdr_rx_2()
{
  // process Q for odd samples  [75% CPU@R=4;Fs=62.5k] (excluding the Comb branch and output stage)
  ADMUX = admux[0];  // set MUX for next conversion
  ADCSRA |= (1 << ADSC);    // start next ADC conversion
  func_ptr = sdr_rx;    // processing function for next conversion
  int16_t adc = (ADCL | (ADCH << 8)) - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH

  static int16_t dc;
  dc += (adc - dc) / 2;
  int16_t ac = adc - dc;     // DC decoupling

  int16_t ac2;
  static int16_t z1;
  if(numSamples == 3 || numSamples == 7){  // 1st stage: down-sample by 2
    static int16_t za1;
    int16_t _ac = ac + za1 + z1;           // 1st stage: FA + FB
    za1 = ac;
    static int16_t _z1;
    if(numSamples == 7){                   // 2nd stage: down-sample by 2
      static int16_t _za1;
      ac2 = _ac + _za1 + _z1;              // 2nd stage: FA + FB
      _za1 = _ac;
      {
        // Process Q (down-sampled) samples
        static int16_t v[16];
        v[15] = ac2;

        for(uint8_t j = 0; j != 15; j++) v[j] = v[j + 1];
        q = v[7];
        qh = ((v[0] - v[14]) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 15) / 128 + (v[6] - v[8]) / 2; // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)
      }
#ifndef PROFILING
      numSamples = 0; return;
#endif
    } else _z1 = _ac * 2;
  } else z1 = ac * 2;
  
  numSamples++;
}

inline void sdr_rx_common()
{
  static int16_t ozi1, ozi2;
  if(_init){ ocomb=0; ozi1 = 0; ozi2 = 0; } // hack
  // Output stage [25% CPU@R=4;Fs=62.5k]
  ozi2 = ozi1 + ozi2;          // Integrator section 
  ozi1 = ocomb + ozi1;
#ifndef PROFILING
  if(volume) OCR1AL = min(max((ozi2>>5) + ICR1L/2, 0), ICR1L);  // center and clip wrt PWM working range
#endif  
}

ISR(TIMER2_COMPA_vect)  // Timer2 COMPA interrupt
{
  func_ptr();
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

char blanks[] = "        ";
#define N_FONTS  5
byte font[][8] = {
{ 0b01000,  // 0
  0b00100,
  0b01010,
  0b00101,
  0b01010,
  0b00100,
  0b01000,
  0b00000 },
{ 0b00000,  // 1
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000 },
{ 0b10000,  // 2
  0b10000,
  0b10000,
  0b10000,
  0b10000,
  0b10000,
  0b10000,
  0b10000 },
{ 0b10000,  // 3
  0b10000,
  0b10100,
  0b10100,
  0b10100,
  0b10100,
  0b10100,
  0b10100 },
{ 0b10000,  // 4
  0b10000,
  0b10101,
  0b10101,
  0b10101,
  0b10101,
  0b10101,
  0b10101 }
};

void customDelay(uint32_t _micros)  //_micros=100000 is 132052us delay
{
  uint32_t i; for(i = 0; i != _micros * 3; i++) wdt_reset();
}

void(* resetFunc)(void) = 0; // declare reset function @ address 0

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

class QCX {
public:
  QCX(){
  }
  void setup(){  
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
  
  float smeter(float ref = 5.1)  //= 10*log(F_SAMP_RX/R/2400)  ref to 2.4kHz BW.
  {
    float rms = _absavg256 / 256; //sqrt(256.0);
    if(dsp_cap == SDR) rms = (float)rms * 1.1 / (1024.0 * (float)R * 100.0 * 50.0);          // rmsV = ADC value * AREF / [ADC DR * processing gain * receiver gain * audio gain]
    else               rms = (float)rms * 5.0 / (1024.0 * (float)R * 100.0 * 120.0 / 1.750);
    float dbm = (10.0 * log10((rms * rms) / 50.0) + 30.0) - ref; //from rmsV to dBm at 50R
    static float dbm_max;
    dbm_max = max(dbm_max, dbm);
    static uint8_t cnt;
    cnt++;
    if((cnt % 8) == 0){
  #define DBM_METER  1
  #ifdef DBM_METER
      lcd.setCursor(9, 0); lcd.print((int16_t)dbm_max); lcd.print((ref == 0.0) ? "dBm   " : "dB    ");
  #endif
  //#define SNR_METER  1
  #ifdef SNR_METER
      uint8_t s = (dbm_max < -63) ? ((dbm_max - -127) / 6) : (uint8_t)(dbm_max - -63 + 10) % 10;  // dBm to S
      lcd.setCursor(14, 0); if(s < 10){ lcd.print("S"); } lcd.print(s); 
  #endif
      dbm_max = -174.0 + 34.0;
    }
  //#define SNR_BAR  1
  #ifdef SNR_BAR
    int8_t s = (dbm < -63) ? ((dbm - -127) / 6) : (uint8_t)(dbm - -63 + 10) % 10;  // dBm to S
    lcd.setCursor(12, 0); 
    char tmp[5];
    tmp[0] = (char)(s<1?0x02:s<2?0x03:s<3?0x04:0x05); s = s - 3;
    tmp[1] = (char)(s<1?0x02:s<2?0x03:s<3?0x04:0x05); s = s - 3;
    tmp[2] = (char)(s<1?0x02:s<2?0x03:s<3?0x04:0x05); s = s - 3;
    tmp[3] = (char)(s<1?0x02:s<2?0x03:s<3?0x04:0x05); s = s - 3;
    tmp[4] = 0;
    lcd.print(tmp);
    //for(i = 0; i != 4; i++){ lcd.print((char)(s<1?0x02:s<2?0x03:s<3?0x04:0x05)); s = s - 3; }
  #endif
    return dbm;
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
  
  float dbmeter(float ref = 0.0)
  {
    float rms = ((float)sample_amp(AUDIO1, 128)) * 5.0 / (1024.0 * 128.0 * 100.0); // rmsV = ADC value * AREF / [ADC DR * processing gain * receiver gain * audio gain]
    float dbm = (10.0 * log10((rms * rms) / 50.0) + 30.0) - ref; //from rmsV to dBm at 50R
    lcd.setCursor(9, 0); lcd.print(dbm); lcd.print("dB    ");
    delay(300);
    return dbm;
  }
  
  // RX I/Q calibration procedure: terminate with 50 ohm, enable CW filter, adjust R27, R24, R17 subsequently to its minimum side-band rejection value in dB
  void calibrate_iq()
  {
    lcd.setCursor(9, 0); lcd.print(blanks);
    digitalWrite(SIG_OUT, true); // loopback on
    si5351.prev_pll_freq = 0;  //enforce PLL reset
    si5351.freq(freq, 0, 90);  // RX in USB
    float dbc;
    si5351.alt_clk2(freq + 700);
    dbc = dbmeter();
    si5351.alt_clk2(freq - 700);
    lcd.setCursor(0, 1); lcd.print("I-Q bal. (700 Hz)"); lcd.print(blanks);
    for(; !digitalRead(BUTTONS);){ wdt_reset(); dbmeter(dbc); } for(; digitalRead(BUTTONS);) wdt_reset();
    si5351.alt_clk2(freq + 600);
    dbc = dbmeter();
    si5351.alt_clk2(freq - 600);
    lcd.setCursor(0, 1); lcd.print("Phase Lo (600 Hz)"); lcd.print(blanks);
    for(; !digitalRead(BUTTONS);){ wdt_reset(); dbmeter(dbc); } for(; digitalRead(BUTTONS);) wdt_reset();
    si5351.alt_clk2(freq + 800);
    dbc = dbmeter();
    si5351.alt_clk2(freq - 800);
    lcd.setCursor(0, 1); lcd.print("Phase Hi (800 Hz)"); lcd.print(blanks);
    for(; !digitalRead(BUTTONS);){ wdt_reset(); dbmeter(dbc); } for(; digitalRead(BUTTONS);) wdt_reset();
  /*
    for(uint32_t offset = 0; offset < 3000; offset += 100){
      si5351.alt_clk2(freq + offset);
      dbc = dbmeter();
      si5351.alt_clk2(freq - offset);
      lcd.setCursor(0, 1); lcd.print(offset); lcd.print(" Hz"); lcd.print(blanks);
      wdt_reset(); dbmeter(dbc); delay(500); wdt_reset(); 
    }
  */
    lcd.setCursor(9, 0); lcd.print(blanks);  // cleanup dbmeter
    digitalWrite(SIG_OUT, false); // loopback off
    si5351.SendRegister(SI_CLK_OE, 0b11111100); // CLK2_EN=0, CLK1_EN,CLK0_EN=1
    change = true;  //restore original frequency setting
  
  }
  
  void powermeter()
  {
    lcd.setCursor(9, 0); lcd.print(blanks);
    si5351.prev_pll_freq = 0;  //enforce PLL reset
    si5351.freq(freq, 0, 90);  // RX in USB
    si5351.alt_clk2(freq + 1000); // si5351.freq_clk2(freq + 1000);
    si5351.SendRegister(SI_CLK_OE, 0b11111000); // CLK2_EN=1, CLK1_EN,CLK0_EN=1
    digitalWrite(KEY_OUT, HIGH); //OCR1BL = 0xFF;
    digitalWrite(RX, LOW);  // TX
  
    //if(dsp_cap != SDR){ adc_start(1, false, F_ADC_CONV); admux[0] = ADMUX; admux[1] = ADMUX; }
    wdt_reset(); delay(100); 
  
    float rms;
    rms = ((float)sample_amp(AUDIO2, 128)) * 5.0 / (1024.0 * 128.0 * 100.0 / 10000.0); // rmsV = ADC value * AREF / [ADC DR * processing gain * receiver gain * rx/tx switch attenuation]
    //if(dsp_cap == SDR) rms = (float)rms256 * 1.1 / (1024.0 * (float)R * 256.0 * 100.0 * 50.0 / 10000);          // rmsV = ADC value * AREF / [ADC DR * processing gain * receiver gain * audio gain]
    //else               rms = (float)rms256 * 5.0 / (1024.0 * (float)R * 256.0 * 100.0 / 10000);
    float w = (rms * rms) / 50.0;
    float dbm = 10.0 * log10(w) + 30.0; //from rmsV to dBm at 50R
  
    lcd.setCursor(9, 0); lcd.print("  "); lcd.print(w); lcd.print("W   ");
    //lcd.setCursor(9, 0); lcd.print(" +"); lcd.print((int16_t)dbm ); lcd.print("dBm   ");
  
    digitalWrite(KEY_OUT, LOW); //OCR1BL = 0x00;
    digitalWrite(RX, HIGH);  // RX
    si5351.SendRegister(SI_CLK_OE, 0b11111100); // CLK2_EN=0, CLK1_EN,CLK0_EN=1
    change = true;  //restore original frequency setting
    delay(1000);
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
  
    si5351.prev_pll_freq = 0;  // enforce PLL reset
    si5351.freq(freq, 0, 90);  // RX in USB
    si5351.alt_clk2(freq);
    si5351.SendRegister(SI_CLK_OE, 0b11111011); // CLK2_EN=1, CLK1_EN,CLK0_EN=0
    digitalWrite(RX, LOW);  // TX
    uint16_t i;
    for(i = 0; i != 256; i++)
    {
      OCR1BL = lut[i];
      si5351.alt_clk2(freq + i * 10);
      wdt_reset();
      lcd.setCursor(0, 1); lcd.print("SWEEP("); lcd.print(i); lcd.print(")="); lcd.print(lut[i]); lcd.print(blanks);
      delay(200);
    }
  
    OCR1BL = 0;
    delay(500);
    digitalWrite(RX, HIGH);  // RX
    si5351.SendRegister(SI_CLK_OE, 0b11111100); // CLK2_EN=0, CLK1_EN,CLK0_EN=1
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
  
    si5351.prev_pll_freq = 0;  //enforce PLL reset
    si5351.freq(freq, 0, 90);  // RX in USB
    si5351.alt_clk2(freq + 1000); // si5351.freq_clk2(freq + 1000);
    si5351.SendRegister(SI_CLK_OE, 0b11111000); // CLK2_EN=1, CLK1_EN,CLK0_EN=1
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
    si5351.SendRegister(SI_CLK_OE, 0b11111100); // CLK2_EN=0, CLK1_EN,CLK0_EN=1
    change = true;  //restore original frequency setting
  }
  
  void start_rx()
  {
  //if(!vox_enable) txen(false);
    timer2_stop();
    timer1_stop();
    adc_stop();
    tx = 1;  // transit from TX to RX
    _init = 1;  
    numSamples = 0;
    func_ptr = sdr_rx;  //enable RX DSP/SDR
    if(dsp_cap == SDR){
      adc_start(0, true, F_ADC_CONV); admux[0] = ADMUX;
      adc_start(1, true, F_ADC_CONV); admux[1] = ADMUX;
      timer2_start(F_SAMP_RX);
      timer1_start(F_SAMP_RX);
    } else { // ANALOG, DSP
      adc_start(0, false, F_ADC_CONV); admux[0] = ADMUX; admux[1] = ADMUX;
      timer2_start(F_SAMP_RX);
      timer1_start(F_SAMP_RX);
    }
  }
  
  void start_tx()
  {
    timer2_stop();
    timer1_stop();
    adc_stop();  //disable RX DSP
    tx = 0;  // transit from RX to TX
    numSamples = 0;
    func_ptr = dsp_tx;
    amp = 0;  // initialize
    adc_start(2, true, F_ADC_CONV);
    timer2_start(F_SAMP_TX);
    timer1_start(39250);
    //if(!vox_enable) txen(true);
  }
    
  uint8_t bandval = 2;
  #define N_BANDS 11
  uint32_t band[N_BANDS] = { /*472000, 1840000,*/ 3573000, 5357000, 7074000, 10136000, 14074000, 18100000, 21074000, 24915000, 28074000, 50313000, 70101000/*, 144125000*/ };  // { 3573000, 5357000, 7074000, 10136000, 14074000, 18100000, 21074000 };
  
  enum step_t { STEP_10M, STEP_1M, STEP_500k, STEP_100k, STEP_10k, STEP_1k, STEP_500, STEP_100, STEP_10, STEP_1 };
  int32_t stepsizes[10] = { 10000000, 1000000, 500000, 100000, 10000, 1000, 500, 100, 10, 1 };
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
  
  void powerDown()
  { // Reduces power from 110mA to 70mA (back-light on) or 30mA (back-light off), remaining current is probably opamp quiescent current
    lcd.setCursor(0, 1); lcd.print("Power-off 73 :-)"); lcd.print(blanks);
  
    MCUSR = ~(1<<WDRF);  // fix: wdt_disable() bug
    wdt_disable();
  
    timer2_stop();
    timer1_stop();
    adc_stop();
  
    si5351.powerDown();
  
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
};

QCX qcx;

#define SAFE  1
void setup()
{
  init();

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

  wdt_enable(WDTO_4S);  // Enable watchdog, resolves QCX startup issue
#endif

  // disable external interrupts
  PCICR = 0;
  PCMSK0 = 0;
  PCMSK1 = 0;
  PCMSK2 = 0;

  encoder_setup();

  qcx.setup();

  lcd.begin(16, 2);  // Init LCD
  for(i = 0; i != N_FONTS; i++)  // Init fonts
    lcd.createChar(0x01 + i, font[i]);

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
  si5351.SendRegister(SI_CLK_OE, 0b11111111); // Mute QSD: CLK2_EN=CLK1_EN,CLK0_EN=0  
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

#ifdef SAFE
  // Measure CPU loads
  if(!(load_tx < 100.0))
  {
    lcd.setCursor(0, 1); lcd.print("!!CPU_tx="); lcd.print(load_tx); lcd.print("%"); lcd.print(blanks);
    delay(1500); wdt_reset();
  }

  {
    lcd.setCursor(0, 1); lcd.print("!!CPU_rx"); lcd.print(0); lcd.print("="); lcd.print(load_rx[0]); lcd.print("%"); lcd.print(blanks);
    delay(1500); wdt_reset();
  }
  if(!(load_rx_avg < 100.0))
  {
    lcd.setCursor(0, 1); lcd.print("!!CPU_rx"); lcd.print("="); lcd.print(load_rx_avg); lcd.print("%"); lcd.print(blanks);
    delay(1500); wdt_reset();
    // and specify indivual timings for each of the eight alternating processing functions:
    for(i = 1; i != 8; i++){
      if(!(load_rx[i] < 100.0))
      {
        lcd.setCursor(0, 1); lcd.print("!!CPU_rx"); lcd.print(i); lcd.print("="); lcd.print(load_rx[i]); lcd.print("%"); lcd.print(blanks);
        delay(1500); wdt_reset();
      }
    }
  }

  // Measure VDD (+5V); should be ~5V
  si5351.SendRegister(SI_CLK_OE, 0b11111111); // Mute QSD: CLK2_EN=CLK1_EN,CLK0_EN=0
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
  si5351.freq(freq, 0, 90);
  wdt_reset();
  t0 = micros();
  for(i = 0; i != 1000; i++) si5351.SendPLLBRegisterBulk();
  t1 = micros();
  uint32_t speed = (1000000 * 8 * 7) / (t1 - t0); // speed in kbit/s
  if(false)
  {
    lcd.setCursor(0, 1); lcd.print("i2cspeed="); lcd.print(speed); lcd.print("kbps"); lcd.print(blanks);
    delay(1500); wdt_reset();
  }

  // Measure I2C Bit-Error Rate (BER); should be error free for a thousand random bulk PLLB writes
  si5351.freq(freq, 0, 90);
  wdt_reset();
  uint16_t i2c_error = 0;  // number of I2C byte transfer errors
  for(i = 0; i != 1000; i++){
    si5351.freq_calc_fast(i);
    //for(int j = 3; j != 8; j++) si5351.pll_data[j] = rand();
    si5351.SendPLLBRegisterBulk();
    for(int j = 3; j != 8; j++) if(si5351.RecvRegister(SI_SYNTH_PLL_B + j) != si5351.pll_data[j]) i2c_error++;
  }
  if(i2c_error){
    lcd.setCursor(0, 1); lcd.print("!!BER_i2c="); lcd.print(i2c_error); lcd.print(""); lcd.print(blanks);
    delay(1500); wdt_reset();
  }
#endif

  //delay(800); wdt_reset();  // Display banner
  lcd.setCursor(7, 0); lcd.print("\x01"); lcd.print(blanks);

  volume = (dsp_cap) ? ((dsp_cap == SDR) ? 8 : 14) : 0;
  mode = (dsp_cap || ssb_cap) ? USB : CW;
  qcx.start_rx();
}
  
void loop()
{
  //delay(10);
  delay(100);

  qcx.smeter();
  if(mode == CW && cw_event){
    cw_event = false;
    lcd.setCursor(0, 1); lcd.print(out);
  }
  
  if(!digitalRead(DIT)){
    ptt = true;
    qcx.start_tx();
    for(; !digitalRead(DIT);){ //until released
      wdt_reset();
    }
    ptt = false;
    delay(1);
    qcx.start_rx();
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
          wdt_reset(); delay(1500); wdt_reset();
          change = true; // refresh display
        }
        break;
      case BL|DC:
        //qcx.powerDown();
        filt++;
        _init = true;
        //if(filt > N_FILT) filt = -1;
        if(mode == CW && filt > N_FILT-1) filt = 3;
        if(mode != CW && filt > 2) filt = -1;
        lcd.setCursor(0, 1); lcd.print("Filter: "); lcd.print(filt); lcd.print(blanks);
        wdt_reset(); delay(1500); wdt_reset();
        change = true; // refresh display
        break;
      case BL|PL:
        //calibrate_predistortion();
        qcx.powermeter();
        //test_tx_amp();
        break;
      case BL|PT: break;
      case BR|SC:
        mode++;  // mode change
        if(mode != CW) qcx.stepsize = qcx.STEP_1k; else qcx.stepsize = qcx.STEP_100;
        if(mode > FM) mode = LSB;
        if(mode == CW) filt = 3; else filt = -1;
        si5351.prev_pll_freq = 0;  // enforce PLL reset
        change = true;
        break;
      case BR|DC:
        if(drive == 0) drive = 1;
        else drive += 1;
        if(drive > 8) drive = 0;
        lcd.setCursor(0, 1); lcd.print("Drive: "); lcd.print(drive); lcd.print(blanks);
        agc = (drive == 4);
        nr = (drive % 2);
        break;
      case BR|PL:
        vox_enable = true;
        qcx.start_tx();
        for(; !digitalRead(BUTTONS);){ // while in VOX mode
          wdt_reset();  // until 2nd press
        }
        vox_enable = false;
        delay(100);
        qcx.start_rx();
        delay(100);
        break;
      case BR|PT: break;
      case BE|SC: qcx.stepsize_change(+1); break;
      case BE|DC:
        delay(100);
        qcx.bandval++;
        if(qcx.bandval >= N_BANDS) qcx.bandval = 0;
        freq = qcx.band[qcx.bandval];
        qcx.stepsize = qcx.STEP_1k;
        change = true;
        break;
      case BE|PL: qcx.stepsize_change(-1); break;
      case BE|PT:
          for(; digitalRead(BUTTONS);){ // process encoder changes until released
          wdt_reset();
          if(encoder_val != 0 && (dsp_cap)){
            int16_t tmp = volume;
            tmp += encoder_val;
            encoder_val = 0;
            if(tmp < 0) qcx.powerDown();  // powerDown when volume < 0
            volume = max(0, min(tmp, 255));
            lcd.setCursor(0, 1); lcd.print("Volume "); lcd.print(volume); lcd.print(blanks);
          }
        }
        change = true; // refresh display
        break;
    }
  }
  if(encoder_val){  // process encoder tuning steps
    qcx.process_encoder_tuning_step(encoder_val);
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
      si5351.freq(freq, 90, 0);  // RX in LSB
    else
      si5351.freq(freq, 0, 90);  // RX in USB
  }
  qcx.stepsize_showcursor();
  wdt_reset();
}
