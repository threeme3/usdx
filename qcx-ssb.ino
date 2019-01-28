// Arduino Sketch of the QCX-SSB: SSB with your QCX transceiver (modification)
//
// https://github.com/threeme3/QCX-SSB 
//
#define VERSION   "1.00"

// QCX pin defintion
#define LCD_D4  0
#define LCD_D5  1
#define LCD_D6  2
#define LCD_D7  3
#define LCD_EN  4
#define ROT_A   6
#define ROT_B   7
#define RX      8
#define KEY_OUT 10
#define SIG_OUT 11
#define DAH     12
#define DIT     13
#define AUDIO1  14 //A0
#define AUDIO2  15 //A1
#define DVM     16 //A2
#define BUTTONS 17 //A3
#define LCD_RS  18
#define SDA     18 //hmm.. shared with LCD_RS
#define SCL     19

#include <LiquidCrystal.h>
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

#include <inttypes.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#define F_CPU 20000000   // Crystal frequency of XTAL1

#define I2C_DDR DDRC     // Pins for the I2C bit banging
#define I2C_PIN PINC
#define I2C_PORT PORTC
#define I2C_SDA (1 << 4) // PC4 (Pin 18)
#define I2C_SCL (1 << 5) // PC5 (Pin 19)

#define I2C_SDA_HI() I2C_DDR &= ~I2C_SDA;
#define I2C_SDA_LO() I2C_DDR |= I2C_SDA;
#define I2C_SCL_HI() I2C_DDR &= ~I2C_SCL; asm("nop"); asm("nop");
#define I2C_SCL_LO() I2C_DDR |= I2C_SCL; asm("nop"); asm("nop");

inline void i2c_start()
{
  i2c_resume();  //prepare for I2C
  I2C_SDA_LO();
  I2C_SCL_LO();
}

inline void i2c_stop()
{
  I2C_SCL_HI();
  I2C_SDA_HI();
  I2C_DDR &= ~(I2C_SDA | I2C_SCL); //prepare for a start
  i2c_suspend();
}

inline void i2c_SendBit(uint8_t data, uint8_t mask)
{
  if(data & mask){
    I2C_SDA_HI();
  } else {
    I2C_SDA_LO();
  }
  I2C_SCL_HI();
  I2C_SCL_LO();
}

inline void i2c_SendByte(uint8_t data)
{
  i2c_SendBit(data, 1<<7);
  i2c_SendBit(data, 1<<6);
  i2c_SendBit(data, 1<<5);
  i2c_SendBit(data, 1<<4);
  i2c_SendBit(data, 1<<3);
  i2c_SendBit(data, 1<<2);
  i2c_SendBit(data, 1<<1);
  i2c_SendBit(data, 1<<0);
  I2C_SDA_HI();  //ack
  asm("nop"); asm("nop");  //delay
  I2C_SCL_HI();
  I2C_SCL_LO();
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
  I2C_PORT &= ~I2C_SDA; // Pin sharing SDA/LCD_RS mitigation
}

inline void i2c_suspend()
{
  I2C_DDR |= I2C_SDA;   // Pin sharing SDA/LCD_RS mitigation
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

#define SI_R_DIV_1 0b00000000 // R-division ratio definitions
#define SI_R_DIV_32 0b01010000
#define SI_R_DIV_128 0b01110000
#define SI_MS_INT 0b01000000  // Clock control
#define SI_CLK_SRC_PLL_A 0b00000000
#define SI_CLK_SRC_PLL_B 0b00100000
#define SI_CLK_SRC_MS 0b00001100
#define SI_CLK_IDRV_8MA 0b00000011

#define SI_XTAL_FREQ 27003843 // Measured crystal frequency of XTAL2 for CL = 10pF

volatile int32_t si5351_raw_freq;
volatile uint8_t si5351_prev_divider;
volatile uint8_t si5351_divider;  // note: because of int8 only freq > 3.6MHz can be covered for R_DIV=1
volatile uint8_t si5351_mult;
volatile uint8_t si5351_pll_data[8];
static int32_t si5351_prev_pll_freq = 0;

void si5351_SendRegister(uint8_t reg, uint8_t data)
{
  i2c_start();
  i2c_SendByte(SI_I2C_ADDR << 1);
  i2c_SendByte(reg);
  i2c_SendByte(data);
  i2c_stop();
}

inline void si5351_SendPLLBRegister()
{
  i2c_start();
  i2c_SendByte(SI_I2C_ADDR << 1);
  i2c_SendByte(SI_SYNTH_PLL_B + 3);  // Skip the first three pll_data bytes (first two always 0xFF and thirds not often changing
  i2c_SendByte(si5351_pll_data[3]);
  i2c_SendByte(si5351_pll_data[4]);
  i2c_SendByte(si5351_pll_data[5]);
  i2c_SendByte(si5351_pll_data[6]);
  i2c_SendByte(si5351_pll_data[7]);
  i2c_stop();
}

// Set up specified PLL with si5351_mult, num and denom
// si5351_mult is 15..90
// num is 0..1,048,575 (0xFFFFF)
// denom is 0..1,048,575 (0xFFFFF)
void si5351_SetupPLL(uint8_t pll, uint8_t mult, uint32_t num, uint32_t denom)
{
  uint32_t P1; // PLL config register P1
  uint32_t P2; // PLL config register P2
  uint32_t P3; // PLL config register P3

  P1 = (uint32_t)(128 * ((float)num / (float)denom));
  P1 = (uint32_t)(128 * (uint32_t)(mult) + P1 - 512);
  P2 = (uint32_t)(128 * ((float)num / (float)denom));
  P2 = (uint32_t)(128 * num - denom * P2);
  P3 = denom;

  si5351_SendRegister(pll + 0, (P3 & 0x0000FF00) >> 8);
  si5351_SendRegister(pll + 1, (P3 & 0x000000FF));
  si5351_SendRegister(pll + 2, (P1 & 0x00030000) >> 16);
  si5351_SendRegister(pll + 3, (P1 & 0x0000FF00) >> 8);
  si5351_SendRegister(pll + 4, (P1 & 0x000000FF));
  si5351_SendRegister(pll + 5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
  si5351_SendRegister(pll + 6, (P2 & 0x0000FF00) >> 8);
  si5351_SendRegister(pll + 7, (P2 & 0x000000FF));
}

// Set up si5351_multiSynth with integer si5351_divider and R si5351_divider
// R si5351_divider is the bit value which is OR'ed onto the appropriate register
void si5351_SetupMultisynth(uint8_t synth, uint32_t divider, uint8_t rDiv)
{
  uint32_t P1; // Synth config register P1
  uint32_t P2; // Synth config register P2
  uint32_t P3; // Synth config register P3

  P1 = 128 * divider - 512;
  P2 = 0; // P2 = 0, P3 = 1 forces an integer value for the si5351_divider
  P3 = 1;

  si5351_SendRegister(synth + 0, (P3 & 0x0000FF00) >> 8);
  si5351_SendRegister(synth + 1, (P3 & 0x000000FF));
  si5351_SendRegister(synth + 2, ((P1 & 0x00030000) >> 16) | rDiv);
  si5351_SendRegister(synth + 3, (P1 & 0x0000FF00) >> 8);
  si5351_SendRegister(synth + 4, (P1 & 0x000000FF));
  si5351_SendRegister(synth + 5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
  si5351_SendRegister(synth + 6, (P2 & 0x0000FF00) >> 8);
  si5351_SendRegister(synth + 7, (P2 & 0x000000FF));
}

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
  si5351_pll_data[3] = P1 >> 8;
  si5351_pll_data[4] = P1;
  si5351_pll_data[5] = 0xF0 | (P2 >> 16);
  si5351_pll_data[6] = P2 >> 8;
  si5351_pll_data[7] = P2;
}

void si5351_freq(uint32_t freq, uint8_t i, uint8_t q)
{
  uint8_t r_div = (freq > 4000000) ? 1 : (freq > 400000) ? 32 : 128; // make sure that si5351_divider is in range 6..255
  freq *= r_div;  // Calculate frequency before r_div

  // freq is in the range 1MHz to 150MHz
  si5351_divider = 900000000 / freq;  // Calculate the division ratio. 900,000,000 is the maximum internal PLL freq: 900MHz
  if(si5351_divider % 2) si5351_divider--;   // Ensure an even integer division ratio
  if( (si5351_divider * (freq - 5000) / SI_XTAL_FREQ) != (si5351_divider * (freq + 5000) / SI_XTAL_FREQ) ) si5351_divider -= 2;  // Test if si5351_multiplier remains same for freq deviation +/- 5kHz, if not use different si5351_divider to make same
  int32_t pll_freq = si5351_divider * freq; // Calculate the pll_freq: the si5351_divider * desired output freq
  
  si5351_raw_freq = freq;   // frequency of PLL and MS without taking R divider into account; used by si5351_freq_calc_fast()

  si5351_mult = pll_freq / SI_XTAL_FREQ;  // Determine the si5351_multiplier to get to the required pll_freq (in the range 15..90)
  uint64_t l = pll_freq % SI_XTAL_FREQ;  // // distance of pll_freq with si5351_multiple of xtal frequency. It has three parts:
  l <<= 20; l--;        // l *= 1048575;  
  l /= SI_XTAL_FREQ;       // normalize
  uint32_t num = l;     // the actual si5351_multiplier is  si5351_mult + num / denom      
  //num and denom are the fractional parts, the numerator and denominator each is 20 bits (range 0..1048575)
  const uint32_t denom = 0xFFFFF;   // For simplicity we set the denominator to the maximum 1048575
  
  // Set up specified PLL with si5351_mult, num and denom: si5351_mult is 15..90, num is 0..1,048,575 (0xFFFFF), denom is 0..1,048,575 (0xFFFFF)
  uint32_t term = num * 128 / denom;                // 128.0 * (num / denom)
  uint32_t P1 = 128 * si5351_mult + term - 512;
  uint32_t P2 = 128 * num - denom * term; 
  uint32_t P3 = denom;
  si5351_pll_data[0] = 0xFF;
  si5351_pll_data[1] = 0xFF;
  si5351_pll_data[2] = (P1 >> 14) & 0x0C;
  si5351_pll_data[3] = P1 >> 8;
  si5351_pll_data[4] = P1;
  si5351_pll_data[5] = 0xF0 | ((P2 & 0x000F0000) >> 16);
  si5351_pll_data[6] = P2 >> 8;
  si5351_pll_data[7] = P2;

  // Set up PLL A and PLL B with the calculated  si5351_multiplication ratio
  si5351_SetupPLL(SI_SYNTH_PLL_A, si5351_mult, num, denom);
  si5351_SetupPLL(SI_SYNTH_PLL_B, si5351_mult, num, denom);
  // Set up si5351_multiSynth si5351_divider 0,1,2 with the calculated si5351_divider, from 6..1800.
  // The final R division stage can divide by a power of two, from 1..128 represented by constants SI_R_DIV1 to SI_R_DIV128
  // if you want to output frequencies below 1MHz, you have to use the final R division stage
  uint8_t rDiv = r_div == 1 ? SI_R_DIV_1 : r_div == 32 ? SI_R_DIV_32 : SI_R_DIV_128;
  si5351_SetupMultisynth(SI_SYNTH_MS_0, si5351_divider, rDiv);
  si5351_SetupMultisynth(SI_SYNTH_MS_1, si5351_divider, rDiv);
  si5351_SetupMultisynth(SI_SYNTH_MS_2, si5351_divider, rDiv);
  // Set I/Q phase    
  si5351_SendRegister(SI_CLK0_PHOFF, i * si5351_divider / 90); // one LSB equivalent to a time delay of Tvco/4 range 0..127
  si5351_SendRegister(SI_CLK1_PHOFF, q * si5351_divider / 90); // one LSB equivalent to a time delay of Tvco/4 range 0..127
  // Switch on the CLK0, CLK1 output to be PLL A and set si5351_multiSynth0, si5351_multiSynth1 input 
  // (0x4F = SI_MS0_INT | SI_CLK_SRC_MS | SI_CLK_IDRV_8MA)
  si5351_SendRegister(SI_CLK0_CONTROL, 0x4F | SI_CLK_SRC_PLL_A);
  si5351_SendRegister(SI_CLK1_CONTROL, 0x4F | SI_CLK_SRC_PLL_A);
  // Switch on the CLK2 output to be PLL B and set si5351_multiSynth2 input 
  si5351_SendRegister(SI_CLK2_CONTROL, 0x4F | SI_CLK_SRC_PLL_B);
  // Reset the PLL. This causes a glitch in the output. For small changes to
  // the parameters, you don't need to reset the PLL, and there is no glitch
  if((abs(pll_freq - si5351_prev_pll_freq) > 16000000L) || si5351_divider != si5351_prev_divider) {
    si5351_prev_pll_freq = pll_freq;
    si5351_prev_divider = si5351_divider;
    si5351_SendRegister(SI_PLL_RESET, 0xA0);
  }
  si5351_SendRegister(SI_CLK_OE, 0b11111100); // Enable CLK1_EN|CLK0_EN
}

void si5351_freq_clk2(uint32_t freq)
{
  uint8_t r_div = (freq > 4000000) ? 1 : (freq > 400000) ? 32 : 128; // make sure that si5351_divider is in range 6..255
  freq *= r_div;  // Calculate frequency before r_div

  // freq is in the range 1MHz to 150MHz
  si5351_divider = 900000000 / freq;  // Calculate the division ratio (in the range 6..1800). 900,000,000 is the maximum internal PLL freq: 900MHz
  if (si5351_divider % 2) si5351_divider--;   // Ensure an even integer division ratio
  if( (si5351_divider * (freq-5000) / SI_XTAL_FREQ) != (si5351_divider * (freq+5000) / SI_XTAL_FREQ) ) si5351_divider-=2;  // Test if si5351_multiplier remains same for freq deviation +/- 5kHz, if not use different si5351_divider to make same
  uint32_t pll_freq = si5351_divider * freq; // Calculate the pll_freq: the si5351_divider * desired output freq

  si5351_mult = pll_freq / SI_XTAL_FREQ;  // Determine the si5351_multiplier to get to the required pll_freq (in the range 15..90)
  uint64_t l = pll_freq % SI_XTAL_FREQ;  // // distance of pll_freq with si5351_multiple of xtal frequency. It has three parts:
  l <<= 20; l--;        // l *= 1048575;  
  l /= SI_XTAL_FREQ;       // normalize
  uint32_t num = l;     // the actual si5351_multiplier is  si5351_mult + num / denom      
  //num and denom are the fractional parts, the numerator and denominator each is 20 bits (range 0..1048575)
  const uint32_t denom = 0xFFFFF;   // For simplicity we set the denominator to the maximum 1048575
  // Set up specified PLL with si5351_mult, num and denom: si5351_mult is 15..90, num is 0..1,048,575 (0xFFFFF), denom is 0..1,048,575 (0xFFFFF)
  uint32_t term = num * 128 / denom;                // 128.0 * (num / denom)
  uint32_t P1 = 128 * si5351_mult + term - 512;
  uint32_t P2 = 128 * num - denom * term; 
  uint32_t P3 = denom;
  si5351_pll_data[0] = 0xFF;
  si5351_pll_data[1] = 0xFF;
  si5351_pll_data[2] = (P1 >> 14) & 0x0C;
  si5351_pll_data[3] = P1 >> 8;
  si5351_pll_data[4] = P1;
  si5351_pll_data[5] = 0xF0 | ((P2 & 0x000F0000) >> 16);
  si5351_pll_data[6] = P2 >> 8;
  si5351_pll_data[7] = P2;
  si5351_SendPLLBRegister();
  si5351_SendRegister(SI_CLK_OE, 0b11111000); //CLK2_EN=en, CLK1_EN, CLK0_EN
}

volatile bool usb = true;
volatile bool change = true;
volatile int32_t freq = 7074000;

void set_tx(bool en)
{
  si5351_SendRegister(SI_CLK_OE, en ? 0b11111011 : 0b11111100); //CLK2_EN=en, CLK1_EN=CLK0_EN=/en
  digitalWrite(RX, en ? LOW : HIGH);
}

inline int16_t arctan2(int8_t q, int8_t i)
{
  int16_t phase;
  int8_t abs_q = abs(q);
  if(i < 0)
    phase = 135 - 45 * (i + abs(q)) / ((abs(q) - i) == 0 ? 1 : (abs(q) - i));
  else
    phase =  45 - 45 * (i - abs(q)) / ((i + abs(q)) == 0 ? 1 : (i + abs(q)));
  return (q < 0) ? -phase : phase; // negate if in quad III or IV
}

volatile uint8_t vox_hangtime;
volatile bool vox_enable = false;
#define VOX_THRESHOLD 4
inline void vox(int8_t amp)
{
  if(vox_enable){
    bool vox_trigger = (amp > VOX_THRESHOLD);  // vox_enable sensitivity
    switch(vox_hangtime){
    case 0: // step 1: VOX triggered -> TX on, minimum initial hangtime
      if(vox_trigger){
        vox_hangtime = 2;
        set_tx(true);
      }
      return 0;
    case 1: // step 3: VOX not triggered recently (hangtime exceeded) -> TX off
      vox_hangtime--;
      set_tx(false);
      return 0;
    default: // step 2: still VOX triggered -> reset hangtime
      vox_hangtime = (vox_trigger) ? 255 : vox_hangtime - 1;  // 255/F_SAMP = 50ms ( =~ 15 300Hz periods)
      break;
    }
  }
}

volatile uint8_t drive = 4;
void set_pwm_pa(uint8_t amp)
{
  // Based on amplitude set the PA voltage through PWM. The following voltages at L4 have been measured for various PWM values: 0x00 (0.48V), 0x10 (0.50V), 0x1A (0.54V), 0x1D (0.66V), 0x20 (3.4-7V), 0x30 (6.5-11V), 0x40 (9.96V), 0x60 (11V), 0x80 (11.5V), 0xFF (11.8V)
  #define KEY_OUT_PWM_MIN  0x1D   // The PWM (threshold) value where the voltage over L4 just start rising ~0.6V
  #define KEY_OUT_PWM_MAX  0x60   // The PWM value where the maximum voltage over L4 is approximated ~11V
  if(drive == 0)  // constant-carrier SSB
    OCR1BL = 0xFF;
  else
    OCR1BL = amp / (0xFF/(KEY_OUT_PWM_MAX - KEY_OUT_PWM_MIN)) + KEY_OUT_PWM_MIN;  // Set PWM amplitude at OC1B (KEY_OUT) in the working range of the PA supply (1 to 11V)  --> the current implmentation is quite crude: it would be nice if we can linearize this via a calibration technique
}

#define F_SAMP 4400
#define MAX_DP  360 // the occupied SSB bandwidth can be further reduced by restricting the maximum phase change (set MAX_DP to 180).

inline int16_t ssb(uint8_t in)
{
   static uint8_t prev_in;

   // [i, q] = hilbert(in);
   int8_t i, q;
   static int8_t v[25];
   uint8_t j;
   for (j = 0; j != 25; j++)
     v[j] = v[j+1];
   v[25] = in - prev_in;  // differential of input signal
   prev_in = in;
   i = v[13];
   // Hilbert transformation (90 degrees phase shift)
   q = ( (v[0] - v[26] + v[2] - v[24]) >> 5) + ((v[4] - v[22]) >> 4) + ((v[6] - v[20] + v[8] - v[18]) >> 3) + ((v[10] - v[16]) >> 2) + ((v[12] - v[14]) );
   
   uint16_t amp = (abs(i) + abs(q));    // approximation of: amp = sqrt(i*i + q*q);

   amp *= drive;  //scale so that input amplitude is full scale; drive=4 seems a good drive
   amp = (amp > 255) ? 255 : amp; // clip

   set_pwm_pa(amp);

   vox(v[1]+v[6]+v[13]+v[17]+v[24]);
   //i+=1; // make phasor off-center redcudes noise: may improve quality
   //q+=1;
   
   static int16_t prev_phase;
   int16_t phase = arctan2(q, i);
   int16_t dp = phase - prev_phase;  // phase difference and restriction
   prev_phase = phase;

   if(dp < 0) dp = dp + 360;  // prevent negative frequencies to reduce spur on other sideband
   if(dp > MAX_DP){ // dp should be less than 180 in order to keep frequencies below F_SAMP/2
     prev_phase = phase - (dp - MAX_DP);  // substract restdp
     dp = MAX_DP;
   }
   if(usb)
     return dp * ( F_SAMP/360); // calculate frequency-difference based on phase-difference
   else
     return dp * (-F_SAMP/360);
}

volatile uint16_t numSamples = 0;

#define MIC_ATTEN  0

/* This is the ADC ISR, issued with sample-rate via timer1 compb interrupt.
 * It performs in real-time the ADC sampling, calculation of SSB phase-differences, 
 * calculation of SI5351 frequency registers and send the registers to SI5351 over I2C.
 */
ISR(ADC_vect)                   // ADC conversion interrupt
{
  si5351_SendPLLBRegister();    // submit frequency registers to SI5351 over ~840kbit/s I2C
  uint8_t low  = ADCL;          // ADC sample 10-bits analog input, first ADCL, then ADCH
  uint8_t high = ADCH;
  uint16_t adc = (high << 8) | low;
  int16_t df = ssb(adc >> MIC_ATTEN);  // convert analog input into phase-shifts (carrier out by periodic frequency shifts)
  si5351_freq_calc_fast(df);         // calculate SI5351 registers based on frequency shift and carrier frequency
  //OCR1BL = amp;               // Set PWM amplitude at OC1B (KEY_OUT)
  numSamples++;
}

ISR (TIMER0_COMPB_vect)  // Timer0 interrupt
{
  ADCSRA |= (1 << ADSC); // start ADC conversion (triggers ADC interrupt)
}

uint8_t old_TCCR0A;
uint8_t old_TCCR0B;
uint8_t old_TCNT0;
uint8_t old_TIMSK0;

void adc_start()
{
  DIDR0 |= (1 << 2);      // disable digital input
  ADCSRA = 0;             // clear ADCSRA register
  ADCSRB = 0;             // clear ADCSRB register
  ADMUX = 0;              // clear ADMUX register
  ADMUX |= (2 & 0x0f);    // set analog input pin
  ADMUX |= (1 << REFS1) | (1 << REFS0);  // set reference voltage Internal 1.1V  (much more gain compared to AREF reference )
  ADCSRA |= (1 << ADPS1) | (1 << ADPS0);    // ADPS=011: 8 prescaler for 153.8 KHz;  sampling rate is [ADC clock] / [prescaler] / [conversion clock cycles]  for Arduino Uno ADC clock is 20 MHz and a conversion takes 13 clock cycles: ADPS=011: 8 prescaler for 153.8 KHz, ADPS=100: 16 prescaler for 76.9 KHz; ADPS=101: 32 prescaler for 38.5 KHz; ADPS=110: 64 prescaler for 19.2kHz; // ADPS=111: 128 prescaler for 9.6kHz
  ADCSRA |= (1 << ADIE);  // enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN);  // enable ADC
  ADCSRA |= (1 << ADSC);  // start ADC measurements

  // backup Timer 0, is used by delay(), micros(), etc.
  old_TCCR0A = TCCR0A;
  old_TCCR0B = TCCR0B;
  old_TCNT0 = TCNT0;
  old_TIMSK0 = TIMSK0;

  // Timer 0: interrupt mode
  TCCR0A = 0;
  TCCR0B = 0;
  TCNT0 = 0;
  TCCR0A |= (1 << WGM01); // turn on CTC mode
  TCCR0B |= (1 << CS01) | (1 << CS00);  // Set CS01 and CS00 bits for 64 prescaler
  TIMSK0 |= (1 << OCIE0B);  // enable timer compare interrupt TIMER0_COMPB_vect
  uint8_t ocr = ((F_CPU / 64) / F_SAMP) - 1;
  OCR0A = ocr; // Set the value that you want to count to :   OCRn = [clock_speed / (Prescaler_value * Fs)] - 1  : sampling frequency 6.25 kHz

  // Timer 1: PWM mode
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A |= (1 << COM1B1); // Clear OC1A/OC1B on Compare Match when upcounting. Set OC1A/OC1B on Compare Match when downcounting.
  TCCR1B |= ((1 << CS10) | (1 << WGM13)); // WGM13: Mode 8 - PWM, Phase and Frequency Correct;  CS10: clkI/O/1 (No prescaling)
  ICR1H = 0x00;  // TOP. This sets the PWM frequency: PWM_FREQ=312.500kHz ICR=0x1freq bit_depth=5; PWM_FREQ=156.250kHz ICR=0x3freq bit_depth=6; PWM_FREQ=78.125kHz  ICR=0x7freq bit_depth=7; PWM_FREQ=39.250kHz  ICR=0xFF bit_depth=8
  ICR1L = 0xFF;  // Fpwm = F_CPU / (2 * Prescaler * TOP) :   PWM_FREQ = 39.25kHz, bit-depth=8
  OCR1BH = 0x00;
  OCR1BL = 0x00;  // PWM duty-cycle (span set by ICR). 
}

void adc_stop()
{
  // restore Timer 0, is shared with delay(), micros(), etc.
  TCCR0A = old_TCCR0A;
  TCCR0B = old_TCCR0B;
  TCNT0  = old_TCNT0;
  TIMSK0 = old_TIMSK0;
  
  // Stop Timer 0 interrupt
  //TIMSK0 &= ~(1 << OCIE0B);  // disable timer compare interrupt TIMER0_COMPB_vect
  //TCCR0A |= (1 << WGM00);  // for some reason WGM00 must be 1 in normal operation
  
  ADCSRA &= ~(1 << ADATE); // disable auto trigger
  ADCSRA &= ~(1 << ADIE);  // disable interrupts when measurement complete
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);    // 128 prescaler for 9.6kHz

  ADMUX = 0;              // clear ADMUX register
  ADMUX |= (1 << REFS0);  // restore reference voltage AREF (5V)
}

static uint8_t bandval = 4;
#define N_BANDS 12
uint32_t band[] = { 472000, 1840000, 3573000, 5357000, 7074000, 10136000, 14074000, 18100000, 21074000, 24915000, 28074000, 50313000, 70101000 };
enum step_enum { STEP_10M, STEP_1M, STEP_500k, STEP_100k, STEP_10k, STEP_1k, STEP_500, STEP_100, STEP_10, STEP_1 };
static uint32_t stepsize = STEP_1k;

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
  if(stepval < STEP_100) freq %= 1000;  // when tuned and stepsize > 100Hz then forget fine-tuning details
  freq += sign * stepval;
  freq = max(1, min(99999999, freq));
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
  if(stepsize < 0) stepsize += STEP_1;
  if(stepsize > STEP_1) stepsize -= STEP_1;
  stepsize_showcursor();
}

ISR(PCINT2_vect) {  // Interrupt on rotary encoder turn
  static uint8_t last_state;
  noInterrupts();
  uint8_t curr_state = (digitalRead(ROT_B) << 1) | digitalRead(ROT_A);
  switch((last_state << 2) | curr_state){ //transition
#ifdef ENCODER_ENHANCED_RESOLUTION // Enhance encoder from 24 to 96 steps/revolution: See: https://www.sdr-kits.net/documents/PA0KLT_Manual.pdf
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

  // pins
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

uint32_t sample_amp(uint8_t pin)
{
  uint16_t avg = 1024/2;
  uint32_t rms = 0;
  uint16_t i;
  for(i=0; i!=16; i++){
    uint16_t adc = analogRead(pin);
    avg = (avg + adc) / 2;
  }
  for(i=0; i!=128; i++){  // 128 overampling is 42dB gain, with 10-but ADC a total of 102dB DR
    uint16_t adc = analogRead(pin);
    avg = (avg + adc) / 2;
    rms += ((adc > avg) ? 1 : -1) * (adc - avg);
  }
  return rms;
}

void test_iq()
{
  si5351_prev_pll_freq = 0;  //enforce PLL reset
  si5351_freq(freq, 0, 90);  // RX in USB
  digitalWrite(SIG_OUT, true); // loopback on
  int16_t df;

  lcd.setCursor(0,0); lcd.print(blanks);
  for(df = 0; df < 3000; df += 100)
  {
    lcd.setCursor(0,0); lcd.print(df); lcd.print("Hz");
    si5351_freq_clk2(freq+df);  //TX in USB
    delay(300);
    float amp_usb = sample_amp(AUDIO2);
    si5351_freq_clk2(freq-df);   //TX in LSB
    delay(600);
    float amp_lsb = sample_amp(AUDIO2);
    lcd.setCursor(7,0); lcd.print(20.0*log10(amp_lsb/amp_usb)); lcd.print("dB"); lcd.print(blanks);
    wdt_reset();
  }
  lcd.print(blanks);
  
  digitalWrite(SIG_OUT, false); // loopback off
}

byte blackonwhite = true;
static byte canvas_bitmaps[2*4][9]; // 4x2 custom char 6x9 (actual visual char 5x8)
inline void canvas_init()
{ int i;
  lcd.clear();
  for(i = 0; i != 4*2; i++){
    if((i%4) == 0) lcd.setCursor(16/2 - 4/2, i/4);
    lcd.write(i+1);
  }
}
inline void canvas_clear()
{ int i,j;
  for(i = 0; i != 8; i++) for(j = 0; j != 8; j++) canvas_bitmaps[i][j] = (blackonwhite) ? 0b11111 : 0b00000; 
}
inline void canvas_plot(int8_t x, int8_t y){  //maps canvas (4*6-1 x 2*9-1) 23x17 to custom char
  if(x<0 || y<0 || x>(4*6-1) || y>(2*9-1)) return;
  x = x + 1;
  y = (2*9-1 - 1) - y;
  if(blackonwhite)
    canvas_bitmaps[4*(y/9) + (x/6)][(y%9)] &= ~(1 << ((6-1) - (x%6))); //black dot
  else
    canvas_bitmaps[4*(y/9) + (x/6)][(y%9)] |=   1 << ((6-1) - (x%6));  //white dot
}

#include <float.h>
inline void canvas_graph(float arr[])
{
  int i;
  float arrmax = -FLT_MAX;
  float arrmin = FLT_MAX;
  for(i=0;i!=(4*6);i++){
    arrmax = max(arrmax, arr[i]);
    arrmin = min(arrmin, arr[i]);
  }
  float scale = (float)(arrmax-arrmin) / (float)(2*9);
  for(i=0;i!=(4*6);i++){ //scale down y and canvas_plot
    canvas_plot( i, (float)(arr[i] - arrmin) / scale );
  }
  lcd.setCursor(0,0); lcd.print("      ");
  lcd.setCursor(0,0); lcd.print(arrmax-arrmin);
  lcd.setCursor(0,1); lcd.print("      ");
  lcd.setCursor(0,1); lcd.print(arrmin);
}
inline void canvas_blit()
{ int i;
  for(i = 0; i != 8; i++) lcd.createChar(i + 1, canvas_bitmaps[i]);
}
void test_scope()
{
  digitalWrite(SIG_OUT, true); // loopback on
  si5351_prev_pll_freq = 0;  //enforce PLL reset
  si5351_freq(freq, 0, 90);  // RX in USB
  si5351_freq_clk2(freq + 1024);
  canvas_init();  

  for(;;)
  {
    uint8_t x;
    float arr[4*6];
    canvas_clear(); 
    //
    uint32_t fx = freq;    
    for(x=0; x!=(4*6); x++){
      si5351_prev_pll_freq = 0;  //enforce PLL reset
      si5351_freq(fx, 0, 90);  // RX in USB
      si5351_freq_clk2(fx + 1024);
      delay(100);
      arr[x] = 20.0*log10((float)sample_amp(AUDIO1)) - 110.0;
      fx += 100000;
      wdt_reset();
    }
    //for(;(micros() % 1024) > 9;);  for(x=0; x<(4*6); x++) arr[x] = analogRead(AUDIO1); //simple scope
    canvas_graph(arr);
    canvas_blit();
    wdt_reset();
  }
  
  digitalWrite(SIG_OUT, false); // loopback off
}

void test_samplerate()
{
  noInterrupts();     // report actual sample-rate: measure number of samples/s
  long n0 = numSamples;
  interrupts();
  delay(1000);
  noInterrupts();
  long n1 = numSamples;
  i2c_suspend(); //recover from I2C before doing any LCD operation
  lcd.setCursor(0,0); lcd.print(n1 - n0);  // do not forget to correct this value with *1.25 because actual F_CPU=20M on QCX
  lcd.print(" ");
  i2c_resume();  //prepare for I2C after LCD operation
  interrupts();
}

void smeter()
{
  float dbm = 20.0 * log10(sample_amp(AUDIO1)) - 102.0;
  lcd.setCursor(10,0); lcd.print((int8_t)dbm); lcd.print("dBm ");
}

void setup()
{
  wdt_enable(WDTO_2S);  // Enable watchdog, resolves QCX startup issue

  lcd.begin(16, 2);
  lcd.createChar(1, font_run);
  lcd.setCursor(0,0); lcd.print("QCX-SSB R"); lcd.print(VERSION); lcd.print(blanks);

  i2c_init();

  encoder_setup();

  qcx_setup();
  
  //PCICR |= (1 << PCIE0);
  //PCMSK0 |= (1 << PCINT5) | (1 << PCINT4) | (1 << PCINT3);
  //interrupts();
  
  delay(1000);
  lcd.setCursor(7,0); lcd.print("\001"); lcd.print(blanks); // display initialization complete

  numSamples = 0; // DO NOT remove this volatile variable
}

void loop()
{
  delay(100);
  
  smeter();

  if(!digitalRead(DIT)){
    lcd.setCursor(15,1); lcd.print("T");
    set_tx(true);
    adc_start();
    for(;!digitalRead(DIT);){ //until depressed
      //test_samplerate();
      wdt_reset();
    }
    adc_stop();
    set_tx(false);
    lcd.setCursor(15,1); lcd.print("R");
  }
  if(digitalRead(BUTTONS)){  // Left-/Right-/Rotary-button
    uint16_t val = analogRead(BUTTONS);
    bool longpress = false;
    bool doubleclick = false;
    int32_t t0 = millis();
    for(;digitalRead(BUTTONS) && !longpress;){ //until depressed or long-press
      longpress = ((millis() - t0) > 300);
      wdt_reset();
    }
    delay(10); //debounce
    for(;!longpress && ((millis() - t0) < 500) && !doubleclick;){ //until 2nd press or timeout
      doubleclick = digitalRead(BUTTONS);
      wdt_reset();
    }
    for(;digitalRead(BUTTONS);) wdt_reset();  //until depressed
    if(val < 852) {        // Left-button   ADC=772
      if(doubleclick){
        return;
      }
      if(longpress){
        lcd.setCursor(15,1); lcd.print("V");
        vox_enable = true;
        set_tx(true);
        adc_start();
        for(;!digitalRead(BUTTONS);) wdt_reset();  //until 2nd press
        adc_stop();
        set_tx(false);
        vox_enable = false;
        lcd.setCursor(15,1); lcd.print("R");
        for(;digitalRead(BUTTONS);) wdt_reset();  //until depressed
        delay(100);
        return;
      }
      usb = !usb;
      si5351_prev_pll_freq = 0;  //enforce PLL reset
      change=true;
    } else if(val < 978) { // Right-button  ADC=933
      if(doubleclick){
        test_scope();
        return;
      }
      if(longpress){
        test_iq();
        return;
      }
      if(drive == 0) drive = 2;
      else drive += 2;
      if(drive>16) drive = 0;
      lcd.setCursor(0,1); lcd.print("Drive "); lcd.print(drive); lcd.print(blanks);
    } else {               // Rotory-button ADC=1023 
      if(doubleclick){
        delay(100);
        bandval++;
        if(bandval > N_BANDS) bandval=0;
        freq=band[bandval];
        stepsize = STEP_1k;
        change=true;
        return;
      }
      if(longpress){
        stepsize_change(-1);
        return;
      }
      stepsize_change(+1);
    }     
  }
  if(change){
    change = false;
    uint32_t n = freq / 1000000;  //lcd.print(f) with commas
    uint32_t n2 = freq % 1000000;
    uint32_t scale = 1000000;
    char buf[16];  
    sprintf(buf, "%2u", n); lcd.setCursor(0,1); lcd.print(buf);
    while (scale != 1) {
        scale /= 1000;
        n = n2 / scale;
        n2 = n2  % scale;
        sprintf(buf, ",%03u", n); lcd.print(buf);
    }
    lcd.print((usb) ? " USB" : " LSB");
    
    if(usb)
      si5351_freq(freq, 0, 90);
    else
      si5351_freq(freq, 90, 0);    
  }
  wdt_reset();
  stepsize_showcursor();
}
