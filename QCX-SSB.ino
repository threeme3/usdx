//  QCX-SSB.ino - https://github.com/threeme3/QCX-SSB
//
//  Copyright 2019, 2020   Guido PE1NNZ <pe1nnz@amsat.org>
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#define VERSION   "1.02a"

// QCX pin defintions
#define LCD_D4  0         //PD0    (pin 2)
#define LCD_D5  1         //PD1    (pin 3)
#define LCD_D6  2         //PD2    (pin 4)
#define LCD_D7  3         //PD3    (pin 5)
#define LCD_EN  4         //PD4    (pin 6)
#define FREQCNT 5         //PD5    (pin 11)
#define ROT_A   6         //PD6    (pin 12)
#define ROT_B   7         //PD7    (pin 13)
#define RX      8         //PB0    (pin 14)
#define SIDETONE 9        //PB1    (pin 15)
#define KEY_OUT 10        //PB2    (pin 16)
#define SIG_OUT 11        //PB3    (pin 17)
#define DAH     12        //PB4    (pin 18)
#define DIT     13        //PB5    (pin 19)
#define AUDIO1  14        //PC0/A0 (pin 23)
#define AUDIO2  15        //PC1/A1 (pin 24)
#define DVM     16        //PC2/A2 (pin 25)
#define BUTTONS 17        //PC3/A3 (pin 26)
#define LCD_RS  18        //PC4    (pin 27)
#define SDA     18        //PC4    (pin 27)
#define SCL     19        //PC5    (pin 28)
//#define NTX	11          //PB3    (pin 17)  - experimental: LOW on TX

/*
// UCX installation: On blank chip, use (standard Arduino Uno) fuse settings (E:FD, H:DE, L:FF), and use customized Optiboot bootloader for 20MHz clock, then upload via serial interface (with RX, TX and DTR lines connected to pin 1, 2, 3 respectively)
// UCX pin defintions
+#define SDA     3         //PD3    (pin 5)
+#define SCL     4         //PD4    (pin 6)
+#define ROT_A   6         //PD6    (pin 12)
+#define ROT_B   7         //PD7    (pin 13)
+#define RX      8         //PB0    (pin 14)
+#define SIDETONE 9        //PB1    (pin 15)
+#define KEY_OUT 10        //PB2    (pin 16)
+#define NTX     11        //PB3    (pin 17)
+#define DAH     12        //PB4    (pin 18)
+#define DIT     13        //PB5    (pin 19)
+#define AUDIO1  14        //PC0/A0 (pin 23)
+#define AUDIO2  15        //PC1/A1 (pin 24)
+#define DVM     16        //PC2/A2 (pin 25)
+#define BUTTONS 17        //PC3/A3 (pin 26)
// In addition set:
#define OLED  1
#define ONEBUTTON  1
#undef DEBUG
adjust I2C and I2C_ ports, 
ssb_cap=1; dsp_cap=2;
#define _DELAY() for(uint8_t i = 0; i != 5; i++) asm("nop");
#define F_XTAL 20004000
#define F_CPU F_XTAL
experimentally: #define AUTO_ADC_BIAS 1
*/

#include <avr/sleep.h>
#include <avr/wdt.h>

//FUSES = { .low = 0xF7, .high = 0xD1, .extended = 0xFD };   // Fuse settings should be E=FD H=D1 L=F7, at programming.

class LCD : public Print {  // inspired by: http://www.technoblogy.com/show?2BET
public:  // LCD1602 display in 4-bit mode, RS is pull-up and kept low when idle to prevent potential display RFI via RS line
  #define _dn  0      // PD0 to PD3 connect to D4 to D7 on the display
  #define _en  4      // PC4 - MUST have pull-up resistor
  #define _rs  4      // PC4 - MUST have pull-up resistor
  #define LCD_RS_HI() DDRC &= ~(1 << _rs);         // RS high (pull-up)
  #define LCD_RS_LO() DDRC |= 1 << _rs;            // RS low (pull-down)
  #define LCD_EN_LO() PORTD &= ~(1 << _en);        // EN low
  #define LCD_EN_HI() PORTD |= (1 << _en);         // EN high
  #define LCD_PREP_NIBBLE(b) (PORTD & ~(0xf << _dn)) | (b) << _dn | 1 << _en // Send data and enable high
  void begin(uint8_t x = 0, uint8_t y = 0){        // Send command , make sure at least 40ms after power-up before sending commands
    bool reinit = (x == 0) && (y == 0);
    DDRD |= 0xf << _dn | 1 << _en;                 // Make data, EN and RS pins outputs
    PORTC &= ~(1 << _rs);                          // Set RS low in case to support pull-down when DDRC is output
    //delayMicroseconds(50000);                      // *
    LCD_RS_LO(); LCD_EN_LO();
    cmd(0x03);                                     // Ensures display is in 8-bit mode
    //delayMicroseconds(4500); cmd(0x03); delayMicroseconds(4500); cmd(0x03); delayMicroseconds(150); // * Ensures display is in 8-bit mode
    cmd(0x02);                                     // Puts display in 4-bit mode
    //cmd(0x28);                                     // * Function set: 2-line, 5x8 
    cmd(0x0c);                                     // Display on
    if(reinit) return;
    cmd(0x01);                                     // Clear display
    delay(3);                                      // Allow to execute Clear on display [https://www.sparkfun.com/datasheets/LCD/HD44780.pdf, p.49, p58]
    //cmd(0x06);                                     // * Entrymode: left, shift-dec
  }
  void nib(uint8_t b){                             // Send four bit nibble to display
    PORTD = LCD_PREP_NIBBLE(b);                    // Send data and enable high
    asm("nop");                                    // Enable high pulse width must be at least 230ns high, data-setup time 80ns
    LCD_EN_LO();
    delayMicroseconds(52);                         // Execution time
  }
  void cmd(uint8_t b){ nib(b >> 4); nib(b & 0xf); }// Write command: send nibbles while RS low
  size_t write(uint8_t b){                         // Write data:    send nibbles while RS high
    //LCD_EN_HI();                                   // Complete Enable cycle must be at least 500ns (so start early)
    uint8_t nibh = LCD_PREP_NIBBLE(b >>  4);       // Prepare high nibble data and enable high
    PORTD = nibh;                                  // Send high nibble data and enable high
    uint8_t nibl = LCD_PREP_NIBBLE(b & 0xf);       // Prepare low nibble data and enable high
    //asm("nop");                                    // Enable high pulse width must be at least 230ns high, data-setup time 80ns; ATMEGA clock-cycle is 50ns (so at least 5 cycles)
    LCD_RS_HI();
    LCD_EN_LO();
    PORTD = nibl;                                  // Send low nibble data and enable high
    LCD_RS_LO();
    //asm("nop"); asm("nop");                        // Complete Enable cycle must be at least 500ns
    //PORTD = nibl;                                  // Send low nibble data and enable high
    //asm("nop");                                    // Enable high pulse width must be at least 230ns high, data-setup time 80ns; ATMEGA clock-cycle is 50ns (so at least 5 cycles)
    LCD_RS_HI();
    LCD_EN_LO();
    LCD_RS_LO();
    PORTD |= 0x02;                                 // To support serial-interface keep LCD_D5 high, so that DVM is not pulled-down via D
    delayMicroseconds(52);                         // Execution time  (37+4)*1.25 us
    return 1;
  }
  void setCursor(uint8_t x, uint8_t y){ cmd(0x80 | (x + y * 0x40)); }
  void cursor(){ cmd(0x0e); }
  void noCursor(){ cmd(0x0c); }
  void noDisplay(){ cmd(0x08); }
  void createChar(uint8_t l, uint8_t glyph[]){ cmd(0x40 | ((l & 0x7) << 3)); for(int i = 0; i != 8; i++) write(glyph[i]); }
};

/*
class LCD : public Print {  // inspired by: http://www.technoblogy.com/show?2BET
public:  // LCD1602 display in 4-bit mode, RS is pull-up and kept low when idle to prevent potential display RFI via RS line
  #define _dn  0      // PD0 to PD3 connect to D4 to D7 on the display
  #define _en  4      // PC4 - MUST have pull-up resistor
  #define _rs  4      // PC4 - MUST have pull-up resistor
  #define LCD_RS_HI() DDRC &= ~(1 << _rs);         // RS high (pull-up)
  #define LCD_RS_LO() DDRC |= 1 << _rs;            // RS low (pull-down)
  #define LCD_EN_LO() PORTD &= ~(1 << _en);        // EN low
  #define LCD_PREP_NIBBLE(b) (PORTD & ~(0xf << _dn)) | (b) << _dn | 1 << _en // Send data and enable high
  void begin(uint8_t x, uint8_t y){                // Send command
    DDRD |= 0xf << _dn | 1 << _en;                 // Make data, EN and RS pins outputs
    PORTC &= ~(1 << _rs);                          // Set RS low in case to support pull-down when DDRC is output
    delayMicroseconds(50000);                      // * At least 40ms after power rises above 2.7V before sending commands
    LCD_RS_LO();
    cmd(0x33);                                     // Ensures display is in 8-bit mode
    cmd(0x32);                                     // Puts display in 4-bit mode
    cmd(0x0e);                                     // Display and cursor on
    cmd(0x01);                                     // Clear display
    delay(3);                                      // Allow to execute on display [https://www.sparkfun.com/datasheets/LCD/HD44780.pdf, p.49, p58]
  }
  void nib(uint8_t b){                             // Send four bit nibble to display
    PORTD = LCD_PREP_NIBBLE(b);                    // Send data and enable high
    delayMicroseconds(4);
    LCD_EN_LO();
    delayMicroseconds(60);                         // Execution time  (was: 37)
  }
  void cmd(uint8_t b){ nib(b >> 4); nib(b & 0xf); }// Write command: send nibbles while RS low
  size_t write(uint8_t b){                         // Write data:    send nibbles while RS high
    uint8_t nibh = LCD_PREP_NIBBLE(b >>  4);       // Prepare high nibble data and enable high
    uint8_t nibl = LCD_PREP_NIBBLE(b & 0xf);       // Prepare low nibble data and enable high
    PORTD = nibh;                                  // Send high nibble data and enable high
    LCD_RS_HI();
    LCD_EN_LO();
    PORTD = nibl;                                  // Send low nibble data and enable high
    LCD_RS_LO();
    LCD_RS_HI();
    LCD_EN_LO();
    LCD_RS_LO();
    delayMicroseconds(41);                         // Execution time
    PORTD |= 0x02;                                 // To support serial-interface keep LCD_D5 high, so that DVM is not pulled-down via D
    return 1;
  }
  void setCursor(uint8_t x, uint8_t y){ cmd(0x80 | (x + y * 0x40)); }
  void cursor(){ cmd(0x0e); }
  void noCursor(){ cmd(0x0c); }
  void noDisplay(){ cmd(0x08); }
  void createChar(uint8_t l, uint8_t glyph[]){ cmd(0x40 | ((l & 0x7) << 3)); for(int i = 0; i != 8; i++) write(glyph[i]); }
};
*/

#include <LiquidCrystal.h>
class LCD_ : public LiquidCrystal {
public: // QCXLiquidCrystal extends LiquidCrystal library for pull-up driven LCD_RS, as done on QCX. LCD_RS needs to be set to LOW in advance of calling any operation.
  //LCD_(uint8_t rs = LCD_RS, uint8_t en = LCD_EN, uint8_t d4 = LCD_D4, uint8_t d5, = LCD_D5 uint8_t d6 = LCD_D6, uint8_t d7 = LCD_D7) : LiquidCrystal(rs, en, d4, d5, d6, d7){ };
  LCD_() : LiquidCrystal(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7){ };
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

// I2C class used by SSD1306 driver; you may connect a SSD1306 (128x32) display on LCD header pins: 1 (GND); 2 (VCC); 13 (SDA); 14 (SCL)
class I2C_ {  // direct port I/O (disregards/does-not-need pull-ups)
public:
  #define _DELAY() for(uint8_t i = 0; i != 4; i++) asm("nop"); // 4=731kb/s
  #define _I2C_SDA (1<<2) // PD2
  #define _I2C_SCL (1<<3) // PD3
  #define _I2C_INIT() _I2C_SDA_HI(); _I2C_SCL_HI(); DDRD |= (_I2C_SDA | _I2C_SCL);
  #define _I2C_SDA_HI() PORTD |=  _I2C_SDA; _DELAY();
  #define _I2C_SDA_LO() PORTD &= ~_I2C_SDA; _DELAY();
  #define _I2C_SCL_HI() PORTD |=  _I2C_SCL; _DELAY();
  #define _I2C_SCL_LO() PORTD &= ~_I2C_SCL; _DELAY();
  #define _I2C_START() _I2C_SDA_LO(); _I2C_SCL_LO(); _I2C_SDA_HI();
  #define _I2C_STOP()  _I2C_SCL_HI(); _I2C_SDA_HI();
  #define _I2C_SUSPEND() //_I2C_SDA_LO(); // SDA_LO to allow re-use as output port
  #define _SendBit(data, bit) \
    if(data & 1 << bit){ \
      _I2C_SDA_HI();  \
    } else {         \
      _I2C_SDA_LO();  \
    }                \
    _I2C_SCL_HI();    \
    _I2C_SCL_LO();
  inline void start(){ _I2C_INIT(); _I2C_START(); };
  inline void stop() { _I2C_STOP(); _I2C_SUSPEND(); }; 
  inline void SendByte(uint8_t data){
    _SendBit(data, 7);
    _SendBit(data, 6);
    _SendBit(data, 5);
    _SendBit(data, 4);
    _SendBit(data, 3);
    _SendBit(data, 2);
    _SendBit(data, 1);
    _SendBit(data, 0);
    _I2C_SDA_HI();  // recv ACK
    _DELAY(); //
    _I2C_SCL_HI();
    _I2C_SCL_LO();
  }
  void SendRegister(uint8_t addr, uint8_t* data, uint8_t n){
    start();
    SendByte(addr << 1);
    while(n--) SendByte(*data++);
    stop();      
  }
  //void SendRegister(uint8_t addr, uint8_t val){ SendRegister(addr, &val, 1); }

  void begin(){};
  void beginTransmission(uint8_t addr){ start(); SendByte(addr << 1);  };
  bool write(uint8_t byte){ SendByte(byte); return 1; };
  uint8_t endTransmission(){ stop(); return 0; };
};
I2C_ Wire;
//#include <Wire.h>

/* // 6x8 technoblogy font
const uint8_t font[]PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
   0x00, 0x00, 0x5F, 0x00, 0x00, 0x00, 
   0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 
   0x14, 0x7F, 0x14, 0x7F, 0x14, 0x00, 
   0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x00, 
   0x23, 0x13, 0x08, 0x64, 0x62, 0x00, 
   0x36, 0x49, 0x56, 0x20, 0x50, 0x00, 
   0x00, 0x08, 0x07, 0x03, 0x00, 0x00, 
   0x00, 0x1C, 0x22, 0x41, 0x00, 0x00, 
   0x00, 0x41, 0x22, 0x1C, 0x00, 0x00, 
   0x2A, 0x1C, 0x7F, 0x1C, 0x2A, 0x00, 
   0x08, 0x08, 0x3E, 0x08, 0x08, 0x00, 
   0x00, 0x80, 0x70, 0x30, 0x00, 0x00, 
   0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 
   0x00, 0x00, 0x60, 0x60, 0x00, 0x00, 
   0x20, 0x10, 0x08, 0x04, 0x02, 0x00, 
   0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00, 
   0x00, 0x42, 0x7F, 0x40, 0x00, 0x00, 
   0x72, 0x49, 0x49, 0x49, 0x46, 0x00, 
   0x21, 0x41, 0x49, 0x4D, 0x33, 0x00, 
   0x18, 0x14, 0x12, 0x7F, 0x10, 0x00, 
   0x27, 0x45, 0x45, 0x45, 0x39, 0x00, 
   0x3C, 0x4A, 0x49, 0x49, 0x31, 0x00, 
   0x41, 0x21, 0x11, 0x09, 0x07, 0x00, 
   0x36, 0x49, 0x49, 0x49, 0x36, 0x00, 
   0x46, 0x49, 0x49, 0x29, 0x1E, 0x00, 
   0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 
   0x00, 0x40, 0x34, 0x00, 0x00, 0x00, 
   0x00, 0x08, 0x14, 0x22, 0x41, 0x00, 
   0x14, 0x14, 0x14, 0x14, 0x14, 0x00, 
   0x00, 0x41, 0x22, 0x14, 0x08, 0x00, 
   0x02, 0x01, 0x59, 0x09, 0x06, 0x00, 
   0x3E, 0x41, 0x5D, 0x59, 0x4E, 0x00, 
   0x7C, 0x12, 0x11, 0x12, 0x7C, 0x00, 
   0x7F, 0x49, 0x49, 0x49, 0x36, 0x00, 
   0x3E, 0x41, 0x41, 0x41, 0x22, 0x00, 
   0x7F, 0x41, 0x41, 0x41, 0x3E, 0x00, 
   0x7F, 0x49, 0x49, 0x49, 0x41, 0x00, 
   0x7F, 0x09, 0x09, 0x09, 0x01, 0x00, 
   0x3E, 0x41, 0x41, 0x51, 0x73, 0x00, 
   0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00, 
   0x00, 0x41, 0x7F, 0x41, 0x00, 0x00, 
   0x20, 0x40, 0x41, 0x3F, 0x01, 0x00, 
   0x7F, 0x08, 0x14, 0x22, 0x41, 0x00, 
   0x7F, 0x40, 0x40, 0x40, 0x40, 0x00, 
   0x7F, 0x02, 0x1C, 0x02, 0x7F, 0x00, 
   0x7F, 0x04, 0x08, 0x10, 0x7F, 0x00, 
   0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00, 
   0x7F, 0x09, 0x09, 0x09, 0x06, 0x00, 
   0x3E, 0x41, 0x51, 0x21, 0x5E, 0x00, 
   0x7F, 0x09, 0x19, 0x29, 0x46, 0x00, 
   0x26, 0x49, 0x49, 0x49, 0x32, 0x00, 
   0x03, 0x01, 0x7F, 0x01, 0x03, 0x00, 
   0x3F, 0x40, 0x40, 0x40, 0x3F, 0x00, 
   0x1F, 0x20, 0x40, 0x20, 0x1F, 0x00, 
   0x3F, 0x40, 0x38, 0x40, 0x3F, 0x00, 
   0x63, 0x14, 0x08, 0x14, 0x63, 0x00, 
   0x03, 0x04, 0x78, 0x04, 0x03, 0x00, 
   0x61, 0x59, 0x49, 0x4D, 0x43, 0x00, 
   0x00, 0x7F, 0x41, 0x41, 0x41, 0x00, 
   0x02, 0x04, 0x08, 0x10, 0x20, 0x00, 
   0x00, 0x41, 0x41, 0x41, 0x7F, 0x00, 
   0x04, 0x02, 0x01, 0x02, 0x04, 0x00, 
   0x40, 0x40, 0x40, 0x40, 0x40, 0x00, 
   0x00, 0x03, 0x07, 0x08, 0x00, 0x00, 
   0x20, 0x54, 0x54, 0x78, 0x40, 0x00, 
   0x7F, 0x28, 0x44, 0x44, 0x38, 0x00, 
   0x38, 0x44, 0x44, 0x44, 0x28, 0x00, 
   0x38, 0x44, 0x44, 0x28, 0x7F, 0x00, 
   0x38, 0x54, 0x54, 0x54, 0x18, 0x00, 
   0x00, 0x08, 0x7E, 0x09, 0x02, 0x00, 
   0x18, 0xA4, 0xA4, 0x9C, 0x78, 0x00, 
   0x7F, 0x08, 0x04, 0x04, 0x78, 0x00, 
   0x00, 0x44, 0x7D, 0x40, 0x00, 0x00, 
   0x20, 0x40, 0x40, 0x3D, 0x00, 0x00, 
   0x7F, 0x10, 0x28, 0x44, 0x00, 0x00, 
   0x00, 0x41, 0x7F, 0x40, 0x00, 0x00, 
   0x7C, 0x04, 0x78, 0x04, 0x78, 0x00, 
   0x7C, 0x08, 0x04, 0x04, 0x78, 0x00, 
   0x38, 0x44, 0x44, 0x44, 0x38, 0x00, 
   0xFC, 0x18, 0x24, 0x24, 0x18, 0x00, 
   0x18, 0x24, 0x24, 0x18, 0xFC, 0x00, 
   0x7C, 0x08, 0x04, 0x04, 0x08, 0x00, 
   0x48, 0x54, 0x54, 0x54, 0x24, 0x00, 
   0x04, 0x04, 0x3F, 0x44, 0x24, 0x00, 
   0x3C, 0x40, 0x40, 0x20, 0x7C, 0x00, 
   0x1C, 0x20, 0x40, 0x20, 0x1C, 0x00, 
   0x3C, 0x40, 0x30, 0x40, 0x3C, 0x00, 
   0x44, 0x28, 0x10, 0x28, 0x44, 0x00, 
   0x4C, 0x90, 0x90, 0x90, 0x7C, 0x00, 
   0x44, 0x64, 0x54, 0x4C, 0x44, 0x00, 
   0x00, 0x08, 0x36, 0x41, 0x00, 0x00, 
   0x00, 0x00, 0x77, 0x00, 0x00, 0x00, 
   0x00, 0x41, 0x36, 0x08, 0x00, 0x00, 
   0x02, 0x01, 0x02, 0x04, 0x02, 0x00, 
   0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00 };

#define FONT_W 12//6
#define FONT_H 2
#define FONT_STRETCHV 1
#define FONT_STRETCHH 1//0
*/

// C64 real
const uint8_t font[]PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
   0x00, 0x00, 0x00, 0x4f, 0x4f, 0x00, 0x00, 0x00, 
   0x00, 0x07, 0x07, 0x00, 0x00, 0x07, 0x07, 0x00, 
   0x14, 0x7f, 0x7f, 0x14, 0x14, 0x7f, 0x7f, 0x14, 
   0x00, 0x24, 0x2e, 0x6b, 0x6b, 0x3a, 0x12, 0x00, 
   0x00, 0x63, 0x33, 0x18, 0x0c, 0x66, 0x63, 0x00, 
   0x00, 0x32, 0x7f, 0x4d, 0x4d, 0x77, 0x72, 0x50, 
   0x00, 0x00, 0x00, 0x04, 0x06, 0x03, 0x01, 0x00, 
   0x00, 0x00, 0x1c, 0x3e, 0x63, 0x41, 0x00, 0x00, 
   0x00, 0x00, 0x41, 0x63, 0x3e, 0x1c, 0x00, 0x00, 
   0x08, 0x2a, 0x3e, 0x1c, 0x1c, 0x3e, 0x2a, 0x08, 
   0x00, 0x08, 0x08, 0x3e, 0x3e, 0x08, 0x08, 0x00, 
   0x00, 0x00, 0x80, 0xe0, 0x60, 0x00, 0x00, 0x00, 
   0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 
   0x00, 0x00, 0x00, 0x60, 0x60, 0x00, 0x00, 0x00, 
   0x00, 0x40, 0x60, 0x30, 0x18, 0x0c, 0x06, 0x02, 
   0x00, 0x3e, 0x7f, 0x49, 0x45, 0x7f, 0x3e, 0x00, 
   0x00, 0x40, 0x44, 0x7f, 0x7f, 0x40, 0x40, 0x00, 
   0x00, 0x62, 0x73, 0x51, 0x49, 0x4f, 0x46, 0x00, 
   0x00, 0x22, 0x63, 0x49, 0x49, 0x7f, 0x36, 0x00, 
   0x00, 0x18, 0x18, 0x14, 0x16, 0x7f, 0x7f, 0x10, 
   0x00, 0x27, 0x67, 0x45, 0x45, 0x7d, 0x39, 0x00, 
   0x00, 0x3e, 0x7f, 0x49, 0x49, 0x7b, 0x32, 0x00, 
   0x00, 0x03, 0x03, 0x79, 0x7d, 0x07, 0x03, 0x00, 
   0x00, 0x36, 0x7f, 0x49, 0x49, 0x7f, 0x36, 0x00, 
   0x00, 0x26, 0x6f, 0x49, 0x49, 0x7f, 0x3e, 0x00, 
   0x00, 0x00, 0x00, 0x24, 0x24, 0x00, 0x00, 0x00, 
   0x00, 0x00, 0x80, 0xe4, 0x64, 0x00, 0x00, 0x00, 
   0x00, 0x08, 0x1c, 0x36, 0x63, 0x41, 0x41, 0x00, 
   0x00, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x00, 
   0x00, 0x41, 0x41, 0x63, 0x36, 0x1c, 0x08, 0x00, 
   0x00, 0x02, 0x03, 0x51, 0x59, 0x0f, 0x06, 0x00, 
   0x00, 0x3e, 0x7f, 0x41, 0x4d, 0x4f, 0x2e, 0x00, 
   0x00, 0x7c, 0x7e, 0x0b, 0x0b, 0x7e, 0x7c, 0x00, 
   0x00, 0x7f, 0x7f, 0x49, 0x49, 0x7f, 0x36, 0x00, 
   0x00, 0x3e, 0x7f, 0x41, 0x41, 0x63, 0x22, 0x00, 
   0x00, 0x7f, 0x7f, 0x41, 0x63, 0x3e, 0x1c, 0x00, 
   0x00, 0x7f, 0x7f, 0x49, 0x49, 0x41, 0x41, 0x00, 
   0x00, 0x7f, 0x7f, 0x09, 0x09, 0x01, 0x01, 0x00, 
   0x00, 0x3e, 0x7f, 0x41, 0x49, 0x7b, 0x3a, 0x00, 
   0x00, 0x7f, 0x7f, 0x08, 0x08, 0x7f, 0x7f, 0x00, 
   0x00, 0x00, 0x41, 0x7f, 0x7f, 0x41, 0x00, 0x00, 
   0x00, 0x20, 0x60, 0x41, 0x7f, 0x3f, 0x01, 0x00, 
   0x00, 0x7f, 0x7f, 0x1c, 0x36, 0x63, 0x41, 0x00, 
   0x00, 0x7f, 0x7f, 0x40, 0x40, 0x40, 0x40, 0x00, 
   0x00, 0x7f, 0x7f, 0x06, 0x0c, 0x06, 0x7f, 0x7f, 
   0x00, 0x7f, 0x7f, 0x0e, 0x1c, 0x7f, 0x7f, 0x00, 
   0x00, 0x3e, 0x7f, 0x41, 0x41, 0x7f, 0x3e, 0x00, 
   0x00, 0x7f, 0x7f, 0x09, 0x09, 0x0f, 0x06, 0x00, 
   0x00, 0x1e, 0x3f, 0x21, 0x61, 0x7f, 0x5e, 0x00, 
   0x00, 0x7f, 0x7f, 0x19, 0x39, 0x6f, 0x46, 0x00, 
   0x00, 0x26, 0x6f, 0x49, 0x49, 0x7b, 0x32, 0x00, 
   0x00, 0x01, 0x01, 0x7f, 0x7f, 0x01, 0x01, 0x00, 
   0x00, 0x3f, 0x7f, 0x40, 0x40, 0x7f, 0x3f, 0x00, 
   0x00, 0x1f, 0x3f, 0x60, 0x60, 0x3f, 0x1f, 0x00, 
   0x00, 0x7f, 0x7f, 0x30, 0x18, 0x30, 0x7f, 0x7f, 
   0x00, 0x63, 0x77, 0x1c, 0x1c, 0x77, 0x63, 0x00, 
   0x00, 0x07, 0x0f, 0x78, 0x78, 0x0f, 0x07, 0x00, 
   0x00, 0x61, 0x71, 0x59, 0x4d, 0x47, 0x43, 0x00, 
   0x00, 0x00, 0x7f, 0x7f, 0x41, 0x41, 0x00, 0x00, 
   0x00, 0x02, 0x06, 0x0c, 0x18, 0x30, 0x60, 0x40, 
   0x00, 0x00, 0x41, 0x41, 0x7f, 0x7f, 0x00, 0x00, 
   0x00, 0x08, 0x0c, 0xfe, 0xfe, 0x0c, 0x08, 0x00, 
   0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 
   0x00, 0x01, 0x03, 0x06, 0x04, 0x00, 0x00, 0x00, 
   0x00, 0x20, 0x74, 0x54, 0x54, 0x7c, 0x78, 0x00, 
   0x00, 0x7e, 0x7e, 0x48, 0x48, 0x78, 0x30, 0x00, 
   0x00, 0x38, 0x7c, 0x44, 0x44, 0x44, 0x00, 0x00, 
   0x00, 0x30, 0x78, 0x48, 0x48, 0x7e, 0x7e, 0x00, 
   0x00, 0x38, 0x7c, 0x54, 0x54, 0x5c, 0x18, 0x00, 
   0x00, 0x00, 0x08, 0x7c, 0x7e, 0x0a, 0x0a, 0x00, 
   0x00, 0x98, 0xbc, 0xa4, 0xa4, 0xfc, 0x7c, 0x00, 
   0x00, 0x7e, 0x7e, 0x08, 0x08, 0x78, 0x70, 0x00, 
   0x00, 0x00, 0x48, 0x7a, 0x7a, 0x40, 0x00, 0x00, 
   0x00, 0x00, 0x80, 0x80, 0x80, 0xfa, 0x7a, 0x00, 
   0x00, 0x7e, 0x7e, 0x10, 0x38, 0x68, 0x40, 0x00, 
   0x00, 0x00, 0x42, 0x7e, 0x7e, 0x40, 0x00, 0x00, 
   0x00, 0x7c, 0x7c, 0x18, 0x38, 0x1c, 0x7c, 0x78, 
   0x00, 0x7c, 0x7c, 0x04, 0x04, 0x7c, 0x78, 0x00, 
   0x00, 0x38, 0x7c, 0x44, 0x44, 0x7c, 0x38, 0x00, 
   0x00, 0xfc, 0xfc, 0x24, 0x24, 0x3c, 0x18, 0x00, 
   0x00, 0x18, 0x3c, 0x24, 0x24, 0xfc, 0xfc, 0x00, 
   0x00, 0x7c, 0x7c, 0x04, 0x04, 0x0c, 0x08, 0x00, 
   0x00, 0x48, 0x5c, 0x54, 0x54, 0x74, 0x24, 0x00, 
   0x00, 0x04, 0x04, 0x3e, 0x7e, 0x44, 0x44, 0x00, 
   0x00, 0x3c, 0x7c, 0x40, 0x40, 0x7c, 0x7c, 0x00, 
   0x00, 0x1c, 0x3c, 0x60, 0x60, 0x3c, 0x1c, 0x00, 
   0x00, 0x1c, 0x7c, 0x70, 0x38, 0x70, 0x7c, 0x1c, 
   0x00, 0x44, 0x6c, 0x38, 0x38, 0x6c, 0x44, 0x00, 
   0x00, 0x9c, 0xbc, 0xa0, 0xe0, 0x7c, 0x3c, 0x00, 
   0x00, 0x44, 0x64, 0x74, 0x5c, 0x4c, 0x44, 0x00, 
   0x00, 0x08, 0x3e, 0x77, 0x41, 0x41, 0x00, 0x00, 
   0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 
   0x00, 0x00, 0x41, 0x41, 0x77, 0x3e, 0x08, 0x00, 
   0x00, 0x04, 0x02, 0x02, 0x04, 0x04, 0x02, 0x00
   };

#define FONT_W 8
#define FONT_H 2
#define FONT_STRETCHV 1
#define FONT_STRETCHH 0

/*  //16x8 C-64 kind of
const uint8_t font[]PROGMEM = {
   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x20
   0x00,0x06,0x5F,0x5F,0x06,0x00,0x00,0x00, // 0x21
   0x00,0x07,0x07,0x00,0x07,0x07,0x00,0x00, // 0x22
   0x14,0x7F,0x7F,0x14,0x7F,0x7F,0x14,0x00, // 0x23
   0x24,0x2E,0x6B,0x6B,0x3A,0x12,0x00,0x00, // 0x24
   0x46,0x66,0x30,0x18,0x0C,0x66,0x62,0x00, // 0x25
   0x30,0x7A,0x4F,0x5D,0x37,0x7A,0x48,0x00, // 0x26
   0x04,0x07,0x03,0x00,0x00,0x00,0x00,0x00, // 0x27
   0x00,0x1C,0x3E,0x63,0x41,0x00,0x00,0x00, // 0x28
   0x00,0x41,0x63,0x3E,0x1C,0x00,0x00,0x00, // 0x29
   0x08,0x2A,0x3E,0x1C,0x1C,0x3E,0x2A,0x08, // 0x2A
   0x08,0x08,0x3E,0x3E,0x08,0x08,0x00,0x00, // 0x2B
   0x00,0xA0,0xE0,0x60,0x00,0x00,0x00,0x00, // 0x2C
   0x08,0x08,0x08,0x08,0x08,0x08,0x00,0x00, // 0x2D
   0x00,0x00,0x60,0x60,0x00,0x00,0x00,0x00, // 0x2E
   0x60,0x30,0x18,0x0C,0x06,0x03,0x01,0x00, // 0x2F
   0x3E,0x7F,0x59,0x4D,0x7F,0x3E,0x00,0x00, // 0x30
   0x42,0x42,0x7F,0x7F,0x40,0x40,0x00,0x00, // 0x31
   0x62,0x73,0x59,0x49,0x6F,0x66,0x00,0x00, // 0x32
   0x22,0x63,0x49,0x49,0x7F,0x36,0x00,0x00, // 0x33
   0x18,0x1C,0x16,0x13,0x7F,0x7F,0x10,0x00, // 0x34
   0x27,0x67,0x45,0x45,0x7D,0x39,0x00,0x00, // 0x35
   0x3C,0x7E,0x4B,0x49,0x79,0x30,0x00,0x00, // 0x36
   0x03,0x63,0x71,0x19,0x0F,0x07,0x00,0x00, // 0x37
   0x36,0x7F,0x49,0x49,0x7F,0x36,0x00,0x00, // 0x38
   0x06,0x4F,0x49,0x69,0x3F,0x1E,0x00,0x00, // 0x39
   0x00,0x00,0x6C,0x6C,0x00,0x00,0x00,0x00, // 0x3A
   0x00,0xA0,0xEC,0x6C,0x00,0x00,0x00,0x00, // 0x3B
   0x08,0x1C,0x36,0x63,0x41,0x00,0x00,0x00, // 0x3C
   0x14,0x14,0x14,0x14,0x14,0x14,0x00,0x00, // 0x3D
   0x00,0x41,0x63,0x36,0x1C,0x08,0x00,0x00, // 0x3E
   0x02,0x03,0x51,0x59,0x0F,0x06,0x00,0x00, // 0x3F
   0x3E,0x7F,0x41,0x5D,0x5D,0x1F,0x1E,0x00, // 0x40
   0x7C,0x7E,0x13,0x13,0x7E,0x7C,0x00,0x00, // 0x41
   0x41,0x7F,0x7F,0x49,0x49,0x7F,0x36,0x00, // 0x42
   0x1C,0x3E,0x63,0x41,0x41,0x63,0x22,0x00, // 0x43
   0x41,0x7F,0x7F,0x41,0x63,0x7F,0x1C,0x00, // 0x44
   0x41,0x7F,0x7F,0x49,0x5D,0x41,0x63,0x00, // 0x45
   0x41,0x7F,0x7F,0x49,0x1D,0x01,0x03,0x00, // 0x46
   0x1C,0x3E,0x63,0x41,0x51,0x73,0x72,0x00, // 0x47
   0x7F,0x7F,0x08,0x08,0x7F,0x7F,0x00,0x00, // 0x48
   0x00,0x41,0x7F,0x7F,0x41,0x00,0x00,0x00, // 0x49
   0x30,0x70,0x40,0x41,0x7F,0x3F,0x01,0x00, // 0x4A
   0x41,0x7F,0x7F,0x08,0x1C,0x77,0x63,0x00, // 0x4B
   0x41,0x7F,0x7F,0x41,0x40,0x60,0x70,0x00, // 0x4C
   0x7F,0x7F,0x06,0x0C,0x06,0x7F,0x7F,0x00, // 0x4D
   0x7F,0x7F,0x06,0x0C,0x18,0x7F,0x7F,0x00, // 0x4E
   0x1C,0x3E,0x63,0x41,0x63,0x3E,0x1C,0x00, // 0x4F
   0x41,0x7F,0x7F,0x49,0x09,0x0F,0x06,0x00, // 0x50
   0x1E,0x3F,0x21,0x71,0x7F,0x5E,0x00,0x00, // 0x51
   0x41,0x7F,0x7F,0x19,0x39,0x6F,0x46,0x00, // 0x52
   0x26,0x67,0x4D,0x59,0x7B,0x32,0x00,0x00, // 0x53
   0x03,0x41,0x7F,0x7F,0x41,0x03,0x00,0x00, // 0x54
   0x7F,0x7F,0x40,0x40,0x7F,0x7F,0x00,0x00, // 0x55
   0x1F,0x3F,0x60,0x60,0x3F,0x1F,0x00,0x00, // 0x56
   0x7F,0x7F,0x30,0x18,0x30,0x7F,0x7F,0x00, // 0x57
   0x63,0x77,0x1C,0x08,0x1C,0x77,0x63,0x00, // 0x58
   0x07,0x4F,0x78,0x78,0x4F,0x07,0x00,0x00, // 0x59
   0x67,0x73,0x59,0x4D,0x47,0x63,0x71,0x00, // 0x5A
   0x00,0x7F,0x7F,0x41,0x41,0x00,0x00,0x00, // 0x5B
   0x01,0x03,0x06,0x0C,0x18,0x30,0x60,0x00, // 0x5C
   0x00,0x41,0x41,0x7F,0x7F,0x00,0x00,0x00, // 0x5D
   0x08,0x0C,0x06,0x03,0x06,0x0C,0x08,0x00, // 0x5E
   0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80, // 0x5F
   0x00,0x00,0x03,0x07,0x04,0x00,0x00,0x00, // 0x60
   0x20,0x74,0x54,0x54,0x3C,0x78,0x40,0x00, // 0x61
   0x41,0x3F,0x7F,0x44,0x44,0x7C,0x38,0x00, // 0x62
   0x38,0x7C,0x44,0x44,0x6C,0x28,0x00,0x00, // 0x63
   0x30,0x78,0x48,0x49,0x3F,0x7F,0x40,0x00, // 0x64
   0x38,0x7C,0x54,0x54,0x5C,0x18,0x00,0x00, // 0x65
   0x48,0x7E,0x7F,0x49,0x03,0x02,0x00,0x00, // 0x66
   0x98,0xBC,0xA4,0xA4,0xF8,0x7C,0x04,0x00, // 0x67
   0x41,0x7F,0x7F,0x08,0x04,0x7C,0x78,0x00, // 0x68
   0x00,0x44,0x7D,0x7D,0x40,0x00,0x00,0x00, // 0x69
   0x40,0xC4,0x84,0xFD,0x7D,0x00,0x00,0x00, // 0x6A
   0x41,0x7F,0x7F,0x10,0x38,0x6C,0x44,0x00, // 0x6B
   0x00,0x41,0x7F,0x7F,0x40,0x00,0x00,0x00, // 0x6C
   0x7C,0x7C,0x0C,0x18,0x0C,0x7C,0x78,0x00, // 0x6D
   0x7C,0x7C,0x04,0x04,0x7C,0x78,0x00,0x00, // 0x6E
   0x38,0x7C,0x44,0x44,0x7C,0x38,0x00,0x00, // 0x6F
   0x84,0xFC,0xF8,0xA4,0x24,0x3C,0x18,0x00, // 0x70
   0x18,0x3C,0x24,0xA4,0xF8,0xFC,0x84,0x00, // 0x71
   0x44,0x7C,0x78,0x44,0x1C,0x18,0x00,0x00, // 0x72
   0x48,0x5C,0x54,0x54,0x74,0x24,0x00,0x00, // 0x73
   0x00,0x04,0x3E,0x7F,0x44,0x24,0x00,0x00, // 0x74
   0x3C,0x7C,0x40,0x40,0x3C,0x7C,0x40,0x00, // 0x75
   0x1C,0x3C,0x60,0x60,0x3C,0x1C,0x00,0x00, // 0x76
   0x3C,0x7C,0x60,0x30,0x60,0x7C,0x3C,0x00, // 0x77
   0x44,0x6C,0x38,0x10,0x38,0x6C,0x44,0x00, // 0x78
   0x9C,0xBC,0xA0,0xA0,0xFC,0x7C,0x00,0x00, // 0x79
   0x4C,0x64,0x74,0x5C,0x4C,0x64,0x00,0x00, // 0x7A
   0x08,0x08,0x3E,0x77,0x41,0x41,0x00,0x00, // 0x7B
   0x00,0x00,0x00,0x77,0x77,0x00,0x00,0x00, // 0x7C
   0x41,0x41,0x77,0x3E,0x08,0x08,0x00,0x00, // 0x7D
   0x02,0x03,0x01,0x03,0x02,0x03,0x01,0x00, // 0x7E
   0x78,0x7C,0x46,0x43,0x46,0x7C,0x78,0x00}; // 0x7F

#define FONT_W 8
#define FONT_H 2
#define FONT_STRETCHV 1
#define FONT_STRETCHH 0
*/

static const uint8_t ssd1306_init_sequence [] PROGMEM = {  // Initialization Sequence
//  0xAE,     // Display OFF (sleep mode)
    0x20, 0b10,   // Set Memory Addressing Mode
          // 00=Horizontal Addressing Mode; 01=Vertical Addressing Mode;
          // 10=Page Addressing Mode (RESET); 11=Invalid
 0xB0,     // Set Page Start Address for Page Addressing Mode, 0-7
  0xC8,     // Set COM Output Scan Direction.  Flip Veritically.
 0x00,     // Set low nibble of column address
 0x10,     // Set high nibble of column address
   0x40,     // Set display start line address
  0x81, /*32*/ 0x7F,   // Set contrast control register
  0xA1,     // Set Segment Re-map. A0=column 0 mapped to SEG0; A1=column 127 mapped to SEG0. Flip Horizontally
   0xA6,     // Set display mode. A6=Normal; A7=Inverse
  0xA8, 0x1F,   // Set multiplex ratio(1 to 64)
   0xA4,     // Output RAM to Display
          // 0xA4=Output follows RAM content; 0xA5,Output ignores RAM content
  0xD3, 0x00,   // Set display offset. 00 = no offset
   0xD5, 0x80,   // --set display clock divide ratio/oscillator frequency
  0xD9, 0xF1, // 0xF1=brighter //0x22,   // Set pre-charge period
  0xDA, 0x02,   // Set com pins hardware configuration
//   0xDB, 0x40, //0x20,   // --set vcomh 0x20 = 0.77xVcc
  0x8D, 0x14,    // Set DC-DC enable
  0xAF,     // Display ON
};

class SSD1306Device: public Print {
public:
  #define SSD1306_ADDR 0x3C  // Slave address  
  #define SSD1306_PAGES 4
  #define SSD1306_COMMAND 0x00
  #define SSD1306_DATA 0x40
  uint8_t oledX = 0, oledY = 0;
  uint8_t renderingFrame = 0xB0;
  bool wrap = false;

  void begin(uint8_t cols, uint8_t rows, uint8_t charsize = 0){
    Wire.begin();
    Wire.beginTransmission(SSD1306_ADDR); Wire.write(SSD1306_COMMAND);
    for (uint8_t i = 0; i < sizeof(ssd1306_init_sequence); i++) {
      Wire.write(pgm_read_byte(&ssd1306_init_sequence[i]));
    }
    Wire.endTransmission();
    delayMicroseconds(100);
  }
  void noCursor(){}
  void cursor(){}
  void noDisplay(){}
  void createChar(uint8_t l, uint8_t glyph[]){}

  void _setCursor(uint8_t x, uint8_t y) { oledX = x; oledY = y;
    Wire.beginTransmission(SSD1306_ADDR); Wire.write(SSD1306_COMMAND);
    Wire.write(renderingFrame | (oledY & 0x07));
    Wire.write(0x10 | ((oledX & 0xf0) >> 4));
    Wire.write(oledX & 0x0f);
    Wire.endTransmission();
  }
  void setCursor(uint8_t x, uint8_t y) { _setCursor(x * FONT_W, y * FONT_H); }
    
  void newLine() {
    oledY+=FONT_H;
    if (oledY > SSD1306_PAGES - FONT_H) {
      oledY = SSD1306_PAGES - FONT_H;
    }
    setCursor(0, oledY);
  }
    
  size_t write(byte c) {
    if((c == '\n') || (oledX > ((uint8_t)128 - FONT_W))) {
      if(wrap)  newLine();
      return 1;
    }
    
    uint16_t offset = ((uint16_t)c - ' ') * FONT_W/(FONT_STRETCHH+1) * FONT_H;
    uint8_t line = FONT_H;
    do
    {
      if(FONT_STRETCHV) offset = ((uint16_t)c - ' ') * FONT_W/(FONT_STRETCHH+1) * FONT_H/(2*FONT_STRETCHV);
      Wire.beginTransmission(SSD1306_ADDR); Wire.write(SSD1306_DATA);
      for (uint8_t i = 0; i < (FONT_W/(FONT_STRETCHH+1)); i++) {
        uint8_t b = pgm_read_byte(&(font[offset++]));
        if(FONT_STRETCHV){
          uint8_t b2 = 0;
          if(line > 1) for(int i = 0; i!=4; i++) b2 |=/* ! */(b & (1<<i)) ? (1<<(i*2)) | (1<<(i*2)+1): 0x00;
          else         for(int i = 0; i!=4; i++) b2 |=/* ! */(b & (1<<(i+4))) ? (1<<(i*2)) | (1<<(i*2)+1): 0x00;
          Wire.write(b2);
          if(FONT_STRETCHH) Wire.write(b2);
        } else { Wire.write(b); if(FONT_STRETCHH) Wire.write(b); }
      }
      Wire.endTransmission();
      if (FONT_H == 1) {
        oledX+=FONT_W;
      }
      else {
        if (line > 1) {
          _setCursor(oledX, oledY + 1);
        }
        else {
          _setCursor(oledX + FONT_W, oledY - (FONT_H - 1));
        }
      }
    }
    while (--line);
    return 1;
  }

  void bitmap(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, const uint8_t bitmap[]) {
    uint16_t j = 0;
    for (uint8_t y = y0; y < y1; y++) {
      _setCursor(x0, y);
      Wire.beginTransmission(SSD1306_ADDR); Wire.write(SSD1306_DATA);
      for (uint8_t x = x0; x < x1; x++) {
        Wire.write(pgm_read_byte(&bitmap[j++]));
      }
      Wire.endTransmission();
    }
    setCursor(0, 0);
  }
};
//#define OLED  1   // SDD1306 connection on display header: 1=GND(black), 2=RED(red), 13=SDA(brown), 14=SCK(orange)
#ifdef OLED
SSD1306Device lcd;
#else
LCD lcd;     // highly-optimized LCD driver, OK for QCX supplied displays
//LCD_ lcd;  // slower LCD, suitable for non-QCX supplied displays
//#include <LiquidCrystal.h>
//LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
#endif

volatile int8_t encoder_val = 0;
volatile int8_t encoder_step = 0;
static uint8_t last_state;
ISR(PCINT2_vect){  // Interrupt on rotary encoder turn
  //noInterrupts();
  switch(last_state = (last_state << 4) | (digitalRead(ROT_B) << 1) | digitalRead(ROT_A)){ //transition  (see: https://www.allaboutcircuits.com/projects/how-to-use-a-rotary-encoder-in-a-mcu-based-project/  )
//#define ENCODER_ENHANCED_RESOLUTION  1
#ifdef ENCODER_ENHANCED_RESOLUTION // Option: enhance encoder from 24 to 96 steps/revolution, see: appendix 1, https://www.sdr-kits.net/documents/PA0KLT_Manual.pdf
    case 0x31: case 0x10: case 0x02: case 0x23: encoder_val++; break;
    case 0x32: case 0x20: case 0x01: case 0x13: encoder_val--; break;
#else
    case 0x31: case 0x10: case 0x02: case 0x23: if(encoder_step < 0) encoder_step = 0; encoder_step++; if(encoder_step >  3){ encoder_step = 0; encoder_val++; } break;
    case 0x32: case 0x20: case 0x01: case 0x13: if(encoder_step > 0) encoder_step = 0; encoder_step--; if(encoder_step < -3){ encoder_step = 0; encoder_val--; } break;  
#endif
  }
  //interrupts();
}
void encoder_setup()
{
  pinMode(ROT_A, INPUT_PULLUP);
  pinMode(ROT_B, INPUT_PULLUP);
  PCMSK2 |= (1 << PCINT22) | (1 << PCINT23); // interrupt-enable for ROT_A, ROT_B pin changes; see https://github.com/EnviroDIY/Arduino-SDI-12/wiki/2b.-Overview-of-Interrupts
  PCICR |= (1 << PCIE2); 
  last_state = (digitalRead(ROT_B) << 1) | digitalRead(ROT_A);
  interrupts();
}
/*
class Encoder {
public:
  volatile int8_t val = 0;
  volatile int8_t step = 0;
  uint8_t last_state;
  Encoder(){
    pinMode(ROT_A, INPUT_PULLUP);
    pinMode(ROT_B, INPUT_PULLUP);
    PCMSK2 |= (1 << PCINT22) | (1 << PCINT23); // interrupt-enable for ROT_A, ROT_B pin changes; see https://github.com/EnviroDIY/Arduino-SDI-12/wiki/2b.-Overview-of-Interrupts
    PCICR |= (1 << PCIE2);
    last_state = (digitalRead(ROT_B) << 1) | digitalRead(ROT_A);
    sei();    
  }
  void event(){
    switch(last_state = (last_state << 4) | (digitalRead(ROT_B) << 1) | digitalRead(ROT_A)){ //transition  (see: https://www.allaboutcircuits.com/projects/how-to-use-a-rotary-encoder-in-a-mcu-based-project/  )
      case 0x31: case 0x10: case 0x02: case 0x23: if(step < 0) step = 0; step++; if(step >  3){ step = 0; val++; } break;
      case 0x32: case 0x20: case 0x01: case 0x13: if(step > 0) step = 0; step--; if(step < -3){ step = 0; val--; } break;  
    }
  }
};
Encoder enc;
ISR(PCINT2_vect){  // Interrupt on rotary encoder turn
  enc.event();
}*/

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
  /*#define SendByte(data) \
    SendBit(data, 1 << 7) \
    SendBit(data, 1 << 6) \
    SendBit(data, 1 << 5) \
    SendBit(data, 1 << 4) \
    SendBit(data, 1 << 3) \
    SendBit(data, 1 << 2) \
    SendBit(data, 1 << 1) \
    SendBit(data, 1 << 0) \
    I2C_SDA_HI();  // recv ACK \
    DELAY(I2C_DELAY);     \
    I2C_SCL_HI();         \
    I2C_SCL_LO();*/
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
    if(!i){ lcd.setCursor(0, 1); lcd.print(F("E07 I2C timeout")); }
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

#define log2(n) (log(n) / log(2))

// /*
I2C i2c;
class SI5351 {
public:
  volatile int32_t _fout;
  volatile uint8_t _div;  // note: uint8_t asserts fout > 3.5MHz with R_DIV=1
  volatile uint16_t _msa128min512;
  volatile uint32_t _msb128;
  volatile uint8_t pll_regs[8];

  #define BB0(x) ((uint8_t)(x))           // Bash byte x of int32_t
  #define BB1(x) ((uint8_t)((x)>>8))
  #define BB2(x) ((uint8_t)((x)>>16))

  #define FAST __attribute__((optimize("Ofast")))

  #define F_XTAL 27005000            // Crystal freq in Hz, nominal frequency 27004300
  //#define F_XTAL 25004000          // Alternate SI clock
  //#define F_XTAL 20004000          // A shared-single 20MHz processor/pll clock
  volatile uint32_t fxtal = F_XTAL;

  inline void FAST freq_calc_fast(int16_t df)  // note: relies on cached variables: _msb128, _msa128min512, _div, _fout, fxtal
  { 
    #define _MSC  0x80000  //0x80000: 98% CPU load   0xFFFFF: 114% CPU load
    uint32_t msb128 = _msb128 + ((int64_t)(_div * (int32_t)df) * _MSC * 128) / fxtal;

    //#define _MSC  0xFFFFF  // Old algorithm 114% CPU load, shortcut for a fixed fxtal=27e6
    //register uint32_t xmsb = (_div * (_fout + (int32_t)df)) % fxtal;  // xmsb = msb * fxtal/(128 * _MSC);
    //uint32_t msb128 = xmsb * 5*(32/32) - (xmsb/32);  // msb128 = xmsb * 159/32, where 159/32 = 128 * 0xFFFFF / fxtal; fxtal=27e6

    //#define _MSC  (F_XTAL/128)  // 114% CPU load  perfect alignment
    //uint32_t msb128 = (_div * (_fout + (int32_t)df)) % fxtal;

    uint32_t msp1 = _msa128min512 + msb128 / _MSC;  // = 128 * _msa + msb128 / _MSC - 512;
    uint32_t msp2 = msb128 % _MSC;  // = msb128 - msb128/_MSC * _MSC;

    //pll_regs[0] = BB1(msc);  // 3 regs are constant
    //pll_regs[1] = BB0(msc);
    //pll_regs[2] = BB2(msp1);
    pll_regs[3] = BB1(msp1);
    pll_regs[4] = BB0(msp1);
    pll_regs[5] = ((_MSC&0xF0000)>>(16-4))|BB2(msp2); // top nibble MUST be same as top nibble of _MSC !
    pll_regs[6] = BB1(msp2);
    pll_regs[7] = BB0(msp2);
  }
  #define SI5351_ADDR 0x60              // I2C address of Si5351   (typical)

  inline void SendPLLBRegisterBulk(){
    i2c.start();
    i2c.SendByte(SI5351_ADDR << 1);
    i2c.SendByte(26+1*8 + 3);  // Write to PLLB
    i2c.SendByte(pll_regs[3]);
    i2c.SendByte(pll_regs[4]);
    i2c.SendByte(pll_regs[5]);
    i2c.SendByte(pll_regs[6]);
    i2c.SendByte(pll_regs[7]);
    i2c.stop();
  }

  void SendRegister(uint8_t reg, uint8_t* data, uint8_t n){
    i2c.start();
    i2c.SendByte(SI5351_ADDR << 1);
    i2c.SendByte(reg);
    while (n--) i2c.SendByte(*data++);
    i2c.stop();      
  }
  void SendRegister(uint8_t reg, uint8_t val){ SendRegister(reg, &val, 1); }

  int16_t iqmsa; // to detect a need for a PLL reset

  void freq(uint32_t fout, uint8_t i, uint8_t q){  // Set a CLK0,1 to fout Hz with phase i, q
      uint8_t msa; uint32_t msb, msc, msp1, msp2, msp3p2;
      uint8_t rdiv = 0;             // CLK pin sees fout/(2^rdiv)
      if(fout < 500000){ rdiv = 7; fout *= 128; }; // Divide by 128 for fout 4..500kHz

      uint16_t d = (16 * fxtal) / fout;  // Integer part
      if(fout > 30000000) d = (34 * fxtal) / fout; // when fvco is getting too low (400 MHz)

      if( (d * (fout - 5000) / fxtal) != (d * (fout + 5000) / fxtal) ) d--; // Test if multiplier remains same for freq deviation +/- 5kHz, if not use different divider to make same
      uint32_t fvcoa = d * fout;  // Variable PLLA VCO frequency at integer multiple of fout at around 27MHz*16 = 432MHz
      msa = fvcoa / fxtal;     // Integer part of vco/fxtal
      msb = ((uint64_t)(fvcoa % fxtal)*_MSC) / fxtal; // Fractional part
      msc = _MSC;
      
      msp1 = 128*msa + 128*msb/msc - 512;
      msp2 = 128*msb - 128*msb/msc * msc;    // msp3 == msc        
      msp3p2 = (((msc & 0x0F0000) <<4) | msp2);  // msp3 on top nibble
      uint8_t pll_regs[8] = { BB1(msc), BB0(msc), BB2(msp1), BB1(msp1), BB0(msp1), BB2(msp3p2), BB1(msp2), BB0(msp2) };
      SendRegister(26+0*8, pll_regs, 8); // Write to PLLA
      SendRegister(26+1*8, pll_regs, 8); // Write to PLLB

      msa = fvcoa / fout;     // Integer part of vco/fout
      msp1 = (128*msa - 512) | (((uint32_t)rdiv)<<20);     // msp1 and msp2=0, msp3=1, not fractional
      uint8_t ms_regs[8] = {0, 1, BB2(msp1), BB1(msp1), BB0(msp1), 0, 0, 0};
      SendRegister(42+0*8, ms_regs, 8); // Write to MS0
      SendRegister(42+1*8, ms_regs, 8); // Write to MS1
      SendRegister(42+2*8, ms_regs, 8); // Write to MS2
      SendRegister(16+0, 0x0C|3|0x40);  // CLK0: 0x0C=PLLA local msynth; 3=8mA; 0x40=integer division; bit7:6=0->power-up
      SendRegister(16+1, 0x0C|3|0x40);  // CLK1: 0x0C=PLLA local msynth; 3=8mA; 0x40=integer division; bit7:6=0->power-up
      SendRegister(16+2, 0x2C|3|0x40);  // CLK2: 0x2C=PLLB local msynth; 3=8mA; 0x40=integer division; bit7:6=0->power-up
      SendRegister(165, i * msa / 90);  // CLK0: I-phase (on change -> Reset PLL)
      SendRegister(166, q * msa / 90);  // CLK1: Q-phase (on change -> Reset PLL)
      if(iqmsa != ((i-q)*msa/90)){ iqmsa = (i-q)*msa/90; SendRegister(177, 0xA0); } // 0x20 reset PLLA; 0x80 reset PLLB
      SendRegister(3, 0b11111100);      // Enable/disable clock

      _fout = fout;  // cache
      _div = d;
      _msa128min512 = fvcoa / fxtal * 128 - 512;
      _msb128=((uint64_t)(fvcoa % fxtal)*_MSC*128) / fxtal;
  }

  volatile int32_t prev_pll_freq;
  uint8_t RecvRegister(uint8_t reg){
    i2c.start();  // Data write to set the register address
    i2c.SendByte(SI5351_ADDR << 1);
    i2c.SendByte(reg);
    i2c.stop();
    i2c.start(); // Data read to retrieve the data from the set address
    i2c.SendByte((SI5351_ADDR << 1) | 1);
    uint8_t data = i2c.RecvByte(true);
    i2c.stop();
    return data;
  }
  void powerDown(){
    for(int addr = 16; addr != 24; addr++) SendRegister(addr, 0b11000000);  // Conserve power when output is disabled
    SendRegister(3, 0b11111111); // Disable all CLK outputs    
  }
  #define SI_CLK_OE 3

};
static SI5351 si5351;
// */

 /*
class SI5351 : public I2C {
public:
  #define SI_I2C_ADDR 0x60  // SI5351A I2C address: 0x60 for SI5351A-B-GT; 0x62 for SI5351A-B-04486-GT; 0x6F for SI5351A-B02075-GT; see here for other variants: https://www.silabs.com/TimingUtility/timing-download-document.aspx?OPN=Si5351A-B02075-GT&OPNRevision=0&FileType=PublicAddendum
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
  volatile uint32_t fxtal = 27004300;  //myqcx1:27003847 myqcx2:27004900  Actual crystal frequency of 27MHz XTAL2 for CL = 10pF (default), calibrate your QCX 27MHz crystal frequency here
  #define SI_PLL_FREQ (16*fxtal)  //900000000, with 432MHz(=16*27M) PLL freq, usable range is 3.46..100MHz

  volatile uint8_t prev_divider;
  volatile int32_t raw_freq;
  volatile uint8_t divider;  // note: because of int8 only freq > 3.6MHz can be covered for R_DIV=1
  volatile uint8_t mult;
  volatile uint8_t pll_regs[8];
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
  inline void SendPLLBRegisterBulk()  // fast freq change of PLLB, takes about [ 2 + 7*(8+1) + 2 ] / 840000 = 80 uS
  {
    start();
    SendByte(SI_I2C_ADDR << 1);
    SendByte(SI_SYNTH_PLL_B + 3);  // Skip the first three pll_regs bytes (first two always 0xFF and third not often changing
    SendByte(pll_regs[3]);
    SendByte(pll_regs[4]);
    SendByte(pll_regs[5]);
    SendByte(pll_regs[6]);
    SendByte(pll_regs[7]);
    stop();
  }
  
  #define FAST __attribute__((optimize("Ofast")))

  // this function relies on cached (global) variables: divider, mult, raw_freq, pll_regs
  inline void FAST freq_calc_fast(int16_t freq_offset)
  { // freq_offset is relative to freq set in freq(freq)
    // uint32_t num128 = ((divider * (raw_freq + offset)) % fxtal) * (float)(0xFFFFF * 128) / fxtal;
    // Above definition (for fxtal=27.00491M) can be optimized by pre-calculating factor (0xFFFFF*128)/fxtal (=4.97) as integer constant (5) and
    // substracting the rest error factor (0.03). Note that the latter is shifted left (0.03<<6)=2, while the other term is shifted right (>>6)
    register int32_t z = ((divider * (raw_freq + freq_offset)) % fxtal);
    register int32_t z2 = -(z >> 5);
    int32_t num128 = (z * 5) + z2;
  
    // Set up specified PLL with mult, num and denom: mult is 15..90, num128 is 0..128*1,048,575 (128*0xFFFFF), denom is 0..1,048,575 (0xFFFFF)
    uint32_t P1 = 128 * mult + (num128 / 0xFFFFF) - 512;
    uint32_t P2 = num128 % 0xFFFFF;
    //pll_regs[0] = 0xFF;
    //pll_regs[1] = 0xFF;
    //pll_regs[2] = (P1 >> 14) & 0x0C;
    pll_regs[3] = P1 >> 8;
    pll_regs[4] = P1;
    pll_regs[5] = 0xF0 | (P2 >> 16);
    pll_regs[6] = P2 >> 8;
    pll_regs[7] = P2;
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
    //uint8_t r_div = (freq > (SI_PLL_FREQ/256/1)) ? 1 : (freq > (SI_PLL_FREQ/256/32)) ? 32 : 128; // helps divider to be in range
    uint8_t r_div = (freq < 500000) ? 128 : 1;
    freq *= r_div;  // take r_div into account, now freq is in the range 1MHz to 150MHz
    raw_freq = freq;   // cache frequency generated by PLL and MS stages (excluding R divider stage); used by freq_calc_fast()
  
    divider = SI_PLL_FREQ / freq;  // Calculate the division ratio. 900,000,000 is the maximum internal PLL freq (official range 600..900MHz but can be pushed to 300MHz..~1200Mhz)
    if(divider % 2) divider--;  // divider in range 8.. 900 (including 4,6 for integer mode), even numbers preferred. Note that uint8 datatype is used, so 254 is upper limit
    if( (divider * (freq - 5000) / fxtal) != (divider * (freq + 5000) / fxtal) ) divider -= 2; // Test if multiplier remains same for freq deviation +/- 5kHz, if not use different divider to make same
    pll_freq = divider * freq; // Calculate the pll_freq: the divider * desired output freq
    uint32_t num, denom;
    mult = div(pll_freq, fxtal, &num, &denom); // Determine the mult to get to the required pll_freq (in the range 15..90)
  
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
    //if(prev_divider != divider){ lcd.setCursor(0, 0); lcd.print(divider); lcd_blanks();
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
    //SendRegister(24, 0b00000000); // CLK3-0 Disable State: CLK2=0 (BE CAREFUL TO CHANGE THIS!!!), CLK1/0=00 -> IC4-X0 selected -> 2,5V on IC5A/3(+), when IC5/2(-) leaks down below 2.5V -> 12V on IC5A/1, IC6A/2(-) -> 0V on IC6A/1, AUDIO2
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
    SendRegister(SI_CLK0_CONTROL, 0b11000000);  // Conserve power when output is disabled
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
 */

#undef F_CPU
#define F_CPU 20007000   // myqcx1:20008440, myqcx2:20006000   // Actual crystal frequency of 20MHz XTAL1, note that this declaration is just informative and does not correct the timing in Arduino functions like delay(); hence a 1.25 factor needs to be added for correction.
//#define F_CPU F_XTAL   // in case ATMEGA328P clock is the same as SI5351 clock (ATMEGA clock tapped from SI crystal)

#define DEBUG  1   // enable testing and diagnostics features
#ifdef DEBUG
static uint32_t sr = 0;
static uint32_t cpu_load = 0;
volatile uint16_t param_a = 0;  // registers for debugging, testing and experimental purposes
volatile int16_t param_b = 0;
volatile int16_t param_c = 0;
#endif

enum mode_t { LSB, USB, CW, AM, FM };
volatile int8_t mode = USB;
volatile uint16_t numSamples = 0;

volatile uint8_t tx = 0;
volatile bool vox = false;

inline void _vox(uint8_t trigger)
{
  if(trigger){
    //if(!tx){ /* TX can be enabled here */ }
    tx = (vox) ? 255 : 1; // hangtime = 255 / 4402 = 58ms (the time that TX at least stays on when not triggered again)
  } else {
    if(tx){
      tx--;
      //if(!tx){ /* RX can be enabled here */ }
    }
  }
}

//#define F_SAMP_TX 4402
#define F_SAMP_TX 4810        //4810 // ADC sample-rate; is best a multiple of _UA and fits exactly in OCR0A = ((F_CPU / 64) / F_SAMP_TX) - 1 , should not exceed CPU utilization
#define _UA  (F_SAMP_TX)      //360  // unit angle; integer representation of one full circle turn or 2pi radials or 360 degrees, should be a integer divider of F_SAMP_TX and maximized to have higest precision
//#define MAX_DP  (_UA/1)  //(_UA/2) // the occupied SSB bandwidth can be further reduced by restricting the maximum phase change (set MAX_DP to _UA/2).
//#define CONSTANT_AMP  1 // enable this in case there is no circuitry for controlling envelope (key shaping circuit)
//#define CARRIER_COMPLETELY_OFF_ON_LOW  1    // disable oscillator on no-envelope transitions, to prevent potential unwanted biasing/leakage through PA circuit
#define MULTI_ADC  1  // multiple ADC conversions for more sensitive (+12dB) microphone input

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
volatile uint8_t vox_thresh = (1 << 2);
volatile uint8_t drive = 2;   // hmm.. drive>2 impacts cpu load..why?

inline int16_t ssb(int16_t in)
{
  static int16_t dc;

  int16_t i, q;
  uint8_t j;
  static int16_t v[16];

  for(j = 0; j != 15; j++) v[j] = v[j + 1];

  dc += (in - dc) / 2;
  v[15] = in - dc;     // DC decoupling
  //dc = in;  // this is actually creating a high-pass (emphasis) filter

  i = v[7];
  q = ((v[0] - v[14]) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 15) / 128 + (v[6] - v[8]) / 2; // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)

  uint16_t _amp = magn(i, q);
  if(vox) _vox(_amp > vox_thresh);
  //_amp = (_amp > vox_thresh) ? _amp : 0;   // vox_thresh = 1 is a good setting

  _amp = _amp << (drive);
#ifdef CONSTANT_AMP
  if(_amp < 4 ){ amp = 0; return 0; } //hack: for constant amplitude cases, set drive=1 for good results
  //digitalWrite(RX, (_amp < 4)); // fast on-off switching for constant amplitude case
#endif
  _amp = ((_amp > 255) || (drive == 8)) ? 255 : _amp; // clip or when drive=8 use max output
  amp = (tx) ? lut[_amp] : 0;

  static int16_t prev_phase;
  int16_t phase = arctan3(q, i);

  int16_t dp = phase - prev_phase;  // phase difference and restriction
  //dp = (amp) ? dp : 0;  // dp = 0 when amp = 0
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
volatile int8_t mox = 0;
volatile int8_t volume = 12;

// This is the ADC ISR, issued with sample-rate via timer1 compb interrupt.
// It performs in real-time the ADC sampling, calculation of SSB phase-differences, calculation of SI5351 frequency registers and send the registers to SI5351 over I2C.
static int16_t _adc;
void dsp_tx()
{ // jitter dependent things first
#ifdef MULTI_ADC  // SSB with multiple ADC conversions:
  int16_t adc;                         // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  adc = ADC;
  ADCSRA |= (1 << ADSC);
  //OCR1BL = amp;                        // submit amplitude to PWM register (actually this is done in advance (about 140us) of phase-change, so that phase-delays in key-shaping circuit filter can settle)
  si5351.SendPLLBRegisterBulk();       // submit frequency registers to SI5351 over 731kbit/s I2C (transfer takes 64/731 = 88us, then PLL-loopfilter probably needs 50us to stabalize)
  OCR1BL = amp;                      // submit amplitude to PWM register (takes about 1/32125 = 31us+/-31us to propagate) -> amplitude-phase-alignment error is about 30-50us
  adc += ADC;
  //ADCSRA |= (1 << ADSC);  // causes RFI on QCX-SSB units (not on units with direct biasing); ENABLE this line when using direct biasing!!
  int16_t df = ssb(_adc >> MIC_ATTEN); // convert analog input into phase-shifts (carrier out by periodic frequency shifts)
  adc += ADC;
  ADCSRA |= (1 << ADSC);
  si5351.freq_calc_fast(df);           // calculate SI5351 registers based on frequency shift and carrier frequency
  adc += ADC;
  ADCSRA |= (1 << ADSC);
  _adc = (adc/4 - 512);
#else  // SSB with single ADC conversion:
  ADCSRA |= (1 << ADSC);    // start next ADC conversion (trigger ADC interrupt if ADIE flag is set)
  //OCR1BL = amp;                        // submit amplitude to PWM register (actually this is done in advance (about 140us) of phase-change, so that phase-delays in key-shaping circuit filter can settle)
  si5351.SendPLLBRegisterBulk();       // submit frequency registers to SI5351 over 731kbit/s I2C (transfer takes 64/731 = 88us, then PLL-loopfilter probably needs 50us to stabalize)
  OCR1BL = amp;                        // submit amplitude to PWM register (takes about 1/32125 = 31us+/-31us to propagate) -> amplitude-phase-alignment error is about 30-50us
  int16_t adc = ADC - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  int16_t df = ssb(adc >> MIC_ATTEN);  // convert analog input into phase-shifts (carrier out by periodic frequency shifts)
  si5351.freq_calc_fast(df);           // calculate SI5351 registers based on frequency shift and carrier frequency
#endif

#ifdef CARRIER_COMPLETELY_OFF_ON_LOW
  if(OCR1BL == 0){ si5351.SendRegister(SI_CLK_OE, (amp) ? 0b11111011 : 0b11111111); } // experimental carrier-off for low amplitudes
#endif

  if(!mox) return;
  OCR1AL = (adc << (mox-1)) + 128;  // TX audio monitoring
}

volatile uint16_t acc;
volatile uint32_t cw_offset;

volatile int16_t p_sin = 0;   // initialized with A*sin(0) = 0
volatile int16_t n_cos = 448/2; // initialized with A*cos(t) = A
inline void process_minsky() // Minsky circle sample [source: https://www.cl.cam.ac.uk/~am21/hakmemc.html, ITEM 149]: p_sin+=n_cos*2*PI*f/fs; n_cos-=p_sin*2*PI*f/fs;
{
  uint8_t alpha100 = cw_offset * 628 / F_SAMP_TX;  // alpha = f_tone * 6.28 / fs
  p_sin += alpha100 * n_cos / 100;
  n_cos -= alpha100 * p_sin / 100;
}

void dummy()
{
}

void dsp_tx_cw()
{ // jitter dependent things first
  OCR1BL = lut[255];
  
  process_minsky();
  OCR1AL = (p_sin >> (16 - volume)) + 128;
}

void dsp_tx_am()
{ // jitter dependent things first
  ADCSRA |= (1 << ADSC);    // start next ADC conversion (trigger ADC interrupt if ADIE flag is set)
  OCR1BL = amp;                        // submit amplitude to PWM register (actually this is done in advance (about 140us) of phase-change, so that phase-delays in key-shaping circuit filter can settle)
  int16_t adc = ADC - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  int16_t in = (adc >> MIC_ATTEN);
  in = in << (drive-4);
  //static int16_t dc;
  //dc += (in - dc) / 2;
  //in = in - dc;     // DC decoupling
  #define AM_BASE 32
  in=max(0, min(255, (in + AM_BASE)));
  amp=in;// lut[in];
}

uint8_t reg;
void dsp_tx_dsb()
{ // jitter dependent things first
  ADCSRA |= (1 << ADSC);    // start next ADC conversion (trigger ADC interrupt if ADIE flag is set)
  OCR1BL = amp;                        // submit amplitude to PWM register (actually this is done in advance (about 140us) of phase-change, so that phase-delays in key-shaping circuit filter can settle)
  si5351.SendRegister(16+2, reg);             // CLK2 polarity depending on amplitude
  int16_t adc = ADC - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  int16_t in = (adc >> MIC_ATTEN);
  in = in << drive;
  reg = (in < 0) ? 0x2C|3|0x10 : 0x2C|3;  //0x2C=PLLB local msynth 3=8mA 0x10=invert
  in=min(255, abs(in));
  amp=in;// lut[in];
}

void dsp_tx_fm()
{ // jitter dependent things first
  ADCSRA |= (1 << ADSC);    // start next ADC conversion (trigger ADC interrupt if ADIE flag is set)
  OCR1BL = lut[255];                   // submit amplitude to PWM register (actually this is done in advance (about 140us) of phase-change, so that phase-delays in key-shaping circuit filter can settle)
  si5351.SendPLLBRegisterBulk();       // submit frequency registers to SI5351 over 731kbit/s I2C (transfer takes 64/731 = 88us, then PLL-loopfilter probably needs 50us to stabalize)
  int16_t adc = ADC - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  int16_t in = (adc >> MIC_ATTEN);
  in = in << (drive);
  int16_t df = in;
  si5351.freq_calc_fast(df);           // calculate SI5351 registers based on frequency shift and carrier frequency
}

volatile int8_t cwdec = 0;

static int32_t signal;
static int16_t avg = 0;
static int16_t maxpk=0;
static int16_t k0=0;
static int16_t k1=0;
static uint8_t sym;
static int16_t ta=0;
const char m2c[] PROGMEM = "**ETIANMSURWDKGOHVF*L*PJBXCYZQ**54S3***2**+***J16=/***H*7*G*8*90************?_****\"**.****@***'**-********;!*)*****,****:****";
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
        if(sym<128) ch=/*m2c[sym]*/ pgm_read_byte_near(m2c + sym);
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
#define F_ADC_CONV (192307/1)

volatile bool agc = true;
volatile uint8_t nr = 0;
volatile uint8_t att = 0;
volatile uint8_t att2 = 0;
volatile uint8_t _init;

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

inline int16_t process_nr_old(int16_t ac)
{
  ac = ac >> (6-abs(ac));  // non-linear below amp of 6; to reduce noise (switchoff agc and tune-up volume until noise dissapears, todo:extra volume control needed)
  ac = ac << 3;
  return ac;
}

#define EA(y, x, one_over_alpha)  (y) = (y) + ((x) - (y)) / (one_over_alpha); // exponental averaging [Lyons 13.33.1]
#define MLEA(y, x, L, M)  (y)  = (y) + ((((x) - (y)) >> (L)) - (((x) - (y)) >> (M))); // multiplierless exponental averaging [Lyons 13.33.1], with alpha=1/2^L - 1/2^M

inline int16_t process_nr_old2(int16_t ac)
{  
  int16_t x = ac;
  static int16_t ea1;
  //ea1 = MLEA(ea1, ac, 5, 6); // alpha=0.0156
  ea1 = EA(ea1, ac, 64); // alpha=1/64=0.0156
  //static int16_t ea2;
  //ea2 = EA(ea2, ea1, 64); // alpha=1/64=0.0156
 
  return ea1;
}

inline int16_t process_nr(int16_t in)
{ 
/*
  static int16_t avg;
  avg = EA(avg, abs(in), 64); // alpha=1/64=0.0156
param_c = avg;
*/

/*
  int32_t _avg = 64 * avg;
//  if(_avg > 4) _avg = 4;  // clip
//  uint16_t brs_avgsq = 1 << (_avg * _avg);
  if(_avg > 14) _avg = 14;  // clip
  uint16_t brs_avgsq = 1 << (_avg);

  
  int16_t inv_gain;
  if(brs_avgsq > 1) inv_gain = brs_avgsq / (brs_avgsq - 1);  // = 1 / (1 - 1/(1 << (1*avg*avg)) );
  else inv_gain = 32768;*/

  static int16_t ea1;
  ea1 = EA(ea1, in, 1 << (nr-1) );
  //static int16_t ea2;
  //ea2 = EA(ea2, ea1, inv_gain);

  return ea1;
}

#define N_FILT 7
volatile int8_t filt = 0;

inline int16_t filt_var(int16_t za0)  //filters build with www.micromodeler.com
{ 
  static int16_t za1,za2;
  
  //if(filt < 4)
  {  // for SSB filters
    // 1st Order (SR=8kHz) IIR in Direct Form I, 8x8:16
    static int16_t zz1,zz2;
    za0=(29*(za0-zz1)+50*za1)/64;                               //300-Hz
    zz2=zz1;
    zz1=za0;
  }

  // 4th Order (SR=8kHz) IIR in Direct Form I, 8x8:16
  static int16_t zb0,zb1,zb2;
  switch(filt){
    case 1: zb0=za0; break; //0-4000Hz (pass-through)
    case 2: zb0=(10*(za0+2*za1+za2)+16*zb1-17*zb2)/32; break;    //0-2500Hz  elliptic -60dB@3kHz
    case 3: zb0=(7*(za0+2*za1+za2)+48*zb1-18*zb2)/32; break;     //0-1700Hz  elliptic
  }

  static int16_t zc0,zc1,zc2;
  switch(filt){
    case 1: zc0=zb0; break; //0-4000Hz (pass-through)
    case 2: zc0=(8*(zb0+zb2)+13*zb1-43*zc1-52*zc2)/64; break;   //0-2500Hz  elliptic -60dB@3kHz
    case 3: zc0=(4*(zb0+zb1+zb2)+22*zc1-47*zc2)/64; break;   //0-1700Hz  elliptic
  }

  zc2=zc1;
  zc1=zc0;

  zb2=zb1;
  zb1=zb0;

  za2=za1;
  za1=za0;
  
  return zc0;
}

inline int16_t filt_var_cw(int16_t za0)  //filters build with www.micromodeler.com
{ //   (2nd Order (SR=4465kHz) IIR in Direct Form I, 8x8:16), adding 64x front-gain, 
  static int16_t za1,za2;
  static int16_t zb0,zb1,zb2;

  switch(filt){
    //case 4: zb0=(5*za0+9*za1+5*za2)+(30L*zb1-38L*zb2)/64; break; //720Hz+-250Hz
    //case 5: zb0=(2*za0+4*za1+2*za2)+(51L*zb1-52L*zb2)/64; break; //720Hz+-100Hz
    //case 6: zb0=(1*za0+2*za1+1*za2)+(59L*zb1-58L*zb2)/64; break; //720Hz+-50Hz
    //case 7: zb0=(0*za0+1*za1+0*za2)+(66L*zb1-61L*zb2)/64; break; //720Hz+-25Hz
    case 4: zb0=(za0+2*za1+za2)/2+(41L*zb1-23L*zb2)/32; break;   //500-1000Hz
    case 5: zb0=5*(za0-2*za1+za2)+(105L*zb1-58L*zb2)/64; break;   //650-840Hz
    case 6: zb0=3*(za0-2*za1+za2)+(108L*zb1-61L*zb2)/64; break;   //650-750Hz
    case 7: zb0=(2*za0-3*za1+2*za2)+(111L*zb1-62L*zb2)/64; break; //630-680Hz
  }

  static int16_t zc0,zc1,zc2;
  switch(filt){
    //case 4: zc0=(zb0-2*zb1+zb2)/4+(76L*zc1-44L*zc2)/64; break; //720Hz+-250Hz
    //case 5: zc0=(zb0-2*zb1+zb2)/8+(72L*zc1-53L*zc2)/64; break; //720Hz+-100Hz
    //case 6: zc0=(zb0-2*zb1+zb2)/16+(70L*zc1-58L*zc2)/64; break; //720Hz+-50Hz
    //case 7: zc0=(zb0-2*zb1+zb2)/32+(70L*zc1-62L*zc2)/64; break; //720Hz+-25Hz
    case 4: zc0=(zb0-2*zb1+zb2)/4+(105L*zc1-52L*zc2)/64; break;      //500-1000Hz
    case 5: zc0=((zb0+2*zb1+zb2)+97L*zc1-57L*zc2)/64; break;      //650-840Hz
    case 6: zc0=((zb0+zb1+zb2)+104L*zc1-60L*zc2)/64; break;       //650-750Hz
    case 7: zc0=((zb1)+109L*zc1-62L*zc2)/64; break;               //630-680Hz
  }

  zc2=zc1;
  zc1=zc0;

  zb2=zb1;
  zb1=zb0;

  za2=za1;
  za1=za0;

  return zc0 / 64;
}

/*inline int16_t filt_var_cw(int32_t za0)  //filters build with www.micromodeler.com
{ 
  static int32_t za1,za2;
  za0 *= 64;

  // 4th Order (SR=8kHz) IIR in Direct Form I, 8x8:16
  static int32_t zb0,zb1,zb2;
  switch(filt){
    case 4: zb0=(za0+2*za1+za2+41*zb1-23*zb2)/32; break;   //500-1000Hz
    case 5: zb0=(5*(za0-2*za1+za2)+105*zb1-58*zb2)/64; break;   //650-840Hz
    case 6: zb0=(3*(za0-2*za1+za2)+108*zb1-61*zb2)/64; break;   //650-750Hz
    case 7: zb0=((2*za0-3*za1+2*za2)+111*zb1-62*zb2)/64; break; //630-680Hz
  }

  static int32_t zc0,zc1,zc2;
  switch(filt){
    case 4: zc0=(16*(zb0-2*zb1+zb2)+105*zc1-52*zc2)/64; break;      //500-1000Hz
    case 5: zc0=((zb0+2*zb1+zb2)+97*zc1-57*zc2)/64; break;      //650-840Hz
    case 6: zc0=((zb0+zb1+zb2)+104*zc1-60*zc2)/64; break;       //650-750Hz
    case 7: zc0=((zb1)+109*zc1-62*zc2)/64; break;               //630-680Hz
  }

  zc2=zc1;
  zc1=zc0;

  zb2=zb1;
  zb1=zb0;

  za2=za1;
  za1=za0;
  
  return zc0 / 64;
}*/

static uint32_t absavg256 = 0;
volatile uint32_t _absavg256 = 0;
volatile int16_t i, q;

inline int16_t slow_dsp(int16_t ac)
{
  static uint8_t absavg256cnt;
  if(!(absavg256cnt--)){ _absavg256 = absavg256; absavg256 = 0; 
//#define AUTO_ADC_BIAS  1
#ifdef AUTO_ADC_BIAS
    if(param_b < 0){
      pinMode(AUDIO1, INPUT_PULLUP);
      pinMode(AUDIO1, INPUT);
    }
    if(param_c < 0){
      pinMode(AUDIO2, INPUT_PULLUP);
      pinMode(AUDIO2, INPUT);
    }
    if(param_b > 500){
      pinMode(AUDIO1, OUTPUT);
      digitalWrite(AUDIO1, LOW);
      pinMode(AUDIO1, INPUT);
    }
    if(param_c > 500){
      pinMode(AUDIO2, OUTPUT);
      digitalWrite(AUDIO2, LOW);
      pinMode(AUDIO2, INPUT);
    }
#endif
  } else absavg256 += abs(ac);

  if(mode == AM) { // (12%CPU for the mode selection etc)
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

  if(mode == CW){
    ac = filt_var_cw(ac) << 2;
    if(cwdec){  // CW decoder enabled?
      char ch = cw(ac >> 0);
      if(ch){
        for(int i=0; i!=15;i++) out[i]=out[i+1];
        out[15] = ch;
        cw_event = true;
      }
    }
  } else if(filt) ac = filt_var(ac) << 2;
  //if(!(absavg256cnt--)){ _absavg256 = absavg256; absavg256 = 0; } else absavg256 += abs(ac);  //hack
  
  //static int16_t dc;
  //dc += (ac - dc) / 2;
  //dc = (15*dc + ac)/16;
  //dc = (15*dc + (ac - dc))/16;
  //ac = ac - dc;    // DC decoupling

  ac = min(max(ac, -512), 511);
  //ac = min(max(ac, -128), 127);
  return ac;
}

typedef void (*func_t)(void);
volatile func_t func_ptr;
#undef  R  // Decimating 2nd Order CIC filter
#define R 4  // Rate change from 62500/2 kSPS to 7812.5SPS, providing 12dB gain

//#define SIMPLE_RX  1
#ifndef SIMPLE_RX
volatile uint8_t admux[3];
volatile int16_t ocomb, qh;
volatile uint8_t rx_state = 0;

// Non-recursive CIC Filter (M=2, R=4) implementation, so two-stages of (followed by down-sampling with factor 2):
// H1(z) = (1 + z^-1)^2 = 1 + 2*z^-1 + z^-2 = (1 + z^-2) + (2) * z^-1 = FA(z) + FB(z) * z^-1;
// with down-sampling before stage translates into poly-phase components: FA(z) = 1 + z^-1, FB(z) = 2
// source: Lyons Understanding Digital Signal Processing 3rd edition 13.24.1
void sdr_rx()
{
  // process I for even samples  [75% CPU@R=4;Fs=62.5k] (excluding the Comb branch and output stage)
  ADMUX = admux[1];  // set MUX for next conversion
  ADCSRA |= (1 << ADSC);    // start next ADC conversion
  int16_t adc = ADC - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  func_ptr = sdr_rx_q;    // processing function for next conversion
  sdr_rx_common();
  
  // Only for I: correct I/Q sample delay by means of linear interpolation
  static int16_t prev_adc;
  int16_t corr_adc = (prev_adc + adc) / 2;
  prev_adc = adc;
  adc = corr_adc;

  //static int16_t dc;
  //dc += (adc - dc) / 2;  // we lose LSB with this method
  //dc = (3*dc + adc)/4;
  //int16_t ac = adc - dc;     // DC decoupling
  int16_t ac = adc;

#ifdef AUTO_ADC_BIAS
  param_b = (7*param_b + adc)/8;
#endif
  int16_t ac2;
  static int16_t z1;
  if(rx_state == 0 || rx_state == 4){  // 1st stage: down-sample by 2
    static int16_t za1;
    int16_t _ac = ac + za1 + z1 * 2;           // 1st stage: FA + FB
    za1 = ac;
    static int16_t _z1;
    if(rx_state == 0){                   // 2nd stage: down-sample by 2
      static int16_t _za1;
      ac2 = _ac + _za1 + _z1 * 2;              // 2nd stage: FA + FB
      _za1 = _ac;
      {
        // post processing I and Q (down-sampled) results
        static int16_t v[8];
        v[7] = ac2 >> att2;
        
        i = v[0]; v[0] = v[1]; v[1] = v[2]; v[2] = v[3]; v[3] = v[4]; v[4] = v[5]; v[5] = v[6]; v[6] = v[7];  // Delay to match Hilbert transform on Q branch

        int16_t ac = i + qh;
        ac = slow_dsp(ac);

        // Output stage
        static int16_t ozd1, ozd2;
        if(_init){ ac = 0; ozd1 = 0; ozd2 = 0; _init = 0; } // hack: on first sample init accumlators of further stages (to prevent instability)
#define SECOND_ORDER_DUC  1
#ifdef SECOND_ORDER_DUC
        int16_t od1 = ac - ozd1; // Comb section
        ocomb = od1 - ozd2;
        ozd2 = od1;
#else
        ocomb = ac - ozd1; // Comb section
#endif
        ozd1 = ac;
      }
    } else _z1 = _ac;
  } else z1 = ac;

  rx_state++;
}

void sdr_rx_q()
{
  // process Q for odd samples  [75% CPU@R=4;Fs=62.5k] (excluding the Comb branch and output stage)
  ADMUX = admux[0];  // set MUX for next conversion
  ADCSRA |= (1 << ADSC);    // start next ADC conversion
  int16_t adc = ADC - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  func_ptr = sdr_rx;    // processing function for next conversion
#ifdef SECOND_ORDER_DUC
//  sdr_rx_common();  //necessary? YES!... Maybe NOT!
#endif

  //static int16_t dc;
  //dc += (adc - dc) / 2;  // we lose LSB with this method
  //dc = (3*dc + adc)/4;
  //int16_t ac = adc - dc;     // DC decoupling
  int16_t ac = adc;

#ifdef AUTO_ADC_BIAS
  param_c = (7*param_c + adc)/8;
#endif
  int16_t ac2;
  static int16_t z1;
  if(rx_state == 3 || rx_state == 7){  // 1st stage: down-sample by 2
    static int16_t za1;
    int16_t _ac = ac + za1 + z1 * 2;           // 1st stage: FA + FB
    za1 = ac;
    static int16_t _z1;
    if(rx_state == 7){                   // 2nd stage: down-sample by 2
      static int16_t _za1;
      ac2 = _ac + _za1 + _z1 * 2;              // 2nd stage: FA + FB
      _za1 = _ac;
      {
        // Process Q (down-sampled) samples
        static int16_t v[16];
        v[15] = ac2 >> att2;

        for(uint8_t j = 0; j != 15; j++) v[j] = v[j + 1];
        q = v[7];
        qh = ((v[0] - v[14]) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 15) / 128 + (v[6] - v[8]) / 2; // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)
      }
      rx_state = 0; return;
    } else _z1 = _ac;
  } else z1 = ac;

  rx_state++;
}

inline void sdr_rx_common()
{
  static int16_t ozi1, ozi2;
  if(_init){ ocomb=0; ozi1 = 0; ozi2 = 0; } // hack
  // Output stage [25% CPU@R=4;Fs=62.5k]
#ifdef SECOND_ORDER_DUC
  ozi2 = ozi1 + ozi2;          // Integrator section
#endif
  ozi1 = ocomb + ozi1;
#ifdef SECOND_ORDER_DUC
  if(volume) OCR1AL = min(max((ozi2>>5) + 128, 0), 255);  //if(volume) OCR1AL = min(max((ozi2>>5) + ICR1L/2, 0), ICR1L);  // center and clip wrt PWM working range
#else
  if(volume) OCR1AL = (ozi1>>5) + 128;
  //if(volume) OCR1AL = min(max((ozi1>>5) + 128, 0), 255);  //if(volume) OCR1AL = min(max((ozi2>>5) + ICR1L/2, 0), ICR1L);  // center and clip wrt PWM working range
#endif
}
#endif

#ifdef SIMPLE_RX
volatile uint8_t admux[3];
static uint8_t rx_state = 0;

static struct rx {
  int16_t z1;
  int16_t za1;
  int16_t _z1;
  int16_t _za1;
} rx_inst[2];

void sdr_rx()
{
  static int16_t ocomb;
  static int16_t qh;

  uint8_t b = !(rx_state & 0x01);
  rx* p = &rx_inst[b];
  uint8_t _rx_state;
  int16_t ac;
  if(b){  // rx_state == 0, 2, 4, 6 -> I-stage
    ADMUX = admux[1];  // set MUX for next conversion
    ADCSRA |= (1 << ADSC);    // start next ADC conversion
    ac = ADC - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH

    //sdr_common
    static int16_t ozi1, ozi2;
    if(_init){ ocomb=0; ozi1 = 0; ozi2 = 0; } // hack
    // Output stage [25% CPU@R=4;Fs=62.5k]
    #define SECOND_ORDER_DUC 1
    #ifdef SECOND_ORDER_DUC
    ozi2 = ozi1 + ozi2;          // Integrator section
    #endif
    ozi1 = ocomb + ozi1;
    #ifdef SECOND_ORDER_DUC
    if(volume) OCR1AL = min(max((ozi2>>5) + 128, 0), 255);  //if(volume) OCR1AL = min(max((ozi2>>5) + ICR1L/2, 0), ICR1L);  // center and clip wrt PWM working range
    #else
    if(volume) OCR1AL = (ozi1>>5) + 128;
    //if(volume) OCR1AL = min(max((ozi1>>5) + 128, 0), 255);  //if(volume) OCR1AL = min(max((ozi2>>5) + ICR1L/2, 0), ICR1L);  // center and clip wrt PWM working range
    #endif
    // Only for I: correct I/Q sample delay by means of linear interpolation
    static int16_t prev_adc;
    int16_t corr_adc = (prev_adc + ac) / 2;
    prev_adc = ac;
    ac = corr_adc;
    _rx_state = ~rx_state;
  } else {
    ADMUX = admux[0];  // set MUX for next conversion
    ADCSRA |= (1 << ADSC);    // start next ADC conversion
    ac = ADC - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
    _rx_state = rx_state;
  }
    
  if(_rx_state & 0x02){  // rx_state == I: 0, 4  Q: 3, 7  1st stage: down-sample by 2
    int16_t _ac = ac + p->za1 + p->z1 * 2;           // 1st stage: FA + FB
    p->za1 = ac;
    if(_rx_state & 0x04){                   // rx_state == I: 0  Q:7   2nd stage: down-sample by 2
      int16_t ac2 = _ac + p->_za1 + p->_z1 * 2;              // 2nd stage: FA + FB
      p->_za1 = _ac;
      if(b){
        // post processing I and Q (down-sampled) results
        static int16_t v[8];
        v[7] = ac2 >> att2;
        
        i = v[0]; v[0] = v[1]; v[1] = v[2]; v[2] = v[3]; v[3] = v[4]; v[4] = v[5]; v[5] = v[6]; v[6] = v[7];  // Delay to match Hilbert transform on Q branch

        int16_t ac = i + qh;
        ac = slow_dsp(ac);

        // Output stage
        static int16_t ozd1, ozd2;
        if(_init){ ac = 0; ozd1 = 0; ozd2 = 0; _init = 0; } // hack: on first sample init accumlators of further stages (to prevent instability)
        #ifdef SECOND_ORDER_DUC
        int16_t od1 = ac - ozd1; // Comb section
        ocomb = od1 - ozd2;
        ozd2 = od1;
        #else
        ocomb = ac - ozd1; // Comb section
        #endif
        ozd1 = ac;
      } else {
        // Process Q (down-sampled) samples
        static int16_t v[16];
        v[15] = ac2 >> att2;

        for(uint8_t j = 0; j != 15; j++) v[j] = v[j + 1];
        q = v[7];
        qh = ((v[0] - v[14]) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 15) / 128 + (v[6] - v[8]) / 2; // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)
      }
    } else p->_z1 = _ac;
  } else p->z1 = ac;  // rx_state == I: 2, 6  Q: 1, 5

  rx_state++;
}
//#pragma GCC push_options
//#pragma GCC optimize ("Ofast")  // compiler-optimization for speed
//#pragma GCC pop_options  // end of DSP section
// */
#endif

ISR(TIMER2_COMPA_vect)  // Timer2 COMPA interrupt
{
  func_ptr();
#ifdef DEBUG
  numSamples++;
#endif
}

void adc_start(uint8_t adcpin, bool ref1v1, uint32_t fs)
{
#ifndef AUTO_ADC_BIAS
  DIDR0 |= (1 << adcpin); // disable digital input
#endif  
  ADCSRA = 0;             // clear ADCSRA register
  ADCSRB = 0;             // clear ADCSRB register
  ADMUX = 0;              // clear ADMUX register
  ADMUX |= (adcpin & 0x0f);    // set analog input pin
  ADMUX |= ((ref1v1) ? (1 << REFS1) : 0) | (1 << REFS0);  // set AREF=1.1V (Internal ref); otherwise AREF=AVCC=(5V)
  ADCSRA |= ((uint8_t)log2((uint8_t)(F_CPU / 13 / fs))) & 0x07;  // ADC Prescaler (for normal conversions non-auto-triggered): ADPS = log2(F_CPU / 13 / Fs) - 1; ADSP=0..7 resulting in resp. conversion rate of 1536, 768, 384, 192, 96, 48, 24, 12 kHz
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

void timer1_start(uint32_t fs)
{  // Timer 1: OC1A and OC1B in PWM mode
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11); // Clear OC1A/OC1B on compare match, set OC1A/OC1B at BOTTOM (non-inverting mode)
  TCCR1B |= (1 << CS10) | (1 << WGM13) | (1 << WGM12); // Mode 14 - Fast PWM;  CS10: clkI/O/1 (No prescaling)
  ICR1H = 0x00;
  ICR1L = min(255, (float)F_CPU / (float)fs - 0.5);  // PWM value range (fs>78431):  Fpwm = F_CPU / [Prescaler * (1 + TOP)]
  //TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10); // Clear OC1A/OC1B on compare match, set OC1A/OC1B at BOTTOM (non-inverting mode)
  //TCCR1B |= (1 << CS10) | (1 << WGM12); // Mode 5 - Fast PWM, 8-bit;  CS10: clkI/O/1 (No prescaling)
  OCR1AH = 0x00;
  OCR1AL = 0x00;  // OC1A (SIDETONE) PWM duty-cycle (span defined by ICR).
  OCR1BH = 0x00;
  OCR1BL = 0x00;  // OC1B (KEY_OUT) PWM duty-cycle (span defined by ICR).
}

void timer1_stop()
{
  OCR1AL = 0x00;
  OCR1BL = 0x00;
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Below a radio-specific implementation based on the above components (seperation of concerns)
//
// Feel free to replace it with your own custom radio implementation :-)

char blanks[] = "        ";
#define lcd_blanks() lcd.print(blanks);

#define N_FONTS  8
const byte fonts[N_FONTS][8] PROGMEM = {
{ 0b01000,  // 1; logo
  0b00100,
  0b01010,
  0b00101,
  0b01010,
  0b00100,
  0b01000,
  0b00000 },
{ 0b00000,  // 2; s-meter, 0 bars
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000 },
{ 0b10000,  // 3; s-meter, 1 bars
  0b10000,
  0b10000,
  0b10000,
  0b10000,
  0b10000,
  0b10000,
  0b10000 },
{ 0b10000,  // 4; s-meter, 2 bars
  0b10000,
  0b10100,
  0b10100,
  0b10100,
  0b10100,
  0b10100,
  0b10100 },
{ 0b10000,  // 5; s-meter, 3 bars
  0b10000,
  0b10101,
  0b10101,
  0b10101,
  0b10101,
  0b10101,
  0b10101 },
{ 0b01100,  // 6; vfo-a
  0b10010,
  0b11110,
  0b10010,
  0b10010,
  0b00000,
  0b00000,
  0b00000 },
{ 0b11100,  // 7; vfo-b
  0b10010,
  0b11100,
  0b10010,
  0b11100,
  0b00000,
  0b00000,
  0b00000 },
{ 0b00000,  // 8; tbd
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000 }
};

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

enum dsp_cap_t { ANALOG, DSP, SDR };
uint8_t dsp_cap = 0;
uint8_t ssb_cap = 0;

uint16_t analogSampleMic()
{
  uint16_t adc;
  noInterrupts();
  if(dsp_cap == SDR) digitalWrite(RX, LOW);  // disable RF input, only for SDR mod
  //si5351.SendRegister(SI_CLK_OE, 0b11111111); // CLK2_EN=0, CLK1_EN,CLK0_EN=0
  ADMUX = admux[2];  // set MUX for next conversion
  ADCSRA |= (1 << ADSC);    // start next ADC conversion
  for(;!(ADCSRA & (1 << ADIF)););  // wait until ADC conversion is completed
  if(dsp_cap == SDR) digitalWrite(RX, HIGH);  // enable RF input, only for SDR mod
  //si5351.SendRegister(SI_CLK_OE, 0b11111100); // CLK2_EN=0, CLK1_EN,CLK0_EN=1
  adc = ADC;
  interrupts();
  return adc;
}

volatile bool change = true;
volatile int32_t freq = 7074000;

int8_t smode = 1;

float dbm_max;
float smeter(float ref = 5)  //= 10*log(8000/2400)=5  ref to 2.4kHz BW.  plus some other calibration factor
{
  if(smode == 0){ // none, no s-meter
    return 0;
  }
  float rms = _absavg256 / 256.0; //sqrt(256.0);
  //if(dsp_cap == SDR) rms = (float)rms * 1.1 * (float)(1 << att2) / (1024.0 * (float)R * 4.0 * 100.0 * 40.0);   // 2 rx gain stages: rmsV = ADC value * AREF / [ADC DR * processing gain * receiver gain * audio gain]
  if(dsp_cap == SDR) rms = (float)rms * 1.1 * (float)(1 << att2) / (1024.0 * (float)R * 4.0 * 820.0 * 3.0/*??*/);   // 1 rx gain stage: rmsV = ADC value * AREF / [ADC DR * processing gain * receiver gain * audio gain]
  else               rms = (float)rms * 5.0 * (float)(1 << att2) / (1024.0 * (float)R * 2.0 * 100.0 * 120.0 / 1.750);
  float dbm = (10.0 * log10((rms * rms) / 50.0) + 30.0) - ref; //from rmsV to dBm at 50R
  dbm_max = max(dbm_max, dbm);
  static uint8_t cnt;
  cnt++;
  if((cnt % 8) == 0){

    if(smode == 1){ // dBm meter
      lcd.setCursor(9, 0); lcd.print((int16_t)dbm_max); lcd.print(F("dBm   "));
    }
    if(smode == 2){ // S-meter
      uint8_t s = (dbm_max < -63) ? ((dbm_max - -127) / 6) : (uint8_t)(dbm_max - -63 + 10) % 10;  // dBm to S
      lcd.setCursor(14, 0); if(s < 10){ lcd.print('S'); } lcd.print(s); 
    }
    dbm_max = -174.0 + 34.0;
  }
  if(smode == 3){ // S-bar
    int8_t s = (dbm < -63) ? ((dbm - -127) / 6) : (uint8_t)(dbm - -63 + 10) % 10;  // dBm to S
    lcd.setCursor(12, 0); 
    char tmp[5];
    for(uint8_t i = 0; i != 4; i++){ tmp[i] = max(2, min(5, s + 1)); s = s - 3; } tmp[4] = 0;
    lcd.print(tmp);
  }
  return dbm;
}

void start_rx()
{
  _init = 1;
  rx_state = 0;
  func_ptr = sdr_rx;  //enable RX DSP/SDR
  adc_start(2, true, F_ADC_CONV); admux[2] = ADMUX;
  if(dsp_cap == SDR){
    adc_start(0, !(att == 1)/*true*/, F_ADC_CONV); admux[0] = ADMUX;
    adc_start(1, !(att == 1)/*true*/, F_ADC_CONV); admux[1] = ADMUX;
  } else { // ANALOG, DSP
    adc_start(0, false, F_ADC_CONV); admux[0] = ADMUX; admux[1] = ADMUX;
  }
  timer1_start(78125);
  timer2_start(F_SAMP_RX);  
  TCCR1A &= ~(1 << COM1B1); digitalWrite(KEY_OUT, LOW); // disable KEY_OUT PWM
}

void switch_rxtx(uint8_t tx_enable){
  tx = tx_enable;
  TIMSK2 &= ~(1 << OCIE2A);  // disable timer compare interrupt
  //delay(1);
  noInterrupts();
  if(tx_enable){
    switch(mode){
      case USB:
      case LSB: func_ptr = dsp_tx; break;
      case CW:  func_ptr = dsp_tx_cw; break;
      case AM:  func_ptr = dsp_tx_am; break;
      case FM:  func_ptr = dsp_tx_fm; break;
    }
  } else func_ptr = sdr_rx;
  if((!dsp_cap) && (!tx_enable) && vox)  func_ptr = dummy; //hack: for SSB mode, disable dsp_rx during vox mode enabled as it slows down the vox loop too much!
  interrupts();
  if(tx_enable) ADMUX = admux[2];
  else _init = 1;
  rx_state = 0;
  if(tx_enable){
      digitalWrite(RX, LOW);  // TX (disable RX)
#ifdef NTX
      digitalWrite(NTX, LOW);  // TX (enable TX)
#endif
      lcd.setCursor(15, 1); lcd.print("T");
      si5351.SendRegister(SI_CLK_OE, 0b11111011); // CLK2_EN=1, CLK1_EN,CLK0_EN=0
      //if(!mox) TCCR1A &= ~(1 << COM1A1); // disable SIDETONE, prevent interference during TX
      OCR1AL = 0; // make sure SIDETONE is set to 0%
      TCCR1A |= (1 << COM1B1);  // enable KEY_OUT PWM
  } else {
      //TCCR1A |= (1 << COM1A1);  // enable SIDETONE
      TCCR1A &= ~(1 << COM1B1); digitalWrite(KEY_OUT, LOW); // disable KEY_OUT PWM, prevents interference during RX
      OCR1BL = 0; // make sure PWM (KEY_OUT) is set to 0%
      digitalWrite(RX, !(att == 2)); // RX (enable RX when attenuator not on)
#ifdef NTX
      digitalWrite(NTX, HIGH);  // RX (disable TX)
#endif
      si5351.SendRegister(SI_CLK_OE, 0b11111100); // CLK2_EN=0, CLK1_EN,CLK0_EN=1
      lcd.setCursor(15, 1); lcd.print((vox) ? "V" : "R");
  }
  OCR2A = (((float)F_CPU / (float)64) / (float)((tx_enable) ? F_SAMP_TX : F_SAMP_RX) + 0.5) - 1;
  TIMSK2 |= (1 << OCIE2A);  // enable timer compare interrupt TIMER2_COMPA_vect
}

#define CAL_IQ 1
#ifdef CAL_IQ
// RX I/Q calibration procedure: terminate with 50 ohm, enable CW filter, adjust R27, R24, R17 subsequently to its minimum side-band rejection value in dB
void calibrate_iq()
{
  smode = 1;
  lcd.setCursor(0, 0); lcd.print(blanks); lcd.print(blanks);
  digitalWrite(SIG_OUT, true); // loopback on
  si5351.freq(freq, 0, 90);  // RX in USB
  si5351.SendRegister(SI_CLK_OE, 0b11111000); // CLK2_EN=0, CLK1_EN,CLK0_EN=1
  float dbc;
  si5351.freq_calc_fast(+700); si5351.SendPLLBRegisterBulk(); delay(100);
  dbc = smeter();
  si5351.freq_calc_fast(-700); si5351.SendPLLBRegisterBulk(); delay(100);
  lcd.setCursor(0, 1); lcd.print("I-Q bal. 700Hz"); lcd.print(blanks);
  for(; !digitalRead(BUTTONS);){ wdt_reset(); smeter(dbc); } for(; digitalRead(BUTTONS);) wdt_reset();
  si5351.freq_calc_fast(+600); si5351.SendPLLBRegisterBulk(); delay(100);
  dbc = smeter();
  si5351.freq_calc_fast(-600); si5351.SendPLLBRegisterBulk(); delay(100);
  lcd.setCursor(0, 1); lcd.print("Phase Lo 600Hz"); lcd.print(blanks);
  for(; !digitalRead(BUTTONS);){ wdt_reset(); smeter(dbc); } for(; digitalRead(BUTTONS);) wdt_reset();
  si5351.freq_calc_fast(+800); si5351.SendPLLBRegisterBulk(); delay(100);
  dbc = smeter();
  si5351.freq_calc_fast(-800); si5351.SendPLLBRegisterBulk(); delay(100);
  lcd.setCursor(0, 1); lcd.print("Phase Hi 800Hz"); lcd.print(blanks);
  for(; !digitalRead(BUTTONS);){ wdt_reset(); smeter(dbc); } for(; digitalRead(BUTTONS);) wdt_reset();

  lcd.setCursor(9, 0); lcd.print(blanks);  // cleanup dbmeter
  digitalWrite(SIG_OUT, false); // loopback off
  si5351.SendRegister(SI_CLK_OE, 0b11111100); // CLK2_EN=0, CLK1_EN,CLK0_EN=1
  change = true;  //restore original frequency setting
}
#endif

int8_t prev_bandval = 2;
int8_t bandval = 2;
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
  lcd.setCursor(stepsize+1, 1);  // display stepsize with cursor
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
  lcd.setCursor(0, 1); lcd.print(F("Power-off 73 :-)")); lcd_blanks();

  MCUSR = ~(1<<WDRF);  // MSY be done before wdt_disable()
  wdt_disable();   // WDTON Fuse High bit need to be 1 (0xD1), if NOT it will override and set WDE=1; WDIE=0, meaning MCU will reset when watchdog timer is zero, and this seems to happen when wdt_disable() is called
  
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
  interrupts();
  sleep_bod_disable();
  //MCUCR |= (1<<BODS) | (1<<BODSE);  // turn bod off by settings BODS, BODSE; note BODS is reset after three clock-cycles, so quickly go to sleep before it is too late
  //MCUCR &= ~(1<<BODSE);  // must be done right before sleep
  sleep_cpu();  // go to sleep mode, wake-up by either INT0, INT1, Pin Change, TWI Addr Match, WDT, BOD
  sleep_disable();

  //void(* reset)(void) = 0; reset();   // soft reset by calling reset vector (does not reset registers to defaults)
  do { wdt_enable(WDTO_15MS); for(;;); } while(0);  // soft reset by trigger watchdog timeout
}

void show_banner(){
  lcd.setCursor(0, 0);
  lcd.print(F("QCX"));
  const char* cap_label[] = { "SSB", "DSP", "SDR" };
  if(ssb_cap || dsp_cap){ lcd.print(F("-")); lcd.print(cap_label[dsp_cap]); }
  lcd.print(F("\x01 ")); lcd_blanks();
}

const char* mode_label[5] = { "LSB", "USB", "CW ", "AM ", "FM " };

void display_vfo(uint32_t f){
  lcd.setCursor(0, 1);
  lcd.print('\x06');  // VFO A/B

  uint32_t scale=10e6;  // VFO frequency
  if(f/scale == 0){ lcd.print(' '); scale/=10; }  // Initial space instead of zero
  for(; scale!=1; f%=scale, scale/=10){
    lcd.print(f/scale);
    if(scale == (uint32_t)1e3 || scale == (uint32_t)1e6) lcd.print(',');  // Thousands separator
  }
  
  lcd.print(" "); lcd.print(mode_label[mode]); lcd.print("  ");
  lcd.setCursor(15, 1); lcd.print("R");
}

volatile uint8_t event;
volatile uint8_t menumode = 0;  // 0=not in menu, 1=selects menu item, 2=selects parameter value
volatile int8_t menu = 0;  // current parameter id selected in menu

#define pgm_cache_item(addr, sz) byte _item[sz]; memcpy_P(_item, addr, sz);  // copy array item from PROGMEM to SRAM
#define get_version_id() ((VERSION[0]-'1') * 2048 + ((VERSION[2]-'0')*10 + (VERSION[3]-'0')) * 32 +  ((VERSION[4]) ? (VERSION[4] - 'a' + 1) : 0) * 1)  // converts VERSION string with (fixed) format "9.99z" into uint16_t (max. values shown here, z may be removed) 

uint8_t eeprom_version;
#define EEPROM_OFFSET 0x150  // avoid collision with QCX settings, overwrites text settings though
int eeprom_addr;

// Support functions for parameter and menu handling
enum action_t { UPDATE, UPDATE_MENU, LOAD, SAVE, SKIP };
template<typename T> void paramAction(uint8_t action, T& value, const __FlashStringHelper* menuid, const __FlashStringHelper* label, const char* enumArray[], int32_t _min, int32_t _max, bool continuous){
  switch(action){
    case UPDATE:
    case UPDATE_MENU:
      if(((int32_t)value + encoder_val) < _min) value = (continuous) ? _max : _min; 
      else value += encoder_val;
      encoder_val = 0;
      if(continuous) value = (value % (_max+1));
      value = max(_min, min((int32_t)value, _max));
      if(action == UPDATE_MENU){
        lcd.setCursor(0, 0);
        lcd.print(menuid); lcd.print(' ');
        lcd.print(label); lcd_blanks(); lcd_blanks();
        lcd.setCursor(0, 1); // value on next line
      } else { // UPDATE (not in menu)
        lcd.setCursor(0, 1); lcd.print(label); lcd.print(F(": "));
      }
      if(enumArray == NULL){
        if((_min < 0) && (value >= 0)) lcd.print('+');
        lcd.print(value);
      } else {
        lcd.print(enumArray[value]);
      }
      lcd_blanks(); lcd_blanks();
      //if(action == UPDATE) paramAction(SAVE, value, menuid, label, enumArray, _min, _max, continuous, init_val);
      break;
    case LOAD:
      for(uint8_t* ptr = (uint8_t *) &value, n = sizeof(value); n; --n) *ptr++ = eeprom_read_byte((uint8_t *)eeprom_addr++);
      break;
    case SAVE:
      for(uint8_t* ptr = (uint8_t *) &value, n = sizeof(value); n; --n) eeprom_write_byte((uint8_t *)eeprom_addr++, *ptr++);
      break;
    case SKIP:
      eeprom_addr += sizeof(value);
      break;
  }
}
uint32_t save_event_time = 0;
uint32_t sec_event_time = 0;

static uint8_t pwm_min = 0;    // PWM value for which PA reaches its minimum: 29 when C31 installed;   0 when C31 removed;   0 for biasing BS170 directly
static uint8_t pwm_max = 220;  // PWM value for which PA reaches its maximum: 96 when C31 installed; 255 when C31 removed; 220 for biasing BS170 directly

const char* offon_label[2] = {"OFF", "ON"};
const char* filt_label[N_FILT+1] = { "Full", "4000", "2500", "1700", "500", "200", "100", "50" };
const char* band_label[N_BANDS] = { "80m", "60m", "40m", "30m", "20m", "17m", "15m", "12m", "10m", "6m", "4m" };

#define _N(a) sizeof(a)/sizeof(a[0])

#define N_PARAMS 25  // number of (visible) parameters
#define N_ALL_PARAMS (N_PARAMS+2)  // number of parameters

enum params_t {ALL, VOLUME, MODE, FILTER, BAND, STEP, AGC, NR, ATT, ATT2, SMETER, CWDEC, CWOFF, VOX, VOXGAIN, MOX, DRIVE, SIFXTAL, PWM_MIN, PWM_MAX, CALIB, SR, CPULOAD, PARAM_A, PARAM_B, PARAM_C, FREQ, VERS};

void paramAction(uint8_t action, uint8_t id = ALL)  // list of parameters
{
  if((action == SAVE) || (action == LOAD)){
    eeprom_addr = EEPROM_OFFSET;
    for(uint8_t _id = 1; _id < id; _id++) paramAction(SKIP, _id);
  }
  if(id == ALL) for(id = 1; id != N_ALL_PARAMS+1; id++) paramAction(action, id);  // for all parameters
  
  const char* stepsize_label[10] = { "10M", "1M", "0.5M", "100k", "10k", "1k", "0.5k", "100", "10", "1" };
  const char* att_label[] = { "0dB", "-13dB", "-20dB", "-33dB", "-40dB", "-53dB", "-60dB", "-73dB" };
  const char* smode_label[4] = { "OFF", "dBm", "S", "S-bar" };
  switch(id){
    // Visible parameters
    case VOLUME:  paramAction(action, volume, F("1.1"), F("Volume"), NULL, -1, 16, false); break;
    case MODE:    paramAction(action, mode, F("1.2"), F("Mode"), mode_label, 0, _N(mode_label) - 1, true); break;
    case FILTER:  paramAction(action, filt, F("1.3"), F("Filter BW"), filt_label, 0, _N(filt_label) - 1, false); break;
    case BAND:    paramAction(action, bandval, F("1.4"), F("Band"), band_label, 0, _N(band_label) - 1, false); break;
    case STEP:    paramAction(action, stepsize, F("1.5"), F("Tune Rate"), stepsize_label, 0, _N(stepsize_label) - 1, false); break;
    case AGC:     paramAction(action, agc, F("1.6"), F("AGC"), offon_label, 0, 1, false); break;
    case NR:      paramAction(action, nr, F("1.7"), F("NR"), NULL, 0, 8, false); break;
    case ATT:     paramAction(action, att, F("1.8"), F("ATT"), att_label, 0, 7, false); break;
    case ATT2:    paramAction(action, att2, F("1.9"), F("ATT2"), NULL, 0, 16, false); break;
    case SMETER:  paramAction(action, smode, F("1.10"), F("S-meter"), smode_label, 0, _N(smode_label) - 1, false); break;
    case CWDEC:   paramAction(action, cwdec, F("2.1"), F("CW Decoder"), offon_label, 0, 1, false); break;
    case CWOFF:   paramAction(action, cw_offset, F("2.2"), F("CW Offset"), NULL, 300, 2000, false); break;
    case VOX:     paramAction(action, vox, F("3.1"), F("VOX"), offon_label, 0, 1, false); break;
    case VOXGAIN: paramAction(action, vox_thresh, F("3.2"), F("VOX Level"), NULL, 0, 255, false); break;
    case MOX:     paramAction(action, mox, F("3.3"), F("MOX"), NULL, 0, 4, false); break;
    case DRIVE:   paramAction(action, drive, F("3.4"), F("TX Drive"), NULL, 0, 8, false); break;
    case SIFXTAL: paramAction(action, si5351.fxtal, F("8.1"), F("Ref freq"), NULL, 14000000, 28000000, false); break;
    case PWM_MIN: paramAction(action, pwm_min, F("8.2"), F("PA Bias min"), NULL, 0, 255, false); break;
    case PWM_MAX: paramAction(action, pwm_max, F("8.3"), F("PA Bias max"), NULL, 0, 255, false); break;
#ifdef CAL_IQ
    case CALIB:   if(dsp_cap != SDR) paramAction(action, param_c, F("8.4"), F("IQ Test/Cal."), NULL, 0, 0, false); break;
#endif
#ifdef DEBUG
    case SR:      paramAction(action, sr, F("9.1"), F("Sample rate"), NULL, -2147483648, 2147483647, false); break;
    case CPULOAD: paramAction(action, cpu_load, F("9.2"), F("CPU load %"), NULL, -2147483648, 2147483647, false); break;
    case PARAM_A: paramAction(action, param_a, F("9.3"), F("Param A"), NULL, 0, 65535, false); break;
    case PARAM_B: paramAction(action, param_b, F("9.4"), F("Param B"), NULL, -32768, 32767, false); break;
    case PARAM_C: paramAction(action, param_c, F("9.5"), F("Param C"), NULL, -32768, 32767, false); break;
#endif
    // Invisible parameters
    case FREQ:    paramAction(action, freq, NULL, NULL, NULL, 0, 0, false); break;
    case VERS:    paramAction(action, eeprom_version, NULL, NULL, NULL, 0, 0, false); break;
  }
}

void initPins(){  
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
//#define ONEBUTTON  1
#ifdef ONEBUTTON
  pinMode(BUTTONS, INPUT_PULLUP);  // rotary button
#else
  pinMode(BUTTONS, INPUT);  // L/R/rotary button
#endif
  pinMode(DIT, INPUT_PULLUP);
  //pinMode(DAH, INPUT);  
  pinMode(DAH, INPUT_PULLUP); // Could this replace D4?

  digitalWrite(AUDIO1, LOW);  // when used as output, help can mute RX leakage into AREF
  digitalWrite(AUDIO2, LOW);
  pinMode(AUDIO1, INPUT);
  pinMode(AUDIO2, INPUT);

#ifdef NTX
  digitalWrite(NTX, HIGH);
  pinMode(NTX, OUTPUT);
#endif
}

void setup()
{
  digitalWrite(KEY_OUT, LOW);  // for safety: to prevent exploding PA MOSFETs, in case there was something still biasing them.

#ifdef DEBUG
  uint8_t mcusr = MCUSR;
  MCUSR = 0;
  //wdt_disable();
  wdt_enable(WDTO_4S);  // Enable watchdog

  // Benchmark dsp_tx() ISR (this needs to be done in beginning of setup() otherwise when VERSION containts 5 chars, mis-alignment impact performance by a few percent)
  rx_state = 0;
  uint32_t t0, t1;
  func_ptr = dsp_tx;
  t0 = micros();
  TIMER2_COMPA_vect();
  //func_ptr();
  t1 = micros();
  float load_tx = (t1 - t0) * F_SAMP_TX * 100.0 / 1000000.0;
  // benchmark sdr_rx() ISR
  func_ptr = sdr_rx;
  rx_state = 8;
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
#endif
  ADMUX = (1 << REFS0);  // restore reference voltage AREF (5V)

  // disable external interrupts
  PCICR = 0;
  PCMSK0 = 0;
  PCMSK1 = 0;
  PCMSK2 = 0;

  encoder_setup();

  initPins();

  //Init si5351
  si5351.powerDown();  // Disable all (used) outputs

  delay(100);           // at least 40ms after power rises above 2.7V before sending commands
  lcd.begin(16, 2);  // Init LCD
  for(i = 0; i != N_FONTS; i++){  // Init fonts
    pgm_cache_item(fonts[i], 8);
    lcd.createChar(0x01 + i, /*fonts[i]*/_item);
  }

  // Test if QCX has DSP/SDR capability: SIDETONE output disconnected from AUDIO2
  si5351.SendRegister(SI_CLK_OE, 0b11111111); // Mute QSD: CLK2_EN=CLK1_EN,CLK0_EN=0  
  digitalWrite(RX, HIGH);  // generate pulse on SIDETONE and test if it can be seen on AUDIO2
  delay(1); // settle
  digitalWrite(SIDETONE, LOW);
  int16_t v1 = analogRead(AUDIO2);
  digitalWrite(SIDETONE, HIGH);
  int16_t v2 = analogRead(AUDIO2);
  digitalWrite(SIDETONE, LOW);
  dsp_cap = !(abs(v2 - v1) > (0.05 * 1024.0 / 5.0));  // DSP capability?
  if(dsp_cap){  // Test if QCX has SDR capability: AUDIO2 is disconnected from AUDIO1  (only in case of DSP capability)
    delay(400); wdt_reset(); // settle:  the following test only works well 400ms after startup
    v1 = analogRead(AUDIO1);
    digitalWrite(AUDIO2, HIGH);   // generate pulse on AUDIO2 and test if it can be seen on AUDIO1
    pinMode(AUDIO2, OUTPUT);
    delay(1);
    digitalWrite(AUDIO2, LOW); 
    delay(1);
    digitalWrite(AUDIO2, HIGH);
    v2 = analogRead(AUDIO1);
    pinMode(AUDIO2, INPUT);
    if(!(abs(v2 - v1) > (0.010 * 1024.0 / 5.0))) dsp_cap = SDR;  // SDR capacility?
  }
  // Test if QCX has SSB capability: DAH is connected to DVM
  delay(1); // settle
  pinMode(DAH, OUTPUT);
  digitalWrite(DAH, LOW);
  v1 = analogRead(DVM);
  digitalWrite(DAH, HIGH);
  v2 = analogRead(DVM);
  digitalWrite(DAH, LOW);
  pinMode(DAH, INPUT_PULLUP);
  ssb_cap = (abs(v2 - v1) > (0.05 * 1024.0 / 5.0));  // SSB capability?

  //ssb_cap = 0; dsp_cap = 0;  // force standard QCX capability
  //ssb_cap = 1; dsp_cap = 0;  // force SSB and standard QCX-RX capability
  //ssb_cap = 1; dsp_cap = 1;  // force SSB and DSP capability
  //ssb_cap = 1; dsp_cap = 2;  // force SSB and SDR capability

  show_banner();
  lcd.setCursor(7, 0); lcd.print(F(" R")); lcd.print(F(VERSION)); lcd_blanks();

#ifdef DEBUG
  if((mcusr & WDRF) && (!(mcusr & EXTRF)) && (!(mcusr & BORF))){
    lcd.setCursor(0, 1); lcd.print(F("!!Watchdog RESET")); lcd_blanks();
    delay(1500); wdt_reset();
  }
  if((mcusr & BORF) && (!(mcusr & WDRF))){
    lcd.setCursor(0, 1); lcd.print(F("!!Brownout RESET")); lcd_blanks();  // Brow-out reset happened, CPU voltage not stable or make sure Brown-Out threshold is set OK (make sure E fuse is set to FD)
    delay(1500); wdt_reset();
  }
  if(mcusr & PORF){
    lcd.setCursor(0, 1); lcd.print(F("!!Power-On RESET")); lcd_blanks();
    delay(1500); wdt_reset();
  }
  /*if(mcusr & EXTRF){
  lcd.setCursor(0, 1); lcd.print(F("Power-On")); lcd_blanks();
    delay(1); wdt_reset();
  }*/
  
  // Measure CPU loads
  if(!(load_tx <= 100.0)){
    lcd.setCursor(0, 1); lcd.print(F("!!CPU_tx=")); lcd.print(load_tx); lcd.print(F("%")); lcd_blanks();
    delay(1500); wdt_reset();
  }

  if(!(load_rx_avg <= 100.0)){
    lcd.setCursor(0, 1); lcd.print(F("!!CPU_rx")); lcd.print(F("=")); lcd.print(load_rx_avg); lcd.print(F("%")); lcd_blanks();
    delay(1500); wdt_reset();
    // and specify individual timings for each of the eight alternating processing functions:
    for(i = 1; i != 8; i++){
      if(!(load_rx[i] <= 100.0))
      {
        lcd.setCursor(0, 1); lcd.print(F("!!CPU_rx")); lcd.print(i); lcd.print(F("=")); lcd.print(load_rx[i]); lcd.print(F("%")); lcd_blanks();
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
  if(!(vdd > 4.8 && vdd < 5.2)){
    lcd.setCursor(0, 1); lcd.print(F("!!V5.0=")); lcd.print(vdd); lcd.print(F("V")); lcd_blanks();
    delay(1500); wdt_reset();
  }

  // Measure VEE (+3.3V); should be ~3.3V
  float vee = (float)analogRead(SCL) * 5.0 / 1024.0;
  if(!(vee > 3.2 && vee < 3.8)){
    lcd.setCursor(0, 1); lcd.print(F("!!V3.3=")); lcd.print(vee); lcd.print(F("V")); lcd_blanks();
    delay(1500); wdt_reset();
  }

  // Measure AVCC via AREF and using internal 1.1V reference fed to ADC; should be ~5V
  analogRead(6); // setup almost proper ADC readout
  bitSet(ADMUX, 3); // Switch to channel 14 (Vbg=1.1V)
  delay(1); // delay improves accuracy
  bitSet(ADCSRA, ADSC);
  for(; bit_is_set(ADCSRA, ADSC););
  float avcc = 1.1 * 1023.0 / ADC;
  if(!(avcc > 4.6 && avcc < 5.1)){
    lcd.setCursor(0, 1); lcd.print(F("!!Vavcc=")); lcd.print(avcc); lcd.print(F("V")); lcd_blanks();
    delay(1500); wdt_reset();
  }

  // Report no SSB capability
  if(!ssb_cap){
    lcd.setCursor(0, 1); lcd.print(F("No MIC input...")); lcd_blanks();
    delay(300); wdt_reset();
  }

  // Measure DVM bias; should be ~VAREF/2
  float dvm = (float)analogRead(DVM) * 5.0 / 1024.0;
  if((ssb_cap) && !(dvm > 1.8 && dvm < 3.2)){
    lcd.setCursor(0, 1); lcd.print(F("!!Vadc2=")); lcd.print(dvm); lcd.print(F("V")); lcd_blanks();
    delay(1500); wdt_reset();
  }

  // Measure AUDIO1, AUDIO2 bias; should be ~VAREF/2
  if(dsp_cap == SDR){
    float audio1 = (float)analogRead(AUDIO1) * 5.0 / 1024.0;
    if(!(audio1 > 1.8 && audio1 < 3.2)){
      lcd.setCursor(0, 1); lcd.print(F("!!Vadc0=")); lcd.print(dvm); lcd.print(F("V")); lcd_blanks();
      delay(1500); wdt_reset();
    }
    float audio2 = (float)analogRead(AUDIO2) * 5.0 / 1024.0;
    if(!(audio2 > 1.8 && audio2 < 3.2)){
      lcd.setCursor(0, 1); lcd.print(F("!!Vadc1=")); lcd.print(dvm); lcd.print(F("V")); lcd_blanks();
      delay(1500); wdt_reset();
    }
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
    lcd.setCursor(0, 1); lcd.print(F("i2cspeed=")); lcd.print(speed); lcd.print(F("kbps")); lcd_blanks();
    delay(1500); wdt_reset();
  }

  // Measure I2C Bit-Error Rate (BER); should be error free for a thousand random bulk PLLB writes
  si5351.freq(freq, 0, 90);
  wdt_reset();
  uint16_t i2c_error = 0;  // number of I2C byte transfer errors
  for(i = 0; i != 1000; i++){
    si5351.freq_calc_fast(i);
    //for(int j = 3; j != 8; j++) si5351.pll_regs[j] = rand();
    si5351.SendPLLBRegisterBulk();
    #define SI_SYNTH_PLL_B 34
    for(int j = 3; j != 8; j++) if(si5351.RecvRegister(SI_SYNTH_PLL_B + j) != si5351.pll_regs[j]) i2c_error++;
  }
  if(i2c_error){
    lcd.setCursor(0, 1); lcd.print(F("!!BER_i2c=")); lcd.print(i2c_error); lcd_blanks();
    delay(1500); wdt_reset();
  }
#endif

  drive = 4;  // Init settings
  if(!ssb_cap){ mode = CW; filt = 4; stepsize = STEP_500; }
  if(dsp_cap != SDR) pwm_max = 255; // implies that key-shaping circuit is probably present, so use full-scale
  if(dsp_cap) cw_offset = 325; else cw_offset = 700;

  // Load parameters from EEPROM, reset to factory defaults when stored values are from a different version
  paramAction(LOAD, VERS);
  if(eeprom_version == get_version_id()){  // version signature in EEPROM corresponds with this firmware?
    paramAction(LOAD);  // load all parameters
  } else {
    eeprom_version = get_version_id();
    //for(int n = 0; n != 1024; n++){ eeprom_write_byte((uint8_t *) n, 0); wdt_reset(); } //clean EEPROM
    //eeprom_write_dword((uint32_t *)EEPROM_OFFSET/3, 0x000000);
    paramAction(SAVE);  // save default parfameter values
    lcd.setCursor(0, 1); lcd.print(F("Reset settings.."));
    delay(500); wdt_reset();
  }
  si5351.prev_pll_freq = 0;  // enforce PLL reset
  change = true;
  prev_bandval = bandval;

  if(!dsp_cap) volume = 0;  // mute volume for unmodified QCX receiver

  for(uint16_t i = 0; i != 256; i++)    // refresh LUT based on pwm_min, pwm_max
    lut[i] = (float)i / ((float)255 / ((float)pwm_max - (float)pwm_min)) + pwm_min;

  show_banner();  // remove release number

  start_rx();

// #define CAT 1
#ifdef CAT
  Serial.begin(7680); // 9600 baud corrected for F_CPU=20M
#endif
}

void print_char(uint8_t in){  // Print char in second line of display and scroll right.
  for(int i = 0; i!= 15; i++) out[i] = out[i+1];
  out[15] = in;
  out[16] = '\0';
  cw_event = true;
}

static char cat[20];  // temporary here
static uint8_t nc = 0;
static uint16_t cat_cmd = 0;

void parse_cat(uint8_t in){   // TS480 CAT protocol:  https://www.kenwood.com/i/products/info/amateur/ts_480/pdf/ts_480_pc.pdf
  if(nc == 0) cat_cmd = in << 8;
  if(nc == 1) cat_cmd += in;
  if(in == ';'){  // end of cat command -> parse and process

    switch(cat_cmd){
      case 'I'<<8|'F': Serial.write("IF00010138000     +00000000002000000 ;"); lcd.setCursor(0, 0); lcd.print("IF"); break;
    }
    nc = 0;       // reset
  } else {
    if(nc < (sizeof(cat) - 1)) cat[nc++] = in; // buffer and count-up
  }
}

void loop()
{
#ifdef CAT
  if(Serial.available() > 0){
    uint8_t in = Serial.read();
    print_char(in);
    parse_cat(in);
  }
#endif
  
  delay(1);

  if(millis() > sec_event_time){
    sec_event_time = millis() + 1000;  // schedule time next second
#ifndef OLED
    lcd.begin();  // fast LCD re-init (in case LCD has been taken out and placed back when power-on)
#endif
  }

  if(menumode == 0){
    smeter();
    if(!((mode == CW) && cw_event)) stepsize_showcursor();
  }

  if(cw_event){
    cw_event = false;
    lcd.setCursor(0, 1); lcd.print(out);
  }

  if(!digitalRead(DIT)){
    switch_rxtx(1);
    for(; !digitalRead(DIT);){ //until released
      wdt_reset();
    }
    switch_rxtx(0);
  }
  enum event_t { BL=0x10, BR=0x20, BE=0x30, SC=0x01, DC=0x02, PL=0x04, PT=0x0C }; // button-left, button-right and button-encoder; single-click, double-click, push-long, push-and-turn
#ifdef ONEBUTTON
  uint8_t inv = 1;
#else
  uint8_t inv = 0;
#endif
  if(inv ^ digitalRead(BUTTONS)){   // Left-/Right-/Rotary-button (while not already pressed)
    if(!(event & PL)){  // hack: if there was long-push before, then fast forward
      uint16_t v = analogSafeRead(BUTTONS);
      event = SC;
      int32_t t0 = millis();
      for(; inv ^ digitalRead(BUTTONS);){ // until released or long-press
        if((millis() - t0) > 300){ event = PL; break; }
        wdt_reset();
      }
      delay(10); //debounce
      for(; (event != PL) && ((millis() - t0) < 500);){ // until 2nd press or timeout
        if(inv ^ digitalRead(BUTTONS)){ event = DC; break; }
        wdt_reset();
      }
      for(; inv ^ digitalRead(BUTTONS);){ // until released, or encoder is turned while longpress
        if(encoder_val && event == PL){ event = PT; break; }
        wdt_reset();
      }  // Max. voltages at ADC3 for buttons L,R,E: 3.76V;4.55V;5V, thresholds are in center
      event |= (v < (4.2 * 1024.0 / 5.0)) ? BL : (v < (4.8 * 1024.0 / 5.0)) ? BR : BE; // determine which button pressed based on threshold levels
    } else {  // hack: fast forward handling
      event = (event&0xf0) | ((encoder_val) ? PT : PL);  // only alternate bewteen push-long/turn when applicable
    }
    switch(event){
      case BL|PL:  // Called when menu button released
        menumode = 2;
        //calibrate_predistortion();
        //powermeter();
        //test_tx_amp();
        break;
      case BL|PT:
        menumode = 1;
        //if(menu == 0) menu = 1;
        break;
      case BL|SC:
        //calibrate_iq();
        int8_t _menumode;
        if(menumode == 0){ _menumode = 1; if(menu == 0) menu = 1; }  // short left-click while in default screen: enter menu mode
        if(menumode == 1){ _menumode = 2; }                          // short left-click while in menu: enter value selection screen
        if(menumode == 2){ _menumode = 0; show_banner(); change = true; paramAction(SAVE, menu); } // short left-click while in value selection screen: save, and return to default screen
        menumode = _menumode;
        break;
      case BL|DC:
        //powerDown();
        /*lcd.setCursor(0, 1); lcd.print(F("Pause")); lcd_blanks();
        for(; !digitalRead(BUTTONS);){ // while in VOX mode
          wdt_reset();  // until 2nd press
          delay(300);
        }*/
        break;
      case BR|SC:
        if(!menumode){
          encoder_val = 1;
          paramAction(UPDATE, MODE); // Mode param //paramAction(UPDATE, mode, NULL, F("Mode"), mode_label, 0, _N(mode_label), true);
          if(mode != CW) stepsize = STEP_1k; else stepsize = STEP_500;
          //if(mode > FM) mode = LSB;
          if(mode > CW) mode = LSB;  // skip all other modes (only LSB, USB, CW)
          if(mode == CW) filt = 4; else filt = 0;
          paramAction(SAVE, MODE); 
          paramAction(SAVE, FILTER);
          si5351.prev_pll_freq = 0;  // enforce PLL reset
          change = true;
        } else {
          if(menumode == 1){ menumode = 0; show_banner(); change = true; }  // short right-click while in menu: enter value selection screen
          if(menumode == 2){ menumode = 1; change = true; paramAction(SAVE, menu); } // short right-click while in value selection screen: save, and return to menu screen
        }
        break;
      case BR|DC:
        //encoder_val = 1; paramAction(UPDATE, drive, NULL, F("Drive"), NULL, 0, 8, true);
        filt++;
        _init = true;
        if(mode == CW && filt > N_FILT) filt = 4;
        if(mode == CW && (filt == 5 || filt == 6) && stepsize < STEP_100) stepsize = STEP_100; // for CW BW 200, 100      -> step = 100 Hz
        if(mode == CW && filt == 7 && stepsize < STEP_10) stepsize = STEP_10;                  // for CW BW 50 -> step = 10 Hz
        if(mode != CW && filt > 3) filt = 0;
        encoder_val = 0; 
        paramAction(UPDATE, FILTER);
        paramAction(SAVE, FILTER);
        wdt_reset(); delay(1500); wdt_reset();
        change = true; // refresh display
        break;
      case BR|PL:
      {
        #ifdef SIMPLE_RX
        // Experiment: ISR-less sdr_rx():
        smode = 0;
        TIMSK2 &= ~(1 << OCIE2A);  // disable timer compare interrupt
        delay(100);
        lcd.setCursor(15, 1); lcd.print("X");
        static uint8_t x = 0;
        uint32_t next = 0;
        for(;;){
          func_ptr();
        #ifdef DEBUG
          numSamples++;
        #endif
          if(!rx_state){
            x++;
            if(x > 16){
              loop();
              //lcd.setCursor(9, 0); lcd.print((int16_t)100); lcd.print(F("dBm   "));  // delays are taking too long!
              x= 0;
            }
          }
          //for(;micros() < next;);  next = micros() + 16;   // sync every 1000000/62500=16ms (or later if missed)
        } //
        #endif //SIMPLE_RX
               
        //int16_t x = 0;
        lcd.setCursor(15, 1); lcd.print("V");
        for(; !digitalRead(BUTTONS);){ // while in VOX mode
          
          int16_t in = analogSampleMic() - 512;
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
          //x = max(x, abs(v[15]) );
          //lcd.setCursor(0, 1); lcd.print(x); lcd_blanks();
          //lcd.setCursor(0, 1); lcd.print(_amp); lcd_blanks();
          if(_amp > vox_thresh){            // workaround for RX noise leakage to AREF  
            for(j = 0; j != 16; j++) v[j] = 0;  // clean-up
            switch_rxtx(1);
            vox = 1; tx = 255; //kick
            delay(1);
            for(; tx && !digitalRead(BUTTONS); ) wdt_reset(); // while in tx triggered by vox
            switch_rxtx(0);
            delay(1);
            vox = 0;
            continue;  // skip the rest for the moment
          }    
          wdt_reset();
        }
      }
        lcd.setCursor(15, 1); lcd.print("R");
        break;
      case BR|PT: break;
      case BE|SC:
        if(!menumode)
          stepsize_change(+1);
        else {
          int8_t _menumode;
          if(menumode == 1){ _menumode = 2; }  // short encoder-click while in menu: enter value selection screen
          if(menumode == 2){ _menumode = 1; change = true; paramAction(SAVE, menu); } // short encoder-click while in value selection screen: save, and return to menu screen
          menumode = _menumode;
        }
        break;
      case BE|DC:
        delay(100);
        bandval++;
        if(bandval >= N_BANDS) bandval = 0;
        stepsize = STEP_1k;
        change = true;
        break;
      case BE|PL: stepsize_change(-1); break;
      case BE|PT:
          for(; digitalRead(BUTTONS);){ // process encoder changes until released
          wdt_reset();
          if(dsp_cap && encoder_val){
            paramAction(UPDATE, VOLUME);
            paramAction(SAVE, VOLUME);
            if(volume < 0) powerDown();  // powerDown when volume < 0
          }
        }
        change = true; // refresh display
        break;
    }
  } else event = 0;  // no button pressed: reset event

  if(menumode == 1){
    menu += encoder_val;   // Navigate through menu of parameters and values
    encoder_val = 0;
    menu = max(1 /* 0 */, min(menu, N_PARAMS));
  }

  bool param_change = (encoder_val != 0);
  if(menumode != 0){  // Show parameter and value
    if(menu != 0){
      paramAction(UPDATE_MENU, menu);  // update param with encoder change and display
    } else {
      menumode = 0; show_banner();  // while scrolling through menu: menu item 0 goes back to main console
      change = true; // refresh freq display (when menu = 0)
    }
    if(menumode == 2){
      if(param_change){
        lcd.setCursor(0, 1); lcd.cursor(); delay(10); // edits menu item value; make cursor visible
        if(menu == MODE){ // post-handling Mode parameter
          delay(100);
          change = true;
          si5351.prev_pll_freq = 0;  // enforce PLL reset
          // make more generic: 
          if(mode != CW) stepsize = STEP_1k; else stepsize = STEP_500;
          if(mode == CW) filt = 4; else filt = 0;
        }
        if(menu == ATT){ // post-handling ATT parameter
          if(dsp_cap == SDR){
            adc_start(0, !(att & 0x01), F_ADC_CONV); admux[0] = ADMUX;  // att bit 0 ON: attenuate -13dB by changing ADC AREF (full-scale range) from 1V1 to 5V
            adc_start(1, !(att & 0x01), F_ADC_CONV); admux[1] = ADMUX;
          }
          digitalWrite(RX, !(att & 0x02)); // att bit 1 ON: attenuate -20dB by disabling RX line, switching Q5 (antenna input switch) into 100k resistence
          pinMode(AUDIO1, (att & 0x04) ? OUTPUT : INPUT); // att bit 2 ON: attenuate -40dB by terminating ADC inputs with 10R
          pinMode(AUDIO2, (att & 0x04) ? OUTPUT : INPUT);
        }
        if(menu == SIFXTAL){
          change = true;
        }
        if((menu == PWM_MIN) || (menu == PWM_MAX)){
          for(uint16_t i = 0; i != 256; i++)    // refresh LUT based on pwm_min, pwm_max
            lut[i] = (float)i / ((float)255 / ((float)pwm_max - (float)pwm_min)) + pwm_min;
            //lut[i] = min(pwm_max, (float)106*log(i) + pwm_min);  // compressed microphone output: drive=0, pwm_min=115, pwm_max=220
        }
#ifdef CAL_IQ
        if(menu == CALIB){
          calibrate_iq(); menu = 0;
        }
#endif
      }
#ifdef DEBUG
      if(menu == SR){          // measure sample-rate
        numSamples = 0;
        delay(500 * 1.25);     // delay 0.5s (in reality because F_CPU=20M instead of 16M, delay() is running 1.25x faster therefore we need to multiply wqith 1.25)
        sr = numSamples * 2;   // samples per second */
      }
      if(menu == CPULOAD){     // measure CPU-load
        uint32_t i = 0;
        uint32_t prev_time = millis();
        for(i = 0; i != 300000; i++) wdt_reset(); // fixed CPU-load 132052*1.25us delay under 0% load condition; is 132052*1.25 * 20M = 3301300 CPU cycles fixed load
        cpu_load = 100 - 132 * 100 / (millis() - prev_time);
      }
      if((menu == PARAM_A) || (menu == PARAM_B) || (menu == PARAM_C)){
        delay(300);
      }
#endif
    }
  }

  if(menumode == 0){
    if(encoder_val){  // process encoder tuning steps
      process_encoder_tuning_step(encoder_val);
      encoder_val = 0;
    }
  }

  if(change){
    change = false;
    if(prev_bandval != bandval){ freq = band[bandval]; prev_bandval = bandval; }
    save_event_time = millis() + 1000;  // schedule time to save freq (no save while tuning, hence no EEPROM wear out)
 
    if(menumode == 0){
      display_vfo(freq);
  
      // The following is a hack for SWR measurement:
      //si5351.alt_clk2(freq + 2400);
      //si5351.SendRegister(SI_CLK_OE, 0b11111000); // CLK2_EN=1, CLK1_EN,CLK0_EN=1
      //digitalWrite(SIG_OUT, HIGH);  // inject CLK2 on antenna input via 120K
    }
    
    noInterrupts();
    if(mode == CW){
      si5351.freq(freq + cw_offset, 90, 0);  // RX in CW-R (=LSB), correct for CW-tone offset
      si5351.freq_calc_fast(-cw_offset); si5351.SendPLLBRegisterBulk(); // TX at freq
    } else
    if(mode == LSB)
      si5351.freq(freq, 90, 0);  // RX in LSB
    else
      si5351.freq(freq, 0, 90);  // RX in USB, ...
    interrupts();
  }
  
  if((save_event_time) && (millis() > save_event_time)){  // save freq when time has reached schedule
    paramAction(SAVE, FREQ);  // save freq changes
    save_event_time = 0;
    //lcd.setCursor(15, 1); lcd.print("S"); delay(100); lcd.setCursor(15, 1); lcd.print("R");
  }
  
  wdt_reset();
}

/* BACKLOG:
code definitions and re-use for comb, integrator, dc decoupling, arctan
in func_ptr for different mode types
refactor main()
agc based on rms256, agc/smeter after filter
noisefree integrator (rx audio out) in lower range
raised cosine tx amp for cw, 4ms tau seems enough: http://fermi.la.asu.edu/w9cf/articles/click/index.html
auto paddle
cw tx message/cw encoder
32 bin fft
dynamic range cw
att extended agc
configurable F_CPU
CW-L mode
VFO-A/B+split+RIT
OLED display support aka http://www.technoblogy.com/list?22ML
VOX integration in main loop
auto-bias for AUDIO1+2 inputs
K2/TS480 CAT control
faster RX-TX switch to support CW
clock
qcx API demo code
scan
unwanted VOX feedback in DSP mode
move last bit of arrays into flash? https://www.microchip.com/webdoc/AVRLibcReferenceManual/FAQ_1faq_rom_array.html
remove floats
u-law in RX path?: http://dystopiancode.blogspot.com/2012/02/pcm-law-and-u-law-companding-algorithms.html
Arduino library?
1. RX bias offset correction by measurement avg, 2. charge decoupling cap. by resetting to 0V and setting 5V for a certain amount of (charge) time
AGC amplitude sense behind filter, controlling gain before filter
add 1K (500R?) to GND at TTL RF output to keep zero-level below BS170 threshold
additional PWM output for potential BOOST conversion
SWR measurement?
CW decoder amp thrshld restriction and noise reduction (use of certain pause amounts)
squelch gating

Analyse assembly:
/home/guido/Downloads/arduino-1.8.10/hardware/tools/avr/bin/avr-g++ -S -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10810 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR -I/home/guido/Downloads/arduino-1.8.10/hardware/arduino/avr/cores/arduino -I/home/guido/Downloads/arduino-1.8.10/hardware/arduino/avr/variants/standard /tmp/arduino_build_483134/sketch/QCX-SSB.ino.cpp -o /tmp/arduino_build_483134/sketch/QCX-SSB.ino.cpp.txt

Rewire/code I/Q clk pins so that a Div/1 and Div/2 scheme is used instead of 0 and 90 degrees phase shift
10,11,13,12   10,11,12,13  (pin)
Q- I+ Q+ I-   Q- I+ Q+ I-
90 deg.shift  div/2@S1(pin2)

50MHz LSB OK, USB NOK
LCD_ LCD timing differences
filter setting per mode?

*/
