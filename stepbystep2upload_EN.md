**Disclaimer: I have no interest in the uSDX/uSDR project or product or
character, and I play uSDX with an open-source spirit and a positive
attitude of dedication.**

**Foreword:**

First of all, I would like to thank Guido pe1nnz@amsat.org and Manuel
DL2MAN for creating and open source uSDX. Thanks to all the code
contributors for their efforts and efforts to make uSDX a simple and
efficient QRP radio. Secondly, please allow me to express my sincere
gratitude to BA7LJL. Without his help, this notebook would not have been
possible, and I would not have been able to experience the joy of
participating in modifying the uSDX firmware so quickly. Thanks to the
Chinese enthusiasts for designing a beautiful and sturdy casing and
integrating a huge capacity lithium-ion rechargeable battery, making
uSDX a convenient handheld QRP shortwave radio. I was fortunate to have
had the opportunity to participate in the discussion of the product\'s
form factor, and I was glad to have helped spot a small flaw. I was
fortunate enough to be one of the first users, and testing a new device
for the first time is a very exciting thing, but it also presents some
challenges. Today we will discuss the first challenge that everyone
needs to face, how to flash the firmware.

**Ready to work**

**Hardware required**

1. Of course you need a USB to TTL converter, this converter preferably
   already has a DTR port, if you can\'t find it, you need at least a
   USB-TTL converter with a 5V port.
2. According to your uSDX pin definition and interface method, you will
   need to use the corresponding interface. My uSDX provides a TTL port
   through a 4-ring (3+1) 3.5mm audio jack. Then you will need to use a
   corresponding plug, the solderless plug is a convenient choice.
3. USB-TTL converters usually provide some I/O pins on the board, so
   you need to use some Dupont cables. Needle-to-pin and
   needle-to-needle Dupont threads may be used.
4. If your machine has no bootloader (see below to see if you have won
   the lottery), or the bootloader with the wrong frequency, you will
   need 2 additional pieces of equipment. An ISP interface, I recommend
   everyone to use Arduino Uno or Arduino Nano. Because it can not only
   flash uSDX, but also bring you a lot of other fun. If you really
   don\'t like Arduino Uno/Nano, then adjust a USB-ISP converter you
   like.
5. If you don\'t want to disassemble your uSDX like I did, then you
   need to spend a little money, buy a set of 2.54mm 3x2 (6 pins and 2
   rows in total) programming probe, or like the author. A set of
   probes were also made with two PCB boards, as described later.

**Required Software**

1. Install the Arduino IDE. You should already know that the software
   of uSDX is developed with Arduino. Using Arduino IDE can not only be
   used to upgrade the firmware of uSDX, but also directly modify the
   source code of uSDX to create a personalized uSDX. Download address:
   https://www.arduino.cc/en/software

**See if you win**

Maybe you are lucky enough to win the lottery, let\'s see if you win.
First make sure that your uSDX built-in battery has enough power or has
a suitable external power supply, first turn the power switch to the
mid-range to turn off the uSDX, and then stare at the screen to power
on. If the first line of the LCD screen is black at the moment of
power-on, as shown in the figure below, congratulations! Your uSDX is
already bootstrapped, and you can upgrade the firmware directly via TTL.
If uSDX displays the version number and enters the main interface
without hesitation when booting up, congratulations too! Have fun
flashing the Arduino bootloader.

![https://github.com/justsoft/QCX-SSB/raw/feature-rx-improved/bootloader.jpg](https://github.com/justsoft/QCX-SSB/raw/feature-rx-improved/bootloader.jpg)

**Burn in Boot for uSDX**

**Add 20MHz ATMega328P board**

In order to improve the processing power of ATMega328P, uSDX uses a
20MHz main frequency clock, but the default Arduino Nano or Uno uses a
16MHz clock frequency. In order to make uSDX work normally, the board
information of 20MHz frequency is required. There are 3 ways:

1. Find the AVR board file that comes with the Arduino IDE, back it up
   first and then use the downloaded boards-old-bootloader-20MHz.txt
   file to overwrite the existing boards.txt, restart the Arduino IDE
   and you will find a 20MHz Nano board span
2. Modify a 20MHz board information by yourself:

   1. Change the current directory to：c:\\Program Files(x86)\\Arduino\\hardware\\arduino\\avr
   2. Backup the current board information：copy boards.txt
      boards-orig.txt
   3. Open the board file with an editor that supports Unix mode text
      format：boards.txt
   4. Add the following information to: Arduino Nano w/ ATmega328P
      (old bootloader) quick below：
3. \## Arduino Nano w/ ATmega328P (old bootloader)
4. \## \-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\--
5. nano.menu.cpu.atmega328old20Mhz=ATmega328P (Old Bootloader-20Mhz)
6. 
7. nano.menu.cpu.atmega328old20Mhz.upload.maximum_size=30720
8. nano.menu.cpu.atmega328old20Mhz.upload.maximum_data_size=2048
9. nano.menu.cpu.atmega328old20Mhz.upload.speed=72000
10. 
11. nano.menu.cpu.atmega328old20Mhz.bootloader.low_fuses=0xFF
12. nano.menu.cpu.atmega328old20Mhz.bootloader.high_fuses=0xDA
13. nano.menu.cpu.atmega328old20Mhz.bootloader.extended_fuses=0xFD
14. nano.menu.cpu.atmega328old20Mhz.bootloader.file=atmega/ATmegaBOOT_168_atmega328.hex
15. 
16. nano.menu.cpu.atmega328old20Mhz.build.mcu=atmega328p

    5. After saving boards.txt and reopening the Arduino IDE, you can
       see the Old Bootloader-20MHz

1. Recompile to generate 20MHz board information and bootloader, no
   time? No interest, no problem, others have already done it for us,
   download it here, also replace the corresponding file with the
   Arduino IDE with the boards.txt file, and put the .HEX boot file in
   the corresponding bootloader directory.
2. Challenge yourself? Burn a bootloader with only 0.5KB for uSDX, save
   1.5KB for the main program, and make USB-TTL no longer use the
   non-standard 72000 baud rate? So let\'s get started (see also:)

   ![https://tttapa.github.io/Pages/Arduino/Bootloaders/ATmega328P-custom-frequency.html](https://tttapa.github.io/Pages/Arduino/Bootloaders/ATmega328P-custom-frequency.html)
   If you don\'t have time to compile and want to use it directly, then
   happily skip the following 12 sub-steps and download the \[board
   file and boot program\] (# attachments and references)

   6. [[Download and install MinGW]](https://osdn.net/projects/mingw/downloads/68260/mingw-get-setup.exe)
      
   7. Install the following packages:

      - mingw-developer-toolkit
      - mingw32-base
      - mingw32-gcc-g++
      - msys-base
      
   8. Execute the following command to add to the system PATH environment variable
      >
   9. set AVR_DIR=C:\\Program Files (x86)\\Arduino\\hardware\\tools\\avr
      >
   10. set PATH=%AVR_DIR%\\bin;%AVR_DIR%\\etc;C:\\MinGW\\bin;%PATH%
   11. Open a command line prompt as Administrator and execute the following command to make a directory backup of the current optiboot:
      >
   12. cd \"c:\\Program Files (x86)\\Arduino\\hardware\\arduino\\avr\\bootloaders\"
      >
   13. move optiboot optiboot-orig
   14. Download the recent [Optiboot]{.underline} source tarball
   15. After decompression, copy the optiboot directory to c:\\Program Files (x86)\\Arduino\\hardware\\arduino\\avr\\bootloaders\\
       >
   16. Run c:\\MinGW\\msys\\1.0\\msys.bat as an administrator to enter

       > MinGW\'s Bash Shell
       >
   17. Change the current directory to the optiboot source code directory: 

       > cd C:/Program Files (x86)/Arduino/hardware/Arduino/avr/bootloaders/optiboot
       >
   18. Open the Makefile with your familiar editing software and add the following to atmega328_pro8_isp: After isp:
       >
   19. \# atmega328_pro_20MHz
   20. atmega328_pro20: TARGET = atmega328_pro_20MHz
   21. atmega328_pro20: CHIP = atmega328
   22. atmega328_pro20:
   23. \"\$(MAKE)\" \$(CHIP) AVR_FREQ=20000000L LED_START_FLASHES=3
   24. mv \$(PROGRAM)\_\$(CHIP).hex \$(PROGRAM)\_\$(TARGET).hex
   25. ifndef PRODUCTION
   26. mv \$(PROGRAM)\_\$(CHIP).lst \$(PROGRAM)\_\$(TARGET).lst
   27. endif
   28. 
   29. atmega328_pro20_isp: atmega328_pro20
   30. atmega328_pro20_isp: TARGET = atmega328_pro_20MHz
   31. atmega328_pro20_isp: MCU_TARGET = atmega328p
   32. \# 512 byte boot, SPIEN
   33. atmega328_pro20_isp: HFUSE ?= DE
   34. \# Low power xtal (20MHz) 16KCK/14CK+65ms
   35. atmega328_pro20_isp: LFUSE ?= FF
   36. \# 2.7V brownout
   36. atmega328_pro20_isp: EFUSE ?= FD
   38. atmega328_pro20_isp: isp
   39. \# 20MHz atmega328p
   40. atmega328_pro20: TARGET = atmega328_pro_20MHz
   41. atmega328_pro20: MCU_TARGET = atmega328p
   42. atmega328_pro20: CFLAGS += \'-DLED_START_FLASHES=3\' \'-DBAUD_RATE=115200\'
       >
   43. atmega328_pro20: AVR_FREQ = 20000000L
   44. atmega328_pro20: LDSECTIONS = -Wl,\--section-start=.text=0x7e00 -Wl,\--section-start=.version=0x7ffe
       >
   45. atmega328_pro20: \$(PROGRAM)\_atmega328_pro_20MHz.hex
   46. atmega328_pro20: \$(PROGRAM)\_atmega328_pro_20MHz.lst
   47. 
   48. atmega328_pro20_isp: atmega328_pro20
   49. atmega328_pro20_isp: TARGET = atmega328_pro_20MHz
   50. atmega328_pro20_isp: MCU_TARGET = atmega328p
   51. \# 512 byte boot, SPIEN
   52. atmega328_pro20_isp: HFUSE = DE
   53. \# Low power xtal (20MHz) 16KCK/14CK+65ms
   54. atmega328_pro20_isp: LFUSE = FF
   55. \# 2.7V brownout
   56. atmega328_pro20_isp: EFUSE = 05
   57. atmega328_pro20_isp: isp
   58. Run make atmega328_pro20, which will generate the

       > optiboot_atmega328_pro_20MHz.hex and
       > optiboot_atmega328_pro_20MHz.lst files to
       > C:\\Users\\john\\AppData\\Local\\VirtualStore\\Program Files (x86)\\Arduino\\hardware\\arduino\\avr\\bootloaders\\optiboot\\
       >
   59. Change the current directory to: c:\\Program Files x86)\\Arduino\\hardware\\arduino\\avr
       >
   60. Open the information file with an editor, find

       > uno.build.variant=standard and add the following after it:
       >
3. \##############################################################
4. uno20.name=Arduino Uno (Optiboot 20MHz)
5. 
6. uno20.vid.0=0x2341
7. uno20.pid.0=0x0043
8. uno20.vid.1=0x2341
9. uno20.pid.1=0x0001
10. uno20.vid.2=0x2A03
11. uno20.pid.2=0x0043
12. uno20.vid.3=0x2341
13. uno20.pid.3=0x0243
14. 
15. uno20.upload.tool=avrdude
16. uno20.upload.protocol=arduino
17. uno20.upload.maximum_size=32256
18. uno20.upload.maximum_data_size=2048
19. uno20.upload.speed=115200
20. 
21. uno20.bootloader.tool=avrdude
22. uno20.bootloader.low_fuses=0xFF
23. uno20.bootloader.high_fuses=0xDE
24. uno20.bootloader.extended_fuses=0xFD
25. uno20.bootloader.unlock_bits=0x3F
26. uno20.bootloader.lock_bits=0x0F
27. uno20.bootloader.file=optiboot/optiboot_atmega328_pro_20MHz.hex
28. 
29. uno20.build.mcu=atmega328p
30. uno20.build.f_cpu=20000000L
31. uno20.build.board=AVR_UNO
32. uno20.build.core=arduino
33. uno20.build.variant=standard

**Convert Arduino Uno to ISP Programmer**

In order to achieve a more convenient USB-TTL connection to the UART
port of the uSDX for firmware update, it is necessary to first burn the
bootloader for the ATMeage328P in the uSDX. The bootloader needs to use
the ISP interface, I recommend Arduino Uno. The first step is to make
the Arduino Uno an ISP programmer:

1. Connect your Arduino Uno to the computer via USB, wait for Windows
   to discover the Arduino Uno and COM device, and install the
   corresponding drivers
2. Run the Arduino IDE, click the menu Tools \> Board and select
   Arduino Uno

> ![](https://github.com/justsoft/QCX-SSB/blob/feature-rx-improved/uno-1.jpg)

3. Points: File \> Examples \> ArduinoISP \> ArduinoISP

> ![](https://github.com/justsoft/QCX-SSB/blob/feature-rx-improved/uno-2.jpg)

4. Then click Upload. After a while, your Arduino Uno can serve as an
   ISP programmer for uSDX to load the bootloader

**Connect the probe to the Arduino Uno**

Next you need to connect the probes to the Arduino Uno:

1. Remove the 4 screws on each side of the uSDX, and remove the side
   panel and back cover
2. You can see the back of the PCB board below:

> ![https://github.com/justsoft/QCX-SSB/raw/feature-rx-improved/uSDR-pcb.jpg](https://github.com/justsoft/QCX-SSB/raw/feature-rx-improved/uSDR-pcb.jpg)

3. Connect the corresponding probes to the corresponding pins of the
   Arduino Uno according to the table below：

---

  **uSDX/QCX**                      **Arduino Uno**

---

  GND                               GND

  MOSI                              11

  VDD                               5V

  RESET                             10

  SCK                               13

 MISO                               12

**Start burning the boot for uSDX**

When the board information and booting with the main frequency of 20MHz
are ready, you have also successfully loaded the ArduinoISP for the
Arduino Uno, and the probe has been correctly connected to the Arduino
Uno. The next step is to officially burn the bootloader.

1. Open Arduino IDE, select the current Board as: Arduino Uno (Optiboot 20MHz)
2. Select the current Programmer as: \"Arduino as ISP\"

> ![https://github.com/justsoft/QCX-SSB/raw/feature-rx-improved/select-20MHz-board.jpg](https://github.com/justsoft/QCX-SSB/raw/feature-rx-improved/select-20MHz-board.jpg)

3. Power off the uSDX and unplug the power cord and all connections
4. Remove the 4 screws on both sides of the uSDX, and remove the side
   panel and back cover. If there is a built-in battery, you also need
   to disconnect the battery cable
5. Press the probe against the corresponding solder joint on the PCB
   and keep it steady
6. Click Burn Bootloader If everything goes well, the Arduino IDE will
   give a prompt that the burning is successful after a few seconds

**Flash firmware using USB-TTL conversion interface**

Now your uSDX should be able to directly flash the firmware through the
UART socket. The most convenient way is to use a USB-TTL conversion
interface with DTR output. Refer to the uSDX manual to correctly connect
the USB-TTL conversion interface and the UART port plug. . Follow the
steps below to complete the firmware upgrade for your uSDX:

1. Let Arduino IDE control the DTR level from high to low and keep it
   low to trigger the Reset process of uSDX

   1. Open the C:\\Program Files (x86)\\Arduino\\hardware\\tools\\avr\\etc\\avrdude.conf file
      with an editor
   2. Locate the following location:
2. programmer
3. id = \"arduino\";
4. desc = \"Arduino\";
5. type = \"arduino\";
6. connection_type = serial;

   3. Add a line reset = 4; to become:
7. programmer
8. id = \"arduino\";
9. desc = \"Arduino\";
10. type = \"arduino\";
11. connection_type = serial;
12. reset = 4; \# DTR

    4. Save the file


1. Open the Arduino IDE, select the correct Board and Port, as shown in the figure:

> ![https://github.com/justsoft/QCX-SSB/raw/feature-rx-improved/select-20MHz-board-uart.jpg](https://github.com/justsoft/QCX-SSB/raw/feature-rx-improved/select-20MHz-board-uart.jpg)

2. Insert the UART flash plug into the UART port of the uSDX
3. Power on the uSDX
4. Click the Upload button or press the key combination on the
   keyboard: Ctrl+U
5. Wait for the firmware upload to complete
6. Power off the uSDX
7. Press and hold the encoder to power on the uSDX, wait for the screen
   to appear Reset settings.. Release the encoder If everything goes
   well, you have successfully completed the firmware upgrade of the
   uSDX

**Attachments and References**

1. Version file from Arduino IDE v1.8.16 for Arduino Uno that adds
   20MHz and uses 512-byte Optiboot
2. Compiled 115200 baud, 512 byte Optiboot bootloader from
   https://github.com/Optiboot/optiboot (version 28 October 2021)
3. Materials Required for Homemade Probes:

   1. 2 pieces of 2x8cm double-sided or single-sided perforated boards
      with an interval of 2.54mm
   2. 6 probes for PL75-H2 or PL75-Q2
   3. 4 2-2.5mm diameter and 2cm long screws and corresponding nuts
   4. 4 pieces with an inner diameter of about 3mm and a height of
      about 1cm
   5. 6 male and female Dupont cables with a length of about 30cm
   6. The finished look:

> ![https://github.com/justsoft/QCX-SSB/raw/feature-rx-improved/home-made-isp-header.jpg](https://github.com/justsoft/QCX-SSB/raw/feature-rx-improved/home-made-isp-header.jpg)

4. Click to download the modified uSDX source code I am using, the main
   modification content:

   - The direction of double-clicking the encoder to switch the
     channel is determined by the direction of the previous encoder
     changing frequency
   - Added separate volume control for CW sidetone
   - Fixed the issue of sidetone breaking after switching CW sidetone
   - Fix the problem of missing a sample when calculating the average
     value of Mic sampling
   - Hidden backlight switch with no function
