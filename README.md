# QCX-SDR: SDR and SSB with your QCX transceiver
This is a simple and experimental modification that transforms a [QCX] into a (Class-E driven) SSB transceiver. It can be used to make QRP SSB contacts, or (in combination with a PC) used for the digital modes such as FT8, JS8, FT4. It can be fully-continuous tuned through bands 160m-10m in the LSB/USB-modes with a 2400Hz bandwidth has up to 5W PEP SSB output and features a software-based full Break-In VOX for fast RX/TX switching in voice and digital operations.

The SSB transmit and receiver stages are implemented completely in a digital and software-based manner. At transmit the ATMEGA328P samples the input-audio and reconstructing a SSB-signal by controlling the SI5351 PLL phase (through tiny frequency changes over 800kbit/s I2C) and controlling the PA Power (through PWM on the key-shaping circuit). In this way a highly power-efficient class-E driven SSB-signal can be realized; a PWM driven class-E design keeps the SSB transceiver simple, tiny, cool, power-efficient and low-cost (ie. no need for power-inefficient and complex linear amplifier with bulky heat-sink as often is seen in SSB transceivers). At receive the ATMEGA328P over-samples the I/Q at 62.5kHz, implements a down-sampling phasing receiver in the digital domain via Hilbert transformers and digital filters, and sends the resulting signal via PWM shaping to the headphone/speaker output.

An Open Source Arduino sketch is used as the basis for the firmware, the hardware modification bypasses the QCX CW filter and adds a microphone input in-place of the DVM-circuit; the mod is easy to apply and consist of four wire and four component changes and after applying the transceiver remains compatible with the original QCX (CW) firmware.

This experiment is created to try out what can be done with very minimal and simplified hardware; a simple ATMEGA processor, a QCX and a software-based SSB processing approach. Feel free to experiment with this sketch, let me know your thoughts or contribute here: https://github.com/threeme3/QCX-SSB  There is a forum discussion on the topic here: [QRPLabs Forum]

73, Guido
pe1nnz@amsat.org

![](top.png)

## List of features:
- Modification into a **simple, fun and versatile QRP SSB HF transceiver** with embedded DSP and SDR functions;
- High-performance QSD mixer with steep baseband roll-off followed by a digital signal processing stage provides a good operational performance in terms of sensitivity, selectivity, spurious rejection and overall (blocking) dynamic range. 
- **[EER]/[Polar-transmitter] Class-E** driven SSB transmit-stage
- Approximately **5W PEP SSB output** (depending on supply voltage, amplitude-PA voltage regulated through PWM with 48dB dynamic range)
- Supports **USB and LSB** modes up to **2400 Hz bandwidth** (receiver and transmitter 400..2330Hz)
- Two-tone third-order intermodulation distortion **(IMD3) of -33dBc** (-16dBc for constant-envelope) and **carrier/side-band rejection better than -45dBc** (two-tone)
- Receiver unwanted side-band **rejection up to -20dB**
- Continuously tunable through bands **80m-10m** (anything between 20kHz-99MHz is tunable but with degraded or loss in performance)
- **Multiband** support
- Software-based **VOX** that can be used as **fast Full Break-In** (QSK operation) or assist in RX/TX switching for operating digital modes (no CAT or PTT interface required)
- **Simple to install modification** with **6 component changes and 4 wires** to implement a basic SSB transceiver, **1 component change** to implement DSP feature,  **11 component changes and 6 wires** to implement SDR feature
- Firmware is **open source** through an Arduino Sketch, it allows experimentation, new features can be easily added, contributions can be shared via Github repository QCX-SSB, about 2000 lines of code
- Completely **digital and software-based** SSB transmit-stage (**no additional circuitry needed**, except for the audio-in circuit)
- **ATMEGA328P signal processing:** samples audio-input and reconstruct a SSB-signal by controlling the _phase of the SI5351 PLL_ (through tiny frequency changes over 800kbits/s I2C) and the _amplitude of the PA_ (through PWM of the PA key-shaping circuit).
- **Lean and low-cost SSB transceiver design**: because of the EER/Polar-transmitter class-E stage it is **highly power-efficient** (no bulky heatsinks required), and has a **simple design** (no complex balanced linear power amplifier required)
- An **pre-distorion** algorithm that cancels out the amplitude errors of non-linearities in the PA voltage regulated PWM supply; a lookup table is used that can be calibrated with an internal PA amplitude measurement
- Possibility to extend the QCX analog phasing stage with a **DSP stage**
- Could replace the QCX analog phasing stage completely with a **digital SDR receiver stage**, taking away the need for the manual side-band rejection adjustment procedure and delivering DSP features such as the joy of having a **AGC, adjustable CW/SSB filters**
- Receiver Noise floor **(MDS): –131 dBm** at 28MHz (in 500Hz BW); Blocking dynamic range: 148dB (200kHz offset or greater), 118dB (20kHz offset), 78dB (2kHz offset). Blocking: 18dBm (200kHz offset or greater), -12dBm (20kHz offset), -52dBm (2kHz offset); LO Phase noise: -138dBc/Hz; Front-end selectivity: Octave
- three independent switchable front-end **attenuators (0dB, -13dB, -20dB, -33dB, -53dB, -60dB, -73dB)**
- This firmware is compatible with an unmodified QCX, partial modified QCX (e.g. SSB mod, or DSP mod), or fully modified QCX (SSB + SDR mod, as described below)
- SDR implementation simplifies the receiver heaviliy; **the original QCX PCB is now half empty** by moving the analog processing into the microcontroller and adding new and improving existing features. On a new QCX build: 46 components less to be installed, 8 component design changes, 9 additional wires.
- Probably the most cost effective and easiest to build standalone SDR transceiver that you can find. More versatile and easier to build than the original QCX (less components, no transformer windings, no alignment procedure)


## Revision History:
| Rev.  | Date       | Features                                                            |
| ----- | ---------- | ------------------------------------------------------------------- |
| R1.02 (**current**) | 2019-12-22 | Integrated SDR receiver, CW decoder, DSP filters, AGC, NR, ATT, experimental modes CW, AM, FM, quick menu, persistent settings. |
| R1.01d | 2019-05-05 | Q6 now digitally switched (remove C31) - improving stability and IMD. Improved signal processing, audio quality, increased bandwidth, cosmetic changes and reduced RF feedback, reduced s-meter RFI, S-meter readings, self-test on startup. Receiver I/Q calibration, (experimental) amplitude pre-distortion and calibration. **See here [original QCX-SSB modification] (it is also supported by current firmware)** |
| R1.00 | 2019-01-29 | Initial release of SSB transceiver prototype. |


## Schematic:
Below the schematic after the modification is applied, unused components are left out and changed components are marked in red (click to zoom & download) (link to [original schematic]):
![schematic](schematic.png)


## Installation:
If you just want to try out the firmware, you can upload and use it in an unmodified QCX but it will have the SSB and SDR features disabled. If you like to try out the DSP audio processing feature, you can simply disconnect R59 and hook up a speaker on pin15/U2 (with 10uF in series, similar as shown above).

To make the SDR+SSB modification, you need to remove 9 and change 8 components, install 10 wires, upload firmware and connect a microphone. On a newly to be build QCX, 46 components can be left out.

Please note that if you apply the mod on a QCX Rev5, you have in addition to convert back to the original Rev4 circuit, ie. restore: R49, R50, C39, R53, (C21, C22 excluded) see [Rev5 changes].

Change the following component values (and type of component in some cases), and wire the following component pins on the backside PCB (some pins must be disconnected from the pad):

1. To implement the SDR receiver: R11,12,R14,R15,17,59 (remove); IC6-10,R11-40,R59-60,C9-24,C52-53,D5,Q7 (optionally omit on new builds); change R7,10(82k); C4,C7(1nF); wire IC2(pin15) to IC10(pin1); disconnect R50-(to 5V) pin and R52-5V and both wire to IC2(pin25); disconnect pin C39(to R27) and wire to IC5(pin1); disconnect pin C40(-to R27) and wire to IC5(pin7).
_Rationale: This will feed the amplified I/Q signals to the ADC0, ADC1 input, biased at AREF/2 V, the rest of the receiver will be handled in software and audio output is realised on PB1. For more sensitivity on the higher bands, there is this [2-stage QCX-SDR modification]._
2. To implement the SSB transmitter: change D4,R56 (10k); R58 (.22uF); C32 (10uF); C31 (remove); wire IC2(pin21) to pin R57(to DVM-pin3); wire IC2(pin20) to DVM(pin2); wire IC2(pin18) to junction D4-C42-R58.
_Rationale: This will bias the mic input (at DAH line) with 5V and pass the audio to ADC2, biased at AREF/2 V; the key-shaping circuit is digitally switching the voltage supply to the PA (or alternatively directly controlled via PA bias<sup>[note 3](#note3)</sup>)._
3. To implement multiband support: C1,C5,C8,T1,R64 (remove); at T1 landing pattern (see [QCX Assembly instruction] page 53) install R (1K) over 6-8; R (1K) over 3-4; C (10nF) over 4-8; C30 (30pF); L4 (1uH/16t); replace C25-28,L1-L3 with different LPFs as you wish.
_Rationale: The resonant elements and the transformer are replaced with a pass-through capacitor._
4. Upload the hex firmware-file to original or new ATMEGA328/328P chip (here is [latest released hex file] and click on "Assets" below the description). The [standard QCX firmware upload procedure] can be followed (for details <sup>[note 1](#note1)</sup>). You can safely switch between this/original QCX firmware without any issues. The fuse settings should be E=FD H=D1 L=F7.
5. Connect an electret microphone pins (+) to tip and (-) to sleeve of paddle-jack; PTT-switch pins to ring and sleeve (see [X1M-mic]).

Below the layout with components marked in red that needs to be changed; gray components must be installed and blank components may be omitted and some must be remove (see above):
![layout](layout.png)

Below the wires that needs to be installed on the bottom PCB; a circle indicates that the component pin is disconnected from the PCB pad and directly wired to another pad (the wire may be fed through the PCB hole or wired on the top side):
![pcb](pcb.png)


## Operation:
Currently, the following functions have been assigned to shortcut buttons (L=left, E=encoder, R=right) and menu-items:

| Menu Item           | Function                                     | Shortcut |
| ------------------- | -------------------------------------------- | -------- |
| 1.1 Volume          | Audio level (0..16) & power-off/on | **E +turn** |
| 1.2 Mode            | Modulation (LSB, USB, CW, AM, FM) | **R** |
| 1.3 Filter BW       | Audio passband (Full, 300..4000, 300..2500, 300..1700, 200, 100 Hz) | **R double** |
| 1.4 Band            | Band-switch to pre-defined FT8 freqs (80,60,40,30,20,17,15,12,10,6,4m) | **E double** |
| 1.5 Tuning Rate     | Tuning step size 10M, 1M, 0.5M, 100k, 10k, 1k, 0.5k, 100, 10, 1 | **E or E long** |
| 1.6 AGC             | Automatic Gain Control (ON, OFF) | |
| 1.7 NR              | Noise-reduction level (0-8), load-pass & smooth | |
| 1.8 ATT             | Analog Attenuator (0, -13, -20, -33, -40, -53, -60, -73 dB) | |
| 1.9 ATT2            | Digital Attenuator in CIC-stage (0-16) in steps of 6dB | |
| 1.10 S-meter        | Type of S-Meter (OFF, dBm, S, S-bar) | |
| 2.1 CW Decoder      | Enable/disable CW Decoder (ON, OFF) | |
| 3.1 VOX             | Voice Operated Xmit (ON, OFF) | **R long** | |
| 3.2 VOX Level       | Audio threshold of VOX (0-255) | |
| 3.3 MOX             | Monitor on Xmit (audio unmuted during transmit) | |
| 3.4 TX Drive        | Transmit audio gain (0-8) in steps of 6dB, 8=constant amplitude for SSB | |
| 8.1 Ref freq        | Actual si5351 crystal frequency, used for frequency-calibration | |
| 8.2 PA Bias min     | KEY_OUT PWM level (0-255) for representing   0% RF output | |
| 8.3 PA Bias max     | KEY_OUT PWM level (0-255) for representing 100% RF output | |
| 9.1 Sample rate     | for debugging, testing and experimental purpose | |
| 9.2 CPU load        | for debugging, testing and experimental purpose | |
| 9.3 Param A         | for debugging, testing and experimental purpose | |
| 9.4 Param B         | for debugging, testing and experimental purpose | |
| 9.5 Param C         | for debugging, testing and experimental purpose | |
| main                | Frequency (20kHz..99MHz) | **turn** |
| main                | Quick menu | **L +turn** |
| main                | Menu enter | **L** |
| menu                | Menu back | **R** |


Operating Instructions:

Tuning can be done by turning the rotary encoder. Its step size can be decreased or increased by a short or long press. A change of band can be done with a double press. The mode of operation is altered with a short press on the right button; this can be combined with changing S4 to change between wide-band (SSB) and small-band operation operation.

For SSB voice operation, adjust the amplitude drive by double-pressing right button to a level where voice peaks providing maximum power output (not more than that); this provides an acceptable IMD with good intelligability for local and normal distant operations. In cases where your signal is too weak, set the drive level to 8 to increase the average power output by using a constant amplitude-envelope; in some cases this might be just enough to put your signal above the noise-floor and make yourselve heard; note that this operation degrades the IMD considerably, but since this does not impact the intelligability and since these inter-modulation products are anyway below the noise-floor (and BW limited) they are not in the way, ie. they are not observable by the other station. For long duration QSOs on a specific frequency you can stop holding the PTT by enter (or leave) VOX mode with a long press on right button.

For FT8 (and any other digital) operation, select one of the pre-programmed FT8 bands by double press the rotary encoder, connect the headphone jack to sound card microphone jack, sound card speaker jack to microphone jack, and give a long press on left button to enter VOX mode. Adjust the sound card speaker volume to a minimum and start your favorite FT8 application (JTDX for instance).

To experiment with amplitude pre-distortion algorithm, double-press left button to train the PA amplitude characteristic. This sweeps the amplitude from maximum PWM to minimum PWM and measures the PA response through an internal receiver loopback and stores the values into volatile memory. Once trained, set the appropriate amplitude drive level for voice input. Pre-distorted amplitude response can be measured with a storage spectrum-analyser and a long-press of left button; it will sweep the pre-distorted amplitude from 0 to 100% in 255 steps, where each step has a 10Hz frequency offset.

The receiver side-band rejection can be measured and adjusted through a left single press button. To do so, turn down the volume, connect a dummy-load and enable the original CW-filter. After pressing the button, the I-Q balance, Lo Phase and High phase is measured; adjust R27, R24, R17 subsequently to its minimum side-band rejection value in dB.

On startup, the transceiver is performing a self-test. It is checking the supply and bias voltages, I2C communications and algorithmic performance. In case of deviations, the display will report an error during startup.


## Technical Description:
The principle of operation is depicted in the following video-fragment: [Opzij] (in Dutch; [lyrics])... :-)  jokes aside; below the block diagram of the QCX-SSB and SDR transceiver:
![block diagram](block.png)

For SSB reception, the QCX analog phasing receiver stage is replaced with a digital SDR stage; this means that the phase shifting op-amp IC6 is changed into a regular amplifier and whereby the individual I and Q outputs are directly fed into the ATMEGA328P ADC inputs for signal processing. The ATMEGA328P will over-sample the ADC input at a 32kHz sample-rate and perform a phase-shift by means of a Hilbert-transform and summing the result to obtain side-band rejection; it will also perform CW or SSB filtering and provide an AGC function. Since the phase-shifting network and analog CW filter are no used, about 30% of the components can be left out; by combining the function of IC7B into IC6A another op-amp can be saved. The ADC inputs are low-pass filtered (-40dB/decade roll-off at 1.5kHz cut-off) to prevent aliasing and input are biased with a 1.1V analog reference voltage to obtain additional sensitivity and dynamic range. With the 10-bit ADCs and a 4x over-sampling rate, a theoretical dynamic range of 72dB can be obtained in 2.4kHz SSB bandwidth. LSB/USB mode switching is done by changing the 90 degree phase shift on the CLK1/CLK2 signals of the SI5351 PLL.

For SSB transmission the QCX DVM-circuitry is changed and used as an audio-input circuit. An electret-microphone (with PTT switch) is added to the Paddle jack connecting the DVM-circuitry, whereby the DOT input acts as the PTT and the DASH input acts as the audio-input. The electret microphone is biased with 5V through a 10K resistor. A 10nF blocking capacitor prevents RF leakage into the circuit. The audio is fed into ADC2 input of the ATMEGA328P microprocessor through a 220nF decoupling capacitor. The ADC2 input is biased at 0.55V via a divider network of 10K to a 1.1V analog reference voltage, with 10-bits ADC resolution this means the microphone-input sensitivity is about 1mV (1.1V/1024) which is just sufficient to process unamplified speech.

A new QCX-SSB firmware is uploaded to the ATMEGA328P, and facilitates a [digital SSB generation technique] in a completely software-based manner. A DSP algorithm samples the ADC2 audio-input at a rate of 4800 samples/s, performs a Hilbert transformation and determines the phase and amplitude of the complex-signal; the phase-changes are restricted<sup>[note 2](#note2)</sup> and transformed into either positive (for USB) or negative (for LSB) phase changes which in turn transformed into temporary frequency changes which are sent 4800 times per second over 800kbit/s I2C towards the SI5351 PLL. This result in phase changes on the SSB carrier signal and delivers a SSB-signal with a bandwidth of 2400 Hz whereby spurious in the opposite side-band components is attenuated. 

The amplitude of the complex-signal controls the supply-voltage of the PA, and thus the envelope of the SSB-signal. The key-shaping circuit is controlled with a 32kHz PWM signal, which can control the PA voltage from 0 to about 12V in 256 steps, providing a dynamic range of (log2(256) * 6 =) 48dB in the SSB signal. C31 is removed to ensure that Q6 is operating as a digital switch, this improves the efficiency, thermal stability, linearity, dynamic range and response-time. Though the amplitude information is not mandatory to make a SSB signal intelligable, adding amplitude information improves quality. The complex-amplitude is also used in VOX-mode to determine when RX and TX transitions are supposed to be made.

The IMD performance is related dependent on the quality of the system: the linearity (accuracy) of the amplitude and phase response and the precision (dynamic range) of these quantities. Especially the DSP bit-width, the precision used in the DSP algorithms, the PWM and key-shaping circuit that supplies the PA and the PA phase response are critical. Decreasing (or removing) C32 improves the IMD characteristics but at the cost of an increase of PWM products around the carrier.


## Results
Here is a [sample] me calling CQ on 40m with my QCX-SSB at 5W and received back by the Hack Green websdr about 400km away.

Several OMs reported a successful QCX-SSB modification and were able to make SSB QRP DX contacts over thousands of kilometers on the 20m and 40m bands. During CQ WW contest I was able to make 34 random QSOs on 40m with 5W and an inverted-V over the house in just a few hours with CN3A as my furthest contact, I could observe the benefits of using SSB with constant-envelope in cases where my signal was weak; for FT8 a Raspberry Pi 3B+ with JTDX was used to make FT8 contacts all the way up to NA.

Measurements:
The following performance measurements were made with QCX-SSB R1.01, a modified RTL-SDR, Spektrum-SVmod-v0.19, Sweex 5.0 USB Audio device and Audicity player. It is recognized that this measurement setup has its own limitations, hence the dynamic range of the measurements is somewhat limited by the RTL-SDR as this device goes easily into overload. Measurements were made with the following setttings: USB modulation, no pre-distortion, two-tone input 1000Hz/1200Hz where audio level is set just before the point where compression starts. Results:
- Intermodulation distortion products (two-tone; SSB with varying  envelope) IMD3, IMD5, IMD7: respectively -33dBc; -36dBc; -39dBc
- Intermodulation distortion products (two-tone; SSB with constant envelope) IMD3, IMD5, IMD7: respectively -16dBc; -16dBc; -19dBc
- Opposite side-band rejection (two-tone): better than -45dBc
- Carrier rejection (two-tone): better than -45dBc
- Wide-band spurious (two-tone): better than -45dBc
- 3dB bandwidth (sweep): 400..2330Hz
![twotone](twotone.png)


### Notes:
1. <a name="note1"/>Firmware upload variations:
- [AVRDudess] tool or avrdude CLI (avrdude -c avrisp -b 19200 -P /dev/ttyACM0 (or: /dev/ttyUSB0) -p m328p -e -U efuse:w:0xfd:m -U hfuse:w:0xC1:m -U lfuse:w:0xF7:m -U flash:w:firmware.hex) can be used for uploading the firmware via the ISP connector on the QCX. Follow [Arduino as ISP] instructions if you have a Arduino UNO board available (tip: use female-to-male breadboard cables to connect Arduino to QCX ISP jumper); or [USBasp] instructions if you have a USBasp programmer, alternatively use [USPasp ExtremeBurner]; but many other ISP programmers can be used in similar manner such as [USBtiny] or AVRisp mkII. During ISP, mic should be disconnected, power supply should be connected; in tool do not erase, program EEPROM or set fuse settings (they are by default ok: E=FD H=D1 L=F7).
- Alternatively, in case you have an ATMEGA328P chip with Arduino bootloader, you can place the chip in an Arduino UNO board and upload directly (without the need for a ISP cable and QCX) by specifying 'arduino' programmer and baudrate 115200.
- Alternatively, in case you have an [Arduino 1.8.9] environment installed, you can upload the [QCX-SSB Sketch] directly from the Arduino environment (without using AVRDudess and firmware file); make sure "Tools > Board > Arduino/Genuino Uno",  "Tools > Port > /dev/ttyUSB0 or ttyACM0", and then "Sketch > Upload" is selected, while the ATMEGA328P chip is placed in the Arduino UNO socket. It is also possible to use [Arduino as ISP] method: upload this variation of [ArduinoISP] to the Arduino board and select "Tools > Programmer > Arduino as ISP", and "Sketch > Upload Using Programmer".
2. <a name="note2"/>The occupied SSB bandwidth can be further reduced by restricting the maximum phase change (set MAX_DP to half a unit-circle _UA/2 (equivalent to 180 degrees)). The sensitivity of the VOX switching can be set with parameter VOX_THRESHOLD. Audio-input can be attenuated by increasing parameter MIC_ATTEN (6dB per step).
3. Alternatively, the PA MOSFETs can be directly biased by the PWM envelope signal, basically making the key-shaping circuit redundant. To do so, Q6,Q4,R41,R42,C32,C31 can be removed entirely, whereby C-E pads of Q6 are wired, and where a 100nF capacitor is inserted at IC3A-pin3 and G of Q1-3, and where a 10k resistor is placed at G-D pads of Q4, a 10nF capacitor between S-D pads of Q4, and where a 10k resistor is placed between D of Q4 and G of Q1-3.


### Credits:
[QCX] (QRP Labs CW Xcvr) is a kit designed by _Hans Summers (G0UPL)_, originally built for RSGB's YOTA summer camp 2017, a high performance, image rejecting DC transceiver; basically a simplified implementation of the [NorCal 2030] by _Dan Tayloe (N7VE)_ designed in 2004 combined with a [Hi-Per-Mite] Active Audio CW Filter by _David Cripe (NMØS)_, [Low Pass Filters] from _Ed (W3NQN)_ 1983 Articles, a key-shaping circuit by _Donald Huff (W6JL)_, a BS170 switched [CMOS driven MOSFET PA] architecture as used in the [ATS] designs by _Steven Weber (KD1JV)_ (originating from the [Power MOSFET revolution] in the mid 70s), and combined with popular components such as Atmel [ATMEGA328P] microprocessor, a Hitachi [HD44780] LCD display and a Silicon Labs [SI5351] Clock Generator (and using a [phase shift in the SI5351 clocks]). The [QCX-SSB] transmitter and QCX-SDR receiver stage both running on a ATMEGA328P, including its multiband front-end and direct PA biasing/envelope-generation technique; its concept, circuit, code and modification to run on a QCX are a design by _Guido (PE1NNZ)_; the software-based SSB transmit stage is a derivate of earlier experiments with a [digital SSB generation technique] on a Raspberry Pi.

<!---
### References
- VERON association interviewed me in the [PI4AA June issue] about this project (in Dutch, starting at timestamp 15:50).
- Rüdiger Möller, HPSDR presentation by [DJ1MR], 2018. Transmitter architectures for high efficiency amplification
- [Arduino PWM]
- [Serial interface]
--->

[original schematic]: https://qrp-labs.com/images/qcx/HiRes.png

[Rev5 changes]: https://groups.io/g/QRPLabs/message/42095

[QCX-SSB]: https://github.com/threeme3/QCX-SSB

[QCX-SSB Sketch]: QCX-SSB.ino

[latest released hex file]: https://github.com/threeme3/QCX-SSB/releases

[original QCX-SSB modification]: https://github.com/threeme3/QCX-SSB/tree/26c4e97a034d367e1325c5587a56a7c2a43c69f3

[2-stage QCX-SDR modification]: https://github.com/threeme3/QCX-SSB/tree/653e2d8c387138d30269ffd003065be78cc648ca

[standard QCX firmware upload procedure]: https://www.qrp-labs.com/qcx/qcxfirmware.html

[USBasp]: https://sites.google.com/site/g4zfqradio/qrplabs_program_chip_with_USBasp

[USPasp ExtremeBurner]: https://groups.io/g/QRPLabs/topic/57461404#40024

[USBtiny]: https://groups.io/g/QRPLabs/attachment/40315/0/QCX%20Firmware%20Update%20Instructions.pdf

[Arduino as ISP]: https://qrp-labs.com/images/qcx/HowToUpdateTheFirmwareOnTheQCXusingAnArduinoUNOandAVRDUDESS.pdf

[AVRDudess]: http://zakkemble.net/avrdudess

[Arduino]: https://www.arduino.cc/en/main/software#download

[Arduino 1.8.9]: https://www.arduino.cc/en/Main/OldSoftwareReleases#previous

[ArduinoISP]: https://raw.githubusercontent.com/adafruit/ArduinoISP/master/ArduinoISP.ino

[digital SSB generation technique]: http://pe1nnz.nl.eu.org/2013/05/direct-ssb-generation-on-pll.html

[QCX]: https://qrp-labs.com/qcx.html

[X1M-mic]: https://vignette.wikia.nocookie.net/x1m/images/f/f1/X1M_mic_pinout_diagram.jpg/revision/latest?cb=20131028014710

[QRPLabs Forum]: https://groups.io/g/QRPLabs/topic/29572792

[Norcal 2030]: http://www.norcalqrp.org/nc2030.htm

[Hi-Per-Mite]: http://www.4sqrp.com/hipermite.php

[Low Pass Filters]: http://www.gqrp.com/harmonic_filters.pdf

[CMOS driven MOSFET PA]: http://www.ka7oei.com/mpm_class_e.html

[Power MOSFET revolution]: https://archive.org/details/VMOSSiliconixOCR/page/n17

[ATS]: https://groups.yahoo.com/neo/groups/AT_Sprint/files/AT%20Sprint%20/

[Intelligibility]: https://g8jnj.webs.com/speechintelligibility.htm

[SI5351]: https://www.silabs.com/documents/public/application-notes/AN619.pdf

[ATMEGA328P]: http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega48A-PA-88A-PA-168A-PA-328-P-DS-DS40002061A.pdf

[HD44780]: https://www.sparkfun.com/datasheets/LCD/HD44780.pdf

[sample]: https://youtu.be/-QfMQulk0eA

[PI4AA June issue]: https://cdn.veron.nl/pi4aa/2019/PI4AA_Uitzending20190607.mp3

[EER]: https://core.ac.uk/download/pdf/148657773.pdf

[MBF]: https://www.arrl.org/files/file/QEX_Next_Issue/Mar-Apr2017/MBF.pdf

[Polar-transmitter]: https://sigarra.up.pt/fcup/pt/pub_geral.show_file?pi_doc_id=25850

[DJ1MR]: https://www.youtube.com/watch?v=A6ohr98ikeA

[RX3DPK]: https://www.facebook.com/photo.php?fbid=2382353591830446&set=pcb.2382353628497109&type=3&theater

[QRP-BR]: https://groups.io/g/QRP-BR/topic/gerando_ssb_digitalmente/29628623

[Opzij]: https://youtu.be/uN706PiFLm0

[lyrics]: https://www.google.com/search?q=lyrics+opzij

[phase shift in the SI5351 clocks]: https://www.silabs.com/community/timing/forum.topic.html/difficulty_settingp-LchG

[QCX Assembly instruction]: https://www.qrp-labs.com/images/qcx/assembly_A4_Rev_4b.pdf

[Arduino PWM]: http://interface.khm.de/index.php/lab/interfaces-advanced/arduino-dds-sinewave-generator/

[Serial interface]: https://groups.io/g/QRPLabs/attachment/40706/0/connections.png

