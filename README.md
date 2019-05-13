# **ⓆⒸⓍ-ⓈⓈⒷ**
 
# QCX-SSB: SSB with your QCX transceiver (modification)
This is a simple and experimental modification that transforms a [QCX] into a (Class-E driven) SSB transceiver. It can be used to make QRP SSB contacts, or (in combination with a PC) used for the digital modes such as FT8. It can be fully-continuous tuned through bands 160m-10m in the LSB/USB-modes with a 2400Hz bandwidth has up to 5W PEP SSB output and features a software-based full Break-In VOX for fast RX/TX switching in voice and digital operations.

The SSB transmit-stage is implemented completely in a digital and software-based manner: at the heart the 
ATMEGA328P is sampling the input-audio and reconstructing a SSB-signal by controlling the SI5351 PLL phase (through tiny frequency changes over 800kbit/s I2C) and controlling the PA Power (through PWM on the key-shaping circuit). In this way a highly power-efficient class-E driven SSB-signal can be realized; a PWM driven class-E design keeps the SSB transceiver simple, tiny, cool, power-efficient and low-cost (ie. no need for power-inefficient and complex linear amplifier with bulky heat-sink as often is seen in SSB transceivers).

An Open Source Arduino sketch is used as the basis for the firmware, the hardware modification bypasses the QCX CW filter and adds a microphone input in-place of the DVM-circuit; the mod is easy to apply and consist of four wire and four component changes and after applying the transceiver remains compatible with the original QCX (CW) firmware.

This experiment is created to try out what can be done with minimal hardware; a simple ATMEGA processor, a QCX and a software-based SSB processing approach. It would be nice to add more features to the sketch, and try out if the QCX design can be further simplified e.g. by implementing parts of the receiver stage in software. Feel free to experiment with this sketch, let me know your thoughts or contribute here: https://github.com/threeme3/QCX-SSB  There is a forum discussion on the topic here: [QRPLabs Forum]

73, Guido
pe1nnz@amsat.org

![](https://4.bp.blogspot.com/-bYQAutLaijA/XExM0D5YnDI/AAAAAAAABmw/vZMP3G9xXBovKVClV2j1KN3fTPP-9VL1ACLcBGAs/s1600/IMG_8077.jpg])

## List of features:
- **[EER]/[Polar-transmitter] Class-E** driven SSB transmit-stage
- Approximately **5W PEP SSB output** (depending on supply voltage, PA voltage regulated through PWM with **48dB dynamic range**)
- Supports **USB and LSB** modes up to **2400 Hz bandwidth** (receiver and transmitter)
- Two-tone third-order intermodulation distortion **(IMD3) of -33dBc** and **carrier/side-band rejection better than -45dBc** (two-tone)
- Receiver unwanted side-band **rejection up to -20dB**
- Continuously tunable through bands **80m-10m** (anything between 20kHz-99MHz is tunable but with degraded or loss in performance)
- **Multiband** support <sup>[3](#note3)</sup>
- Software-based **VOX** that can be used as **fast Full Break-In** (QSK operation) or assist in RX/TX switching for operating digital modes (no CAT or PTT interface required)
- **Mod simple to apply** (4 wires, 4 components changes and a firmware change)
- Mod remains **downwards-compatible** with the original firmware (except for the loss of DVM function)
- Firmware is **Open Source** through an Arduino Sketch, it allows experimentation, new features can be easily added, contributions can be shared via Github repository [QCX-SSB]
- Completely **digital and software-based** SSB transmit-stage (**no additional circuitry needed**, except for the audio-in circuit)
- **ATMEGA328P signal processing:** samples audio-input and reconstruct a SSB-signal by controlling the _phase of the SI5351 PLL_ (through tiny frequency changes over 800kbits/s I2C) and the _amplitude of the PA_ (through PWM of the PA key-shaping circuit).
- **Lean and low-cost SSB transceiver design**: because of the EER/Polar-transmitter class-E stage it is **highly power-efficient** (no bulky heatsinks required), and has a **simple design** (no complex balanced linear power amplifier required)
- An **pre-distorion** algorithm that cancels out the amplitude errors of non-linearities in the PA voltage regulated PWM supply; a lookup table is used that can be calibrated with an internal PA amplitude measurement


## Revision History:
| Rev.  | Date       | Features                                                            |
| ----- | ---------- | ------------------------------------------------------------------- |
| R1.01f | 2019-05-14 | Added I/Q Calibration feature. Added Voltage, I2C and CPU load self-tests on startup. Reduced RFI feedback on mic. Fix for clock and amplitude-phase mis-alignments. Reduced s-meter (LCD) interference on RX. S-meter Readings. Increased TX bandwidth to 2.4 kHz. Cosmetic improvements. |
| R1.01 | 2019-04-09 | Q6 now digitally switched - improving stability and IMD (**C31 must be removed**). Improved signal processing. Experimental amplitude pre-distortion and calibration. |
| R1.00 | 2019-01-29 | Initial release of prototype                                        |


## Schematic:
The following changes below (marked in color) are made in the schematic (click to zoom), (see here the [original schematic]):
![schematic](https://raw.githubusercontent.com/threeme3/QCX-SSB/master/schematic.png)

This results in the following component and wiring changes (click to zoom):
![layout](https://raw.githubusercontent.com/threeme3/QCX-SSB/master/layout.png)

## Installation:
You will need to install 4 wires, change 6 components and upload the firmware of this sketch:

1. Disconnect +/side of C21 from IC9/pin1 and wire +/side of C21 to R27/pin2, you may<sup>[1](#note1)</sup> insert SPDT switch with common to C21 and the throws to IC9/pin1 and R27/pin2 for SSB/CW switching
2. Remove D4, insert resistor 10K in D4
3. Move R58 (10K) to R56, insert capacitor 220nF in R58
4. Move C42 (10nF) to the backside of PCB and place it on the top pads of D4 and C42 that are closest to IC2
5. Wire DVM/pin3-R57 to IC2/pin21 (AREF), wire junction D4-C42-R58 to IC2/pin18 (DAH), wire DVM/pin2 to IC2/pin20 (AVCC)
6. Remove C31; and replace C32 with 10uF
7. Connect an electret microphone (+/-) between Tip (DAH) and Sleeve (GND) of Paddle-jack, PTT-switch between Ring (DIT) and Sleeve (GND) (such as a [X1M-mic]).
8. Upload new firmware via an Arduino UNO board; install and start [Arduino] environment, connect Arduino Uno to your PC, 
upload QCX-SSB sketch and place the ATMEGA328P into the QCX. See also <sup>[5](#note5)</sup> for an alternative upload method.


## Operation:
Currently, the following functions have been assigned to the buttons:

| Button              | Function                                                |
| ------------------- | ------------------------------------------------------- |
| LEFT single-press   | RX I/Q calibration                                      |
| LEFT double-press   | Internal calibration of PA amplitude (experimental!)    |
| LEFT long-press     | Sweep over frequency 0..2550Hz and amplitude 0..100%    |
| CENTER single-press | Select (smaller) frequency step                         |
| CENTER double-press | Select Band                                             |
| CENTER long-press   | Select (larger) frequency step                          |
| CENTER turn         | Tune frequency                                          |
| RIGHT single-press  | LSB/USB-mode                                            | 
| RIGHT double-press  | Set amplitude drive level on (8=constant carrier on TX) |
| RIGHT long-press    | VOX mode (for full-break-in or digital modes)           |
| KEY                 | Transmitter-keyed (PTT)                                 |

Operating Instructions:

Tuning can be done by turning the rotary encoder. Its step size can be decreased or increased by a short or long press. A change of band can be done with a double press. The mode of operation is altered with a short press on the right button; this can be combined with changing S4 to change between wide-band (SSB) and small-band operation operation.

For SSB voice operation, adjust the amplitude drive by double-pressing right button to a level where voice peaks providing maximum power output (not more than that); this provides an acceptable IMD with good intelligability for local and normal distant operations. In cases where your signal is too weak, set the drive level to 8 to increase the average power output by using a constant amplitude-envelope; in some cases this might be just enough to put your signal above the noise-floor and make yourselve heard; note that this operation degrades the IMD considerably, but since this does not impact the intelligability and since these inter-modulation products are anyway below the noise-floor (and BW limited) they are not in the way, ie. they are not observable by the other station. For long duration QSOs on a specific frequency you can stop holding the PTT by enter (or leave) VOX mode with a long press on right button.

For FT8 (and any other digital) operation, select one of the pre-programmed FT8 bands by double press the rotary encoder, connect the headphone jack to sound card microphone jack, sound card speaker jack to microphone jack, and give a long press on left button to enter VOX mode. Adjust the sound card speaker volume to a minimum and start your favorite FT8 application (JTDX for instance).

To experiment with amplitude pre-distortion algorithm, double-press left button to train the PA amplitude characteristic. This sweeps the amplitude from maximum PWM to minimum PWM and measures the PA response through an internal receiver loopback and stores the values into volatile memory. Once trained, set the appropriate amplitude drive level for voice input. Pre-distorted amplitude response can be measured with a storage spectrum-analyser and a long-press of left button; it will sweep the pre-distorted amplitude from 0 to 100% in 255 steps, where each step has a 10Hz frequency offset.

The receiver side-band rejection can be measured and adjusted through a left single press button. To do so, turn down the volume, connect a dummy-load and enable the original CW-filter. After pressing the button, the I-Q balance, Lo Phase and High phase is measured; adjust R27, R24, R17 subsequently to its minimum side-band rejection value in dB.

On startup, the transceiver is performing a self-test. It is checking the voltages, I2C communications and algorithmic performance. In case of deviations, the display will report an error during startup:

| Error            | Description                                             |
| ---------------- | ------------------------------------------------------- |
| E01 CPU overload | The interrupt routine ADC_vect() is taking too long, more than there are CPU resources available; try to reduce I2C_DELAY or disable functionality in this routine |
| E02 +5V not OK   | The supply that is fed to pin 7 of IC2 is not the expected 5V; this might be an indication that there is an issue with L6 |
| E03 +3.3V not OK | The supply that is fed to pin 1 of IC1 is not the expected 3.3V |
| E04 AVCC not OK  | The supply that is fed to pin 20 of IC2 is not the expected 5V; this might be an indication that there is an issue with L5 |
| E05 DVM bias err | The bias that is fed to pin 25 of IC2 is not the expected 2.5V; this might be an indication that there is an issue with R56/R57 |
| E06 I2C tx error | The I2C communications with SI5351 fails; this might be caused by a bus speed that is too fast; try to increase I2C_DELAY to slow down the bus speed |
| E07 I2C timeout  | The I2C communications with SI5351 fails; the SI5351 is holding the SCL too long |


## Technical Description:
For SSB reception the QCX CW filter is too small, therefore the first modification step 1 is to bypass the CW filter, providing a 3dB wideband passthrough of about 2kHz, this has side-effect that we loose 18dB audio-gain of the CW filter. Another way is to modify the CW-filter <sup>[2](#note2)</sup>, but this creates a steep filter-transition band. Insertion of a SPDT switch between the CW filter output, unfiltered output and the audio amplifier input may support CW and SSB mode selection. The phase-network is less efficient for the full SSB bandwidth in attenuating the unwanted side band, but overall a rejection of ~20 dB can still be achieved. LSB/USB mode switching is done by changing the 90 degree phase shift on the CLK1/CLK2 signals of the SI5351 PLL.

For SSB transmission the QCX DVM-circuitry is changed and used as an audio-input circuit (installation steps 2-5). An electret-microphone (with PTT switch) is added to the Paddle jack connecting the DVM-circuitry, whereby the DOT input acts as the PTT and the DASH input acts as the audio-input (installation step 7). The electret microphone is biased with 5V through a 10K resistor. A 10nF blocking capacitor prevents RF leakage into the circuit. The audio is fed into ADC2 input of the ATMEGA328P microprocessor through a 220nF decoupling capacitor. The ADC2 input is biased at 0.55V via a divider network of 10K to a 1.1V analog reference voltage, with 10-bits ADC resolution this means the microphone-input sensitivity is about 1mV (1.1V/1024) which is just sufficient to process unamplified speech.

A new QCX-SSB firmware is uploaded to the ATMEGA328P (installation step 8), and facilitates a [digital SSB generation technique] in a completely software-based manner. A DSP algorithm samples the ADC2 audio-input at a rate of 4800 samples/s, performs a Hilbert transformation and determines the phase and amplitude of the complex-signal; the phase-changes are restricted<sup>[4](#note4)</sup> and transformed into either positive (for USB) or negative (for LSB) phase changes which in turn transformed into temporary frequency changes which are sent 4800 times per second over 800kbit/s I2C towards the SI5351 PLL. This result in phase changes on the SSB carrier signal and delivers a SSB-signal with a bandwidth of 2400 Hz whereby spurious in the opposite side-band components is attenuated. 

The amplitude of the complex-signal controls the supply-voltage of the PA, and thus the envelope of the SSB-signal. The key-shaping circuit is controlled with a 32kHz PWM signal, which can control the PA voltage from 0 to about 12V in 256 steps, providing a dynamic range of (log2(256) * 6 =) 48dB in the SSB signal. C31 is removed (installation step 6) to ensure that Q6 is operating as a digital switch, this improves the efficiency, thermal stability, linearity, dynamic range and response-time. Though the amplitude information is not mandatory to make a SSB signal intelligable, adding amplitude information improves quality. The complex-amplitude is also used in VOX-mode to determine when RX and TX transitions are supposed to be made.

The IMD performance is related dependent on the quality of the system: the linearity (accuracy) of the amplitude and phase response and the precision (dynamic range) of these quantities. Especially the DSP bit-width, the precision used in the DSP algorithms, the PWM and key-shaping circuit that supplies the PA and the PA phase response are critical. Decreasing (or removing) C32 improves the IMD characteristics but at the cost of an increase of PWM products around the carrier. The following has been done to improve the quality (since R1.01): A. use a more accurate I/Q amplitude estimation algorithm; B. pre-distort, cancel out PA induced amplitude, the SSB generation algorithm applies a amplitude correction through a (amplitude) lookup in a predefined table before modulating. The following planned improvements are still to be done: C. to pre-distort, cancel out PA induced (amplitude-dependent) phase-errors (an experimental phase measurement algorithm can be found in here [commit phase-measurement-experiment]).


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
![twotone](https://raw.githubusercontent.com/threeme3/QCX-SSB/master/twotone.png)

Known/resolved issues:

| Rev.  | Issue | Cause | Resolution |
| ----- | ----- | ----- | ---------- |
| R1.00 | in some cases degraded audio quality especially in local QSOs | analog operation of Q6 causes challenges with biasing, dynamic range, linearity and thermal-drift | (FIXED in R1.01) change C31/C32 so that Q6 operates in digital mode and together with the more accurate signal processing of the new firmware, the IMD performance, carrier+side-band rejection and spectral purity has been improved considerably |
| R1.00 | crackling sounds and noise on TX | ATMEGA ADC is sensitive for noise and in some cases RF feedback worsen this | (FIXED in R1.01c) dynamic noise gating algorithm could be an effective way of mitigating the issue, adding additional inductor in series with mic in could help preventig RF feedback, increasing MIC_ATTEN value in code attentuates the audio input can put the noise below a threshold at the cost of audio sensitivity |
| R1.00 | in VOX mode TX constantly on when soundcard is connected to mic input | VOX is too sensitive and hence responds to the noise of the external device | (FIXED in R1.01c) reduce gain on audio input, e.g. by reducing the output level of the external device, adding a resistive divider in the audio line, increase the MIC_ATTEN value in code to attenuate the signal in software or increase VOX_THRESHOLD to make the VOX algorithm less sensitive, dynamic noise gating algorithm could be an effective way of mitigating the issue |
| R1.01 | RFI on the headphones during TX  | audio opamp share the same 12V supply as the PA |  (FIXED in R1.01c) performing ADC conversions before submitting envelope changes alleviate RFI issues; adding 100uF capacitor from emitter of Q6 to GND alleviates the issue, issue does not occur with constant amplitude SSB |
| R1.01 | after pressing PTT or while tuning RX stops working, audio quality on TX also impacted | likely caused by an I2C speed that is too fast for si5351, resulting in I2C transmission errors | (FIXED in R1.01a) I2C bus speed has been changed from 900kb/s to 700kb/s, and extensive voltage, CPU-timing and I2C relibility checks are done at startup |


### Notes:
1. <a name="note1"/>on a new kit components IC8, IC9, R28-R35, C13-C20, C53 may be omitted if the CW filter is bypassed permanently
2. <a name="note2"/>optionally (not recommended) a steep 2 kHz SSB filter with gain can be realized by modification of Sallen-Key CW filter: replace C13, C15, C17 with 1nF capacitor and remove C53
3. <a name="note3"/>to support si5351 multi-band operation, the RX BPF can be omitted (C1, C5, C8, secondary 3 of T1), and a switchable LPF-bank/matching-network may be placed instead of the existing LPF C25-C28, L1-L3 and matching network C29, C30, L4. The Arduino sketch may be extended to make I2C controllable filter-bank. When using external filters the on-board LPF may be bypassed with a wire.
4. <a name="note4"/>The occupied SSB bandwidth can be further reduced by restricting the maximum phase change (set MAX_DP to half a unit-circle _UA/2 (equivalent to 180 degrees)). The sensitivity of the VOX switching can be set with parameter VOX_THRESHOLD. Audio-input can be attenuated by increasing parameter MIC_ATTEN (6dB per step).
5. <a name="note5"/>In case the ATMEGA328P chip cannot be exchanged you may proceed with ISP programming the QCX via Uno and overwrite the existing firmware: to do so, upload the [ArduinoISP] sketch to Uno, connect Arduino Uno to QCX via [ISP jumper], power on QCX, in Arduino select "Tools > Programmer > Arduino as ISP", select "Tools > Board > Arduino/Genuino Uno", and upload [QCX-SSB Sketch] to QCX by selecting "Sketch > Upload Using Programmer". Once upload succeeds the LCD should display "QCX-SSB". Note that you must disconnect the Microphone in order to use the ISCP/ISP upload facility.


### Credits:
[QCX] (QRP Labs CW Xcvr) is a kit designed by _Hans Summers (G0UPL)_, a high performance, image rejecting DC transceiver; basically a simplified implementation of the [NorCal 2030] by _Dan Tayloe (N7VE)_ designed in 2004 combined with a [Hi-Per-Mite] Active Audio CW Filter by _David Cripe (NMØS)_, [Low Pass Filters] from _Ed (W3NQN)_ 1983 Articles, a key-shaping circuit by _Donald Huff (W6JL)_, a BS170 switched [CMOS driven MOSFET PA] stage as used in [ATS] by _Steven Weber (KD1JV)_ since 2003 and inspired by _Frank Cathell (W7YAZ)_ in 1988, and combined with popular components such as a Silicon Labs [SI5351] Clock Generator, Atmel [ATMEGA328P] microprocessor and a Hitachi [HD44780] LCD display. The [QCX-SSB] modification and its Arduino [QCX-SSB Sketch] is designed by _Guido (PE1NNZ)_; the software-based SSB transmit stage is a derivate of earlier experiments with a [digital SSB generation technique] on a Raspberry Pi in 2013 and is basically a kind of [EER] implemented in software.


### Other

- The VERON association interviewed me in the [PI4AA June issue] about this project (in Dutch, starting at timestamp 15:30).

[QCX]: https://qrp-labs.com/qcx.html

[original schematic]: https://qrp-labs.com/images/qcx/HiRes.png

[ArduinoISP]: https://raw.githubusercontent.com/adafruit/ArduinoISP/master/ArduinoISP.ino

[ISP jumper]: https://qrp-labs.com/images/qcx/HowToUpdateTheFirmwareOnTheQCXusingAnArduinoUNOandAVRDUDESS.pdf

[Arduino]: https://www.arduino.cc/en/main/software

[digital SSB generation technique]: http://pe1nnz.nl.eu.org/2013/05/direct-ssb-generation-on-pll.html

[QCX-SSB]: https://github.com/threeme3/QCX-SSB

[QCX-SSB Sketch]: https://raw.githubusercontent.com/threeme3/QCX-SSB/master/QCX-SSB.ino

[X1M-mic]: https://vignette.wikia.nocookie.net/x1m/images/f/f1/X1M_mic_pinout_diagram.jpg/revision/latest?cb=20131028014710

[QRPLabs Forum]: https://groups.io/g/QRPLabs/topic/29572792

[Norcal 2030]: http://www.norcalqrp.org/nc2030.htm

[Hi-Per-Mite]: http://www.4sqrp.com/hipermite.php

[Low Pass Filters]: http://www.gqrp.com/harmonic_filters.pdf

[CMOS driven MOSFET PA]: http://www.maxmcarter.com/Classexmtr/simplebeacon/mpm_class_e.html

[ATS]: https://groups.yahoo.com/neo/groups/AT_Sprint/files/AT%20Sprint%20/

[EER]: https://core.ac.uk/download/pdf/148657773.pdf

[MBF]: https://www.arrl.org/files/file/QEX_Next_Issue/Mar-Apr2017/MBF.pdf

[Polar-transmitter]: https://sigarra.up.pt/fcup/pt/pub_geral.show_file?pi_doc_id=25850

[Intelligibility]: https://g8jnj.webs.com/speechintelligibility.htm

[commit phase-measurement-experiment]: https://github.com/threeme3/QCX-SSB/blob/aaa0df404e1566cf2bb30badd9ea57e7e1ac0e98/QCX-SSB.ino

[SI5351]: https://www.silabs.com/documents/public/application-notes/AN619.pdf

[ATMEGA328P]: http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega48A-PA-88A-PA-168A-PA-328-P-DS-DS40002061A.pdf

[HD44780]: https://www.sparkfun.com/datasheets/LCD/HD44780.pdf

[sample]: https://youtu.be/-QfMQulk0eA

[PI4AA June issue]: https://cdn.veron.nl/pi4aa/2019/PI4AA_Uitzending20190607.mp3


