//  This file is part of: https://github.com/threeme3/QCX-SSB
//
//  Copyright 2019, 2020, 2021, 2022, 2023, 2024, 2025   Guido PE1NNZ <threeme3@hotmail.com>
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


#define N_FILT 7
//volatile uint8_t filt = 0;
//uint8_t prev_filt[] = { 0 , 4 }; // default filter for modes resp. CW, SSB

/* basicdsp filter simulation:
  samplerate=7812
  za0=in
  p1=slider1*10
  p2=slider2*10
  p3=slider3*10
  p4=slider4*10
  zb0=(za0+2*za1+za2)/2-(p1*zb1+p2*zb2)/16
  zc0=(zb0+2*zb1+zb2)/4-(p3*zc1+p4*zc2)/16
  zc2=zc1
  zc1=zc0
  zb2=zb1
  zb1=zb0
  za2=za1
  za1=za0
  out=zc0

  samplerate=7812
  za0=in
  p1=slider1*100+100
  p2=slider2*100
  p3=slider3*100+100
  p4=slider4*100
  zb0=(za0+2*za1+za2)-(-p1*zb1+p2*zb2)/64
  zc0=(zb0-2*zb1+zb2)/8-(-p3*zc1+p4*zc2)/64
  zc2=zc1
  zc1=zc0
  zb2=zb1
  zb1=zb0
  za2=za1
  za1=za0
  out=zc0/8
*/
inline int16_t filt_var(int16_t za0)  //filters build with www.micromodeler.com
{ 
  static int16_t za1,za2;
  static int16_t zb0,zb1,zb2;
  static int16_t zc0,zc1,zc2;
  
  if(filt < 4)
  {  // for SSB filters
    // 1st Order (SR=8kHz) IIR in Direct Form I, 8x8:16
    // M0PUB: There was a bug here, since za1 == zz1 at this point in the code, and the old algorithm for the 300Hz high-pass was:
    //    za0=(29*(za0-zz1)+50*za1)/64;
    //    zz2=zz1;
    //    zz1=za0;
    // After correction, this filter still introduced almost 6dB attenuation, so I adjusted the coefficients
    static int16_t zz1,zz2;
    //za0=(29*(za0-zz1)+50*za1)/64;                                //300-Hz
    zz2=zz1;
    zz1=za0;
    //za0=(30*(za0-zz2)+0*zz1)/32;                                 //300-Hz with very steep roll-off down to 0 Hz
    za0=(30*(za0-zz2)+25*zz1)/32;                                  //300-Hz

    // 4th Order (SR=8kHz) IIR in Direct Form I, 8x8:16
    switch(filt){
      case 1: zb0=(za0+2*za1+za2)/2-(13*zb1+11*zb2)/16; break;   // 0-2900Hz filter, first biquad section
      case 2: zb0=(za0+2*za1+za2)/2-(2*zb1+8*zb2)/16; break;     // 0-2400Hz filter, first biquad section
      //case 3: zb0=(za0+2*za1+za2)/2-(4*zb1+2*zb2)/16; break;     // 0-2400Hz filter, first biquad section
      case 3: zb0=(za0+2*za1+za2)/2-(0*zb1+4*zb2)/16; break;     //0-1800Hz  elliptic
      //case 3: zb0=(za0+7*za1+za2)/16-(-24*zb1+9*zb2)/16; break;  //0-1700Hz  elliptic with slope
    }
  
    switch(filt){
      case 1: zc0=(zb0+2*zb1+zb2)/2-(18*zc1+11*zc2)/16; break;     // 0-2900Hz filter, second biquad section
      case 2: zc0=(zb0+2*zb1+zb2)/4-(4*zc1+8*zc2)/16; break;       // 0-2400Hz filter, second biquad section
      //case 3: zc0=(zb0+2*zb1+zb2)/4-(1*zc1+9*zc2)/16; break;       // 0-2400Hz filter, second biquad section
      case 3: zc0=(zb0+2*zb1+zb2)/4-(0*zc1+4*zc2)/16; break;       //0-1800Hz  elliptic
      //case 3: zc0=(zb0+zb1+zb2)/16-(-22*zc1+47*zc2)/64; break;   //0-1700Hz  elliptic with slope
    }
   /*switch(filt){
      case 1: zb0=za0; break; //0-4000Hz (pass-through)
      case 2: zb0=(10*(za0+2*za1+za2)+16*zb1-17*zb2)/32; break;    //0-2500Hz  elliptic -60dB@3kHz
      case 3: zb0=(7*(za0+2*za1+za2)+48*zb1-18*zb2)/32; break;     //0-1700Hz  elliptic
    }
  
    switch(filt){
      case 1: zc0=zb0; break; //0-4000Hz (pass-through)
      case 2: zc0=(8*(zb0+zb2)+13*zb1-43*zc1-52*zc2)/64; break;   //0-2500Hz  elliptic -60dB@3kHz
      case 3: zc0=(4*(zb0+zb1+zb2)+22*zc1-47*zc2)/64; break;   //0-1700Hz  elliptic
    }*/
  
    zc2=zc1;
    zc1=zc0;
  
    zb2=zb1;
    zb1=zb0;
  
    za2=za1;
    za1=za0;
    
    return zc0;
  } else { // for CW filters
    //   (2nd Order (SR=4465Hz) IIR in Direct Form I, 8x8:16), adding 64x front-gain (to deal with later division)
//#define FILTER_700HZ   1
#ifdef FILTER_700HZ
    if(cw_tone == 0){
      switch(filt){
        case 4: zb0=(za0+2*za1+za2)/2+(41L*zb1-23L*zb2)/32; break;   //500-1000Hz
        case 5: zb0=5*(za0-2*za1+za2)+(105L*zb1-58L*zb2)/64; break;   //650-840Hz
        case 6: zb0=3*(za0-2*za1+za2)+(108L*zb1-61L*zb2)/64; break;   //650-750Hz
        case 7: zb0=(2*za0-3*za1+2*za2)+(111L*zb1-62L*zb2)/64; break; //630-680Hz       
        //case 4: zb0=(0*za0+1*za1+0*za2)+(28*zb1-14*zb2)/16; break; //600Hz+-250Hz
        //case 5: zb0=(0*za0+1*za1+0*za2)+(28*zb1-15*zb2)/16; break; //600Hz+-100Hz
        //case 6: zb0=(0*za0+1*za1+0*za2)+(27*zb1-15*zb2)/16; break; //600Hz+-50Hz
        //case 7: zb0=(0*za0+1*za1+0*za2)+(27*zb1-15*zb2)/16; break; //630Hz+-18Hz
      }
    
      switch(filt){
        case 4: zc0=(zb0-2*zb1+zb2)/4+(105L*zc1-52L*zc2)/64; break;      //500-1000Hz
        case 5: zc0=((zb0+2*zb1+zb2)+97L*zc1-57L*zc2)/64; break;      //650-840Hz
        case 6: zc0=((zb0+zb1+zb2)+104L*zc1-60L*zc2)/64; break;       //650-750Hz
        case 7: zc0=((zb1)+109L*zc1-62L*zc2)/64; break;               //630-680Hz
        //case 4: zc0=(zb0-2*zb1+zb2)/1+(24*zc1-13*zc2)/16; break; //600Hz+-250Hz
        //case 5: zc0=(zb0-2*zb1+zb2)/4+(26*zc1-14*zc2)/16; break; //600Hz+-100Hz
        //case 6: zc0=(zb0-2*zb1+zb2)/16+(28*zc1-15*zc2)/16; break; //600Hz+-50Hz
        //case 7: zc0=(zb0-2*zb1+zb2)/32+(27*zc1-15*zc2)/16; break; //630Hz+-18Hz
      }
    }
    if(cw_tone == 1)
#endif
    {
      switch(filt){
        //case 4: zb0=(1*za0+2*za1+1*za2)+(90L*zb1-38L*zb2)/64; break; //600Hz+-250Hz
        //case 5: zb0=(1*za0+2*za1+1*za2)/2+(102L*zb1-52L*zb2)/64; break; //600Hz+-100Hz
        //case 6: zb0=(1*za0+2*za1+1*za2)/2+(107L*zb1-57L*zb2)/64; break; //600Hz+-50Hz
        //case 7: zb0=(0*za0+1*za1+0*za2)+(110L*zb1-61L*zb2)/64; break; //600Hz+-25Hz
        
        case 4: zb0=(0*za0+1*za1+0*za2)+(114L*zb1-57L*zb2)/64; break; //600Hz+-250Hz
        case 5: zb0=(0*za0+1*za1+0*za2)+(113L*zb1-60L*zb2)/64; break; //600Hz+-100Hz
        case 6: zb0=(0*za0+1*za1+0*za2)+(110L*zb1-62L*zb2)/64; break; //600Hz+-50Hz
        case 7: zb0=(0*za0+1*za1+0*za2)+(110L*zb1-61L*zb2)/64; break; //600Hz+-18Hz
        //case 8: zb0=(0*za0+1*za1+0*za2)+(110L*zb1-60L*zb2)/64; break; //591Hz+-12Hz

        /*case 4: zb0=(0*za0+1*za1+0*za2)+2*zb1-zb2+(-14L*zb1+7L*zb2)/64; break; //600Hz+-250Hz
        case 5: zb0=(0*za0+1*za1+0*za2)+2*zb1-zb2+(-15L*zb1+4L*zb2)/64; break; //600Hz+-100Hz
        case 6: zb0=(0*za0+1*za1+0*za2)+2*zb1-zb2+(-14L*zb1+2L*zb2)/64; break; //600Hz+-50Hz
        case 7: zb0=(0*za0+1*za1+0*za2)+2*zb1-zb2+(-14L*zb1+3L*zb2)/64; break; //600Hz+-18Hz*/
      }
    
      switch(filt){
        //case 4: zc0=(zb0-2*zb1+zb2)/4+(95L*zc1-44L*zc2)/64; break; //600Hz+-250Hz
        //case 5: zc0=(zb0-2*zb1+zb2)/8+(104L*zc1-53L*zc2)/64; break; //600Hz+-100Hz
        //case 6: zc0=(zb0-2*zb1+zb2)/16+(106L*zc1-56L*zc2)/64; break; //600Hz+-50Hz
        //case 7: zc0=(zb0-2*zb1+zb2)/32+(112L*zc1-62L*zc2)/64; break; //600Hz+-25Hz
        
        case 4: zc0=(zb0-2*zb1+zb2)/1+(95L*zc1-52L*zc2)/64; break; //600Hz+-250Hz
        case 5: zc0=(zb0-2*zb1+zb2)/4+(106L*zc1-59L*zc2)/64; break; //600Hz+-100Hz
        case 6: zc0=(zb0-2*zb1+zb2)/16+(113L*zc1-62L*zc2)/64; break; //600Hz+-50Hz
        case 7: zc0=(zb0-2*zb1+zb2)/32+(112L*zc1-62L*zc2)/64; break; //600Hz+-18Hz
        //case 8: zc0=(zb0-2*zb1+zb2)/64+(113L*zc1-63L*zc2)/64; break; //591Hz+-12Hz
        
        /*case 4: zc0=(zb0-2*zb1+zb2)/1+zc1-zc2+(31L*zc1+12L*zc2)/64; break; //600Hz+-250Hz
        case 5: zc0=(zb0-2*zb1+zb2)/4+2*zc1-zc2+(-22L*zc1+5L*zc2)/64; break; //600Hz+-100Hz
        case 6: zc0=(zb0-2*zb1+zb2)/16+2*zc1-zc2+(-15L*zc1+2L*zc2)/64; break; //600Hz+-50Hz
        case 7: zc0=(zb0-2*zb1+zb2)/16+2*zc1-zc2+(-16L*zc1+2L*zc2)/64; break; //600Hz+-18Hz*/
      } 
    }
    zc2=zc1;
    zc1=zc0;
  
    zb2=zb1;
    zb1=zb0;
  
    za2=za1;
    za1=za0;
    
    //return zc0 / 64; // compensate the 64x front-end gain
    return zc0 / 8; // compensate the front-end gain
  }
}
