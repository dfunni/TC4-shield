// init_shield
// version 2.1, 17-Sept-2014, JGG

#define OT1 9
#define OT2 10
#define IO3 3
#define IO2 2
#define LED 13

#include <cADC.h>
#include <Wire.h>
#include <thermocouple.h>
#include <mcEEPROM.h>

#define BANNER_K "Shield_Test" // version
//#define TEMP_OFFS -0.0
//#define GAIN_CAL 1.000
#define AMB_FILT 50
#define ADC_FILT 10

//#define REFINED
#define ADC_BITS ADC_BITS_18
#define ADC_GAIN ADC_GAIN_8
#define AMB_BITS AMB_BITS_12
#define CELSIUS

calBlock inforx = {
  "",
  "",
  -99.0,
  -99,
  -99.0,
  -99.0
};

mcEEPROM ep;
cADC adc;
ambSensor amb;
filterRC f[4];
typeK tc;
float ctemp;

long i = 0;
int dly;
uint8_t toggle = 0;

void setup() {
  delay( 1000 );
  Serial.begin(115200);
  Wire.begin();
  
// read contents of EEPROM calibration block
  ep.read( 0, (uint8_t*) &inforx, sizeof( inforx ) );
  Serial.println(inforx.PCB);
  Serial.println(inforx.version);
  Serial.println(inforx.cal_gain, DEC);
  Serial.println(inforx.cal_offset);
  Serial.println(inforx.T_offset);
  Serial.println(inforx.K_offset);
  Serial.println();
  

  delay( 1000 );

  adc.setCal ( inforx.cal_gain, 0 );
  uint8_t j;
  for( j = 0; j < 4; j++ )
    f[j].init( ADC_FILT );

#ifdef REFINED
  amb.init( AMB_FILT, AMB_CONV_1SHOT );
  amb.setCfg( AMB_BITS );
  adc.setCfg( ADC_BITS, inforx.cal_gain );
  int d = amb.getConvTime();
  dly = adc.getConvTime();
  dly = dly > d ? dly : d;
#else
  amb.init( AMB_FILT );
#endif

  amb.setOffset( inforx.K_offset );

  pinMode( OT1, OUTPUT );
  pinMode( OT2, OUTPUT );
  pinMode( IO3, OUTPUT );
  pinMode( IO2, OUTPUT );
  pinMode( LED, OUTPUT );

}

void loop() {
  uint8_t j;
  uint8_t r,c;
  
  digitalWrite( OT1, !toggle );
  digitalWrite( OT2, toggle );
  digitalWrite( LED, !toggle );
  digitalWrite( IO3, !toggle );
  digitalWrite( IO2, toggle );

  Serial.print( i++ );
  
  for( j = 0; j < 4; j++ ) {
    adc.nextConversion( j );
    amb.nextConversion();  
#ifdef REFINED
    delay( dly );
#else
    delay( 300 );
#endif
    int32_t v = adc.readuV();
    int32_t fv = f[j].doFilter( v << 10 );
    fv >>= 10;
    amb.readSensor();
    ctemp = amb.getAmbC();
    if( j == 0 ) {
      Serial.print( "," ); Serial.print( C_TO_F(ctemp), 1 );
    }
    float tempC = tc.Temp_C( 0.001 * fv, ctemp ) ;

#ifdef CELSIUS
    Serial.print( "," ); Serial.print( (tempC) );
#else
    Serial.print( "," ); Serial.print( C_TO_F(tempC) );
#endif
    if( j == 0 ) {
      r = 0; c = 0;
    }
    else if( j == 1 ) {
      r = 0; c = 8;
    }
    else if( j == 2 ) {
      r = 1; c = 0;
    }
    else if( j == 3 ) {
      r = 1; c = 8;
    }

  }
  Serial.println();
  toggle = !toggle;
}
