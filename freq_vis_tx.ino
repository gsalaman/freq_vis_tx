// this is the transmit module for the frequency visualizer.
// We use pin A5 to listen, and transmit an array of 21 output frequency values.
// Transmit the values via xbee.
// s indicates a start

#include "SoftwareSerial.h"
SoftwareSerial XBee(2,3);

// FHT defines.  This library defines an input buffer for us called fht_input of signed integers.  
#define LIN_OUT 1
#define FHT_N   64
#include <FHT.h>

// These are the raw samples from the audio input.
#define SAMPLE_SIZE FHT_N
int sample[SAMPLE_SIZE] = {0};

//  Audio samples from the ADC are "centered" around 2.5v, which maps to 512 on the ADC.
#define SAMPLE_BIAS 512

// With 64 samples, the FHT returns 32 frequency bins.
// I'm only going to send 21
#define FREQ_BINS 21

#define BIT_BANG_ADC

void setupADC( void )
{

   // Prescalar is the last 3 bits of the ADCSRA register.  
   // Here are my measured sample rates (and resultant frequency ranges):
   // Prescalar of 128 gives ~10 KHz sample rate (5 KHz range)    mask  111
   // Prescalar of 64 gives ~20 KHz sample rate (10 KHz range)    mask: 110    
   // Prescalar of 32 gives ~40 KHz sample rate (20 KHz range)    mask: 101
   
    ADCSRA = 0b11100110;      // Upper bits set ADC to free-running mode.

    // A5, internal reference.
    ADMUX =  0b00000101;

    delay(50);  //wait for voltages to stabalize.  

}

// This function fills our buffer with audio samples.
void collect_samples( void )
{
  int i;
  for (i = 0; i < SAMPLE_SIZE; i++)
  {
    #ifdef BIT_BANG_ADC
    while(!(ADCSRA & 0x10));        // wait for ADC to complete current conversion ie ADIF bit set
    ADCSRA = ADCSRA | 0x10;        // clear ADIF bit so that ADC can do next operation (0xf5)
    sample[i] = ADC;
    #else
    sample[i] = analogRead(AUDIO_PIN);
    #endif
  }  
}

int calc_dc_bias( void )
{
  int i;
  unsigned long total=0;
  unsigned long dc_bias;

  for (i = 0; i < SAMPLE_SIZE; i++)
  {
    total = total + sample[i];
  }

  dc_bias = total / SAMPLE_SIZE;

  if (dc_bias > 1023) Serial.println("*****  DC BIAS OVERFLOW!!!!  *****");

  return dc_bias;
}

// This function does the FHT to convert the time-based samples (in the sample[] array)
// to frequency bins.  The FHT library defines an array (fht_input[]) where we put our 
// input values.  After doing it's processing, fht_input will contain raw output values...
// we can use fht_lin_out() to convert those to magnitudes.
void doFHT( void )
{
  int i;
  int temp_sample;
  
  for (i=0; i < SAMPLE_SIZE; i++)
  {
    // Remove DC bias
    temp_sample = sample[i] - SAMPLE_BIAS;

    // Load the sample into the input array
    fht_input[i] = temp_sample;
    
  }
  
  fht_window();
  fht_reorder();
  fht_run();

  // Their lin mag functons corrupt memory!!!  
  //fht_mag_lin();  
}

void glenn_dc_bias_FHT( int dc_bias )
{
  int i;
  int temp_sample;
  
  for (i=0; i < SAMPLE_SIZE; i++)
  {
    // Remove DC bias
    temp_sample = sample[i] - dc_bias;

    // Load the sample into the input array
    fht_input[i] = temp_sample;
    
  }
  
  fht_window();
  fht_reorder();
  fht_run();

  // Their lin mag functons corrupt memory!!!  
  //fht_mag_lin();  
}


// Since it looks like fht_mag_lin is corrupting memory.  Instead of debugging AVR assembly, 
// I'm gonna code my own C version.  
// I'll be using floating point math rather than assembly, so it'll be much slower...
// ...but hopefully still faster than the FFT algos.
int glenn_mag_calc(int bin)
{
  float sum_real_imag=0;
  float diff_real_imag=0;
  float result;
  int   intMag;

  // The FHT algos use the input array as it's output scratchpad.
  // Bins 0 through N/2 are the sums of the real and imaginary parts.
  // Bins N to N/2 are the differences, but note that it's reflected from the beginning.

  sum_real_imag = fht_input[bin];

  if (bin) diff_real_imag = fht_input[FHT_N - bin];

  result = (sum_real_imag * sum_real_imag) + (diff_real_imag * diff_real_imag);

  result = sqrt(result);
  result = result + 0.5;  // rounding

  intMag = result;
  
  return intMag;

}

void setup() 
{

  Serial.begin(9600);

  XBee.begin(115200);

  #ifdef BIT_BANG_ADC
  setupADC();
  #endif
  
}

void send_freq_data( void )
{
  int i;
  int freq_point;

  // Serial.println("Sending freq data:");
  
  XBee.print("s");
  for (i=0; i<FREQ_BINS; i++)
  {
    freq_point = glenn_mag_calc(i);
    freq_point = constrain(freq_point,0,31);

    XBee.print((char) freq_point);
    Serial.println( (int) freq_point);
  }

  // Serial.println("===========");
}

void loop() 
{

  collect_samples();

  // do the FHT to populate our frequency array
  doFHT();
  
  // and send the samples across the xbee.
  send_freq_data();

  
}
