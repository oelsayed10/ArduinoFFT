#include <PDM.h>
#include <arduinoFFT.h>
#include "RunningAverage.h"

#define SAMPLES 256             //Must be a power of 2
#define SAMPLING_FREQUENCY 12000 //Hz, must be less than 10000 due to ADC
#define samp_siz 4 // Experiment
#define rise_threshold 4 // Experiment 

short sampleBuffer[SAMPLES];
volatile int samplesRead;

unsigned long microseconds;


RunningAverage myRA(10); //include these 2 lines of code before void setup()
int samples = 0;
int count = 0; 
int data; 
bool inPeak = false;

int x = 0;
int LastTime = 0;
bool BPMTiming = false;
bool BeatComplete = false;
int BPM = 0;    
#define UpperThreshold 2500
#define LowerThreshold 1700 
int LED13 = 44; // The on-board Arduino LED
int Signal; // holds the incoming raw data. Signal value can range from 0-1024

double vReal[SAMPLES];
double vImag[SAMPLES];

void onPDMdata(void);

const uint8_t amplitude = 100;

arduinoFFT FFT = arduinoFFT();

void setup() {
  Serial.begin(115200);
   
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
 
  PDM.onReceive(onPDMdata);
  PDM.setBufferSize(SAMPLES);
  //PDM.setGain(0);
  // setup the I2S audio input for the sample rate with 32-bits per sample
  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to start PDM!");
    while (1);
  }
  Serial.begin(115200); // Original = 115200
  Serial.println("Demo RunningAverage lib");
  Serial.print("Version: ");
  Serial.println(RUNNINGAVERAGE_LIB_VERSION);
  myRA.clear(); // explicitly start clean -- ONLY need this line in void setup()
  
  Serial.println("heart rate: "); // Experiment
}

void loop() {
  if (samplesRead) {
    for (int i = 0; i < SAMPLES; i++) {
      vReal[i] = sampleBuffer[i];
      vImag[i] = 0;
    }

    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
 
    double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
    
    //Serial.println(peak);
    double rn = peak ; //this line initiates the variable, change "random(0,1000)" to the variable name you want to take the running average of; long is the type of variable rn-- running average-- is 
  myRA.addValue(rn);
  samples++;
  Serial.print("Running Average: ");
  Serial.println(myRA.getAverage(), 4);

  if (samples == 4000) // this line sets the number of samples included in the running average
  {
    samples = 0;
    myRA.clear();
  } 
    samplesRead = 0;
    //delay(80); 
    
int value = myRA.getAverage();
  if (value > UpperThreshold) {
    if (BeatComplete) {
      BPM = millis() - LastTime;
      BPM = int(60 / (float(BPM) / 1000));
      BPMTiming = false;
      BeatComplete = false;
    }
    if (BPMTiming == false) {
      LastTime = millis();
      BPMTiming = true;
    }
  }
  if ((value < LowerThreshold) & (BPMTiming))
    BeatComplete = true;
    // display bpm
  Serial.print(BPM);
  Serial.println(" BPM");
  x++;
  Signal = myRA.getAverage(); // Read the PulseSensor's value.
  // Assign this value to the "Signal" variable.
  if (Signal > UpperThreshold) {
    // If the signal is above "550", then "turn-on" Arduino's on-Board LED.
    digitalWrite(LED13, HIGH);
  } else {
    digitalWrite(LED13, LOW); //  Else, the signal must be below "550", so "turn-off" this LED.
  }
    
  }
}

void onPDMdata()
{
  int bytesAvailable = PDM.available();
  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / 2;
}
