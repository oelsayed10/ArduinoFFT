#include <PDM.h>
#include <arduinoFFT.h>

#define SAMPLES 256             //Must be a power of 2
#define SAMPLING_FREQUENCY 16000 //Hz, must be less than 10000 due to ADC
short sampleBuffer[SAMPLES];
volatile int samplesRead;

unsigned long microseconds;


double vReal[SAMPLES];
double vImag[SAMPLES];

void onPDMdata(void);

const int ledPin = 22;
const int ledPin2 = 23;
const int ledPin3 = 24;
const uint8_t amplitude = 100;

const float alpha = 0.5;
double data_filtered[] = {0, 0};
double data[] = {0, 0};
const int n = 1;
const int analog_pin = 0;

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
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  pinMode(24, OUTPUT);

  digitalWrite(ledPin, HIGH);
  digitalWrite(ledPin2, HIGH);
  digitalWrite(ledPin3, HIGH);
  
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

    Serial.println(peak);

     // Retrieve Data
    double data = analogRead(peak);

    //Low Pass Filter
    data_filtered[n] = alpha * data + (1 - alpha) * data_filtered[n-1];

    //Store the last filtered data in data_filtered[n-1]
    data_filtered[n-1] = data_filtered[n];
    //Print Data
    Serial.println(data_filtered[n]);

    delay(100);
  }
}

void onPDMdata()
{
  int bytesAvailable = PDM.available();
  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / 2;
}
