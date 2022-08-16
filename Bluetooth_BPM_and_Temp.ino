// Libraries
#include "PDM.h"
#include "arduinoFFT.h"
#include "RunningAverage.h"
#include "ArduinoBLE.h"
#include "LSM6DS3.h"
#include "Wire.h"
#include "mic.h"

// Definitions
#define SAMPLES 256             //Must be a power of 2
#define SAMPLING_FREQUENCY 16000 //Hz, must be less than 10000 due to ADC
#define samp_siz 4 // Experiment
#define rise_threshold 4 // Experiment 

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);

//Microphone Configuration
mic_config_t mic_config{
  .channel_cnt = 1,
  .sampling_rate = 16000,
  .buf_size = 1600,
  //.debug_pin = LED_BUILTIN
};

//Create an instance of class NRF52840
NRF52840_ADC_Class Mic(&mic_config);

short sampleBuffer[SAMPLES];
volatile int samplesRead;

unsigned long microseconds;

// Mic, Temp, and Accelerometer Definitions
int16_t recording_buf[SAMPLES];
volatile uint8_t recording = 0;
volatile static bool record_ready = false;


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

// Variables for Limits
int UpperLimit;
String UpperLimitW;
int LowerLimit;
String LowerLimitW;
int counter;
int gate;
int door;
int window; 
int window2;

// Bluetooth
BLEService echoService("00000000-0000-1000-8000-00805f9b34fb");
BLEStringCharacteristic charac ("741c12b9-e13c-4992-8a5e-fce46dec0bff", BLERead | BLEWrite | BLENotify, 40);
BLEDescriptor Descriptor("beca6057-955c-4f8a-e1e3-56a1633f04b1","Descriptor");
int numberbpm;
String wordbpm;
String UpperLimitInput;
String LowerLimitInput;

void setup() {

delay(3000);
Serial.println("Initializing TrachAlert...");
delay(3000);
  
  gate = 3; // 3 = open, 1 = closed
  door = 3; // 3 = open, 1 = closed
  window = 3; // 3 = open, 1 = closed 
  window2 = 3;
  pinMode(LED_BUILTIN, OUTPUT); //setup the LED light
  Serial.begin(115200);
  // Mic, Temp, and Accelerometer
    Mic.set_callback(audio_rec_callback);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  if (!Mic.begin()) {
    Serial.println("Mic initialization failed");
    while (1); }
    
 if (myIMU.begin() != 0) {
        Serial.println("Device error"); } 
    
 else { Serial.println("Device OK!");
        Serial.println("Mic initialization done.");
        Serial.println("Microphone,Accelerometer,Temperature"); }
        
  if(!BLE.begin()){
    Serial.println("BLE failed.");
    while(1);
  }

  BLE.setLocalName("TrachAlert");
  BLE.setAdvertisedService(echoService);
  charac.addDescriptor(Descriptor);
  echoService.addCharacteristic(charac);
  BLE.addService(echoService);
  BLE.advertise();
 
  PDM.onReceive(onPDMdata);
  PDM.setBufferSize(SAMPLES);
  //PDM.setGain(0);
  // setup the I2S audio input for the sample rate with 32-bits per sample
  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to start PDM!");
    while (1);
  }
  Serial.begin(115200); // Original = 115200
  myRA.clear(); // explicitly start clean -- ONLY need this line in void setup()

  Serial.println("TrachAlert initialized.");
  delay(2000);
  Serial.println("Attempting to establish a Bluetooth connection...");
}

void loop() {
  
  BLEDevice central = BLE.central();
  
if(central){
  Serial.println("Bluetooth connection established.");
  Serial.print("Connected to central: ");
  Serial.println(central.address());

while(central.connected()){

if(gate > 2){
  Serial.println("////////// TrachAlert System //////////");
  String title = "TrachAlert System";
  charac.writeValue(title);
  Serial.println("What should the upper limit for the respiratory rate be?");
  String firstquestion = "Input Upper Limit: ";
  charac.writeValue(firstquestion);
 while (central.connected() && window > 2) { 
  if(charac.written()){
      UpperLimitInput = charac.value();
      charac.writeValue(UpperLimitInput);
      UpperLimit = UpperLimitInput.toInt(); 
        Serial.print("Upper respiratory rate limit set to: ");
  String firstresponse = "Upper Limit set to: ";
  charac.writeValue(firstresponse);
  Serial.print(String(UpperLimitInput));
  Serial.println(" BPM");
  window = 1;
  }
 }
 
  //delay (100);
   //Read the data the user has input
  Serial.println("And what should the lower limit for the respiratory rate be?");
  String secondquestion = "Input Lower Limit: ";
  charac.writeValue(secondquestion);
 while(central.connected() && window2 > 2) {
  // Wait for User to Input Data
  if(charac.written()){
     LowerLimitInput = charac.value();
     charac.writeValue(LowerLimitInput);
     LowerLimit = LowerLimitInput.toInt();
     Serial.print("Lower respiratory rate limit set to: ");
  String secondresponse = "Lower Limit set to: ";
  charac.writeValue(secondresponse);
  Serial.print(String(LowerLimitInput));
  Serial.println(" BPM");
  Serial.println("Thank you for choosing TrachAlert. Respiratory rate measurement will start in 5 seconds.");
  String thanks = "Thank you.";
  charac.writeValue(thanks);
 // Initial counter value
 window2 = 1;
  }
  }
counter = 500; // counter = 1000 with Breathing simulator 
  delay(5000); // Seconds before respiratory rate measurement begins
  gate = 1;
}
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
  numberbpm = BPM;
  wordbpm = String(numberbpm);
  charac.writeValue(wordbpm);
  

  if (BPM > UpperLimit || BPM < LowerLimit){ // If the BPM exceeds the upper limit or drops below the lower limit...
    counter = counter - 1;                   // Subtract 1 from the counter value.
  } else {                                   // Otherwise....
    counter = 300;                           // Reset the counter value to the specified number...
    digitalWrite(LED_BUILTIN, HIGH);         // And turn the LED off.
  }
  if (counter <= 0) {                               // If the counter value reaches 0 or below...
    digitalWrite(LED_BUILTIN, LOW);                 // Turn the LED on...
    Serial.println("Abnormal Breathing Detected!"); 
    String ABD = "Abnormal Breathing Detected!";
    charac.writeValue(ABD);
    // And alert the doctor of the abnormal breathing pattern.
  }

  x++;
  Signal = myRA.getAverage(); // Read the PulseSensor's value.
  // Assign this value to the "Signal" variable.
  }

  // Mic, Temp, and Accelerometer 
  if (record_ready)
  {

  for (int i = 0; i < SAMPLES; i++) {

  //int16_t sample = filter.step(recording_buf[i]);
  int16_t sample = recording_buf[i];

  //Microphone (Blue)
  Serial.print(sample); //Serial.print(20*log10(abs(sample)/32767)); //Second command may be useful for displaying dBFS instead of PCM
  Serial.print(",");
  
   //Accelerometer (Red)
    Serial.print(myIMU.readFloatAccelX()*1000, 4); // You can apply mathematical operations to amplify certain values in the serial plotter...
    Serial.print(",");                             // I am only using x-direction to simplify things, it should be a function of all three directions...

    //Thermometer (Green)
    Serial.println((myIMU.readTempF()-82)*1000, 4); // 82F was roughly the temperature in the lab, so this is meant to represent change in temperature...

    delay(250);
  }

  record_ready = false;
  }

}
Serial.println("Bluetooth disconnected. Please reconnect device to access TrachAlert readings.");
}
}
void onPDMdata()
{
  int bytesAvailable = PDM.available();
  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / 2;
}

static void audio_rec_callback(uint16_t *buf, uint32_t buf_len) {

  static uint32_t idx = 0;
  // Copy samples from DMA buffer to inference buffer
  {
    for (uint32_t i = 0; i < buf_len; i++) {

      // Convert 12-bit unsigned ADC value to 16-bit PCM (signed) audio value
      recording_buf[idx++] = buf[i];
      if (idx >= SAMPLES){
      idx = 0;
      recording = 0;
      record_ready = true;
      break;
     }
    }
  }
}
