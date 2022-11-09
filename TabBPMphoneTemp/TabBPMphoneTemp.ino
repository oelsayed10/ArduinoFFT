// Libraries
#include "PDM.h"
#include "arduinoFFT.h"
#include "RunningAverage.h"
#include "ArduinoBLE.h"
#include "LSM6DS3.h"
#include "Wire.h"

// Definitions
#define DEBUG 1
#define SAMPLES 256             //Must be a power of 2
#define SAMPLING_FREQUENCY 16000 //Hz, must be less than 10000 due to ADC
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
#define UpperThreshold 1000 //2500
#define LowerThreshold 850 //1700
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
String UpperLimitInput;
int LowerLimit;
String LowerLimitW;
String LowerLimitInput;
int counter;
int caution;
int escape;
int gate;
int door;
int uppertunnel;
int lowertunnel;

// Bluetooth
BLEService echoService("00000000-0000-1000-8000-00805f9b34fb");
BLEStringCharacteristic charac ("741c12b9-e13c-4992-8a5e-fce46dec0bff", BLERead | BLEWrite | BLENotify, 40);
BLEDescriptor Descriptor("beca6057-955c-4f8a-e1e3-56a1633f04b1","Descriptor");
int numberbpm;
String wordbpm;
int numbertemp;
String wordtemp;
String phonedisplay;
int numberaverage;
String wordaverage;

// Temperature
LSM6DS3 myIMU(I2C_MODE, 0x6A);

void setup() {

delay(3000);
Serial.println("Initializing TrachAlert...");
delay(3000);
  
  gate = 3; // 3 = open, 1 = closed
  door = 3; // 3 = open, 1 = closed
  uppertunnel = 3;
  lowertunnel = 3;
  pinMode(LED_BUILTIN, OUTPUT); //setup the LED light
  Serial.begin(115200);
  //while (!Serial) {
  //  ; // wait for serial port to connect. Needed for native USB port only
  //}

  if(!BLE.begin()){
    Serial.println("BLE failed.");
    while(1);
  }

  if (myIMU.begin() !=0){
    Serial.println("Device error.");
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
  while (central.connected() && uppertunnel > 2) {
    if(charac.written()){
      UpperLimitInput = charac.value();
      UpperLimit = UpperLimitInput.toInt();
      uppertunnel = 1;
    }
  }
  Serial.print("Upper respiratory rate limit set to: ");
  String firstresponse = "Upper Limit set to: ";
  charac.writeValue(firstresponse);
  Serial.print(UpperLimit);
  UpperLimitW = String(UpperLimit);
  charac.writeValue(UpperLimitW);
  Serial.println(" BPM");
  Serial.println("And what should the lower limit for the respiratory rate be?");
  String secondquestion = "Input Lower Limit: ";
  charac.writeValue(secondquestion);
  while (central.connected() && lowertunnel > 2) {
    if(charac.written()){
      LowerLimitInput = charac.value();
      LowerLimit = LowerLimitInput.toInt();
      lowertunnel = 1;
    }
  }
  Serial.print("Lower respiratory rate limit set to: ");
  String secondresponse = "Lower Limit set to: ";
  charac.writeValue(secondresponse);
  Serial.print(LowerLimit);
  LowerLimitW = String(LowerLimit);
  charac.writeValue(LowerLimitW);
  Serial.println(" BPM");
  Serial.println("Thank you for choosing TrachAlert. Respiratory rate measurement will start in 5 seconds.");
  String thanks = "Thank you.";
  charac.writeValue(thanks);
  counter = 1000; // Initial counter value
  caution = 300; //Initial caution value
  escape = 0; //Initial escape value;
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
  int numberaverage = myRA.getAverage();
  String wordaverage = String(numberaverage);
  charac.writeValue(wordaverage);

  if (samples == 2000) // this line sets the number of samples included in the running average
  {
    samples = 0;
    myRA.clear();
  } 
    samplesRead = 0;
    //delay(80); 
    
int value = myRA.getAverage();
if (caution > 0) {
  if (value > UpperThreshold) {
    if (BeatComplete) {
      BPM = millis() - LastTime;
      BPM = int(60 / (float(BPM) / 1000));
      BPMTiming = false;
      BeatComplete = false;
      caution = 300;
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
  Serial.print(" BPM");
  Serial.print("\t");
  Serial.print(myIMU.readTempF(),2);
  Serial.println(" F");
  numberbpm = BPM;
  wordbpm = String(numberbpm);
  numbertemp = myIMU.readTempF();
  wordtemp = String(numbertemp);
  String phonedisplay = String(wordbpm + " BPM    " + wordtemp + " F");
  charac.writeValue(phonedisplay);
  caution = caution - 1;
}

if (caution <= 0) {
 if (value < LowerThreshold) {
  Serial.println("TrachAlert dislodged!");
  String dislodged = "TrachAlert dislodged!";
  charac.writeValue(dislodged);
  caution = caution - 1;
 }

 if (value > UpperThreshold) {
   BeatComplete = true;
  caution = caution + 100;
 }
}
  
  if (BPM > UpperLimit || BPM < LowerLimit){ // If the BPM exceeds the upper limit or drops below the lower limit...
    counter = counter - 1;                   // Subtract 1 from the counter value.
  } else {                                   // Otherwise....
    counter = 300;                           // Reset the counter value to the specified number...
    digitalWrite(LED_BUILTIN, HIGH);         // And turn the LED off.
  }
  if (counter <= 0) {                               // If the counter value reaches 0 or below...
    digitalWrite(LED_BUILTIN, LOW);                 // Turn the LED on...
    Serial.println("Abnormal Breathing Detected!"); // And alert the doctor of the abnormal breathing pattern.
    String ABD = "Abnormal Breathing Detected!";
    charac.writeValue(ABD);
  }

  x++;
  Signal = myRA.getAverage(); // Read the PulseSensor's value.
  // Assign this value to the "Signal" variable.
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
