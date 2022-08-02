//Libraries
#include "LSM6DS3.h"
#include "Wire.h"
#include "mic.h"
#include "RunningAverage.h" 


// Settings
#define DEBUG 1
#define SAMPLES 800                   

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

RunningAverage myRA(10); //include these 2 lines of code before void setup()
int samples = 0;

int16_t recording_buf[SAMPLES];
volatile uint8_t recording = 0;
volatile static bool record_ready = false;

void setup() {

  Serial.begin(9600);
  while (!Serial) {delay(10);}

  Mic.set_callback(audio_rec_callback);

  if (!Mic.begin()) {
    Serial.println("Mic initialization failed");
    while (1); }
    
 if (myIMU.begin() != 0) {
        Serial.println("Device error"); } 
    
 else { Serial.println("Device OK!");
        Serial.println("Mic initialization done.");
        Serial.println("Microphone,Accelerometer,Temperature"); }

  Serial.begin(9600); // Original = 115200
  Serial.println("Demo RunningAverage lib");
  Serial.print("Version: ");
  Serial.println(RUNNINGAVERAGE_LIB_VERSION);
  myRA.clear(); // explicitly start clean -- ONLY need this line in void setup()
}

void loop() {
 
  if (record_ready)
  {

  for (int i = 0; i < SAMPLES; i++) {

  //int16_t sample = filter.step(recording_buf[i]);
  int16_t sample = recording_buf[i];

  //Microphone (Blue)
  //Serial.print(sample); //Serial.print(20*log10(abs(sample)/32767)); //Second command may be useful for displaying dBFS instead of PCM
  //Serial.print(",");
  
   //Accelerometer (Red)
    //Serial.print(myIMU.readFloatAccelX()*1000, 4); // You can apply mathematical operations to amplify certain values in the serial plotter...
    //Serial.print(",");                             // I am only using x-direction to simplify things, it should be a function of all three directions...

    //Thermometer (Green)
    Serial.println((myIMU.readTempF()*1000), 4); // 82F was roughly the temperature in the lab, so this is meant to represent change in temperature...

   // delay(50);
    int rn = myIMU.readTempF() ; //this line initiates the variable, change "random(0,1000)" to the variable name you want to take the running average of; long is the type of variable rn-- running average-- is 
  myRA.addValue(rn * 1000);
  samples++;
  Serial.print("Running Average: ");
  Serial.println(myRA.getAverage(), 3);

  if (samples == 300) // this line sets the number of samples included in the running average
  {
    samples = 0;
    myRA.clear();
  }
  delay(50); //this line indicates the delay, lower the delay to collect more data points per time
  }

  record_ready = false;
  }
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
