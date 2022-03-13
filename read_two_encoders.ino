#include <SPI.h>
#include "wiring_private.h" // pinPeripheral() function

#define DEBUG
// -------------------- Communication ----------------------
// Define SPI settings including creating another SPI bus
//              sercom    miso sck  mosi  tx               rx 
SPIClass SPI2 (&sercom3, 11,   13,  12,   SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3); 
SPISettings settingsA(1000000, MSBFIRST, SPI_MODE2);
// -------------------- State Machine ----------------------
// state machine states for the system 
enum cal_enum {
  OPER = 0, // operating
  CALA = 1, // calibrating encoder A
  CALB = 2  // calibrating encoder B
};
volatile cal_enum state = OPER; // current state (volatile as interupts can change it)

// -------------------- Calibration ----------------------
const int cal_N = 100;  // number of reading to average
int cal_n;              // iterator for calibrations
float calValue = 0;     // variable to store calibration result in

int zeros[2] = {0,0};          // encoder counts at zero degrees
const byte calPins[2] = {5,6};  // pin to look for a rising edge to signify calibration
String encChar[2] = {"A","B"}; // character to reference each encoder 
// -------------------- Encoder reading ----------------------
// a structure to store encoder results / number of errors
struct encoder_result {
    int val;
    int err_count;
};

struct encoder_result results[2]; // variables to store encoder values in
byte Data[4]; // variable to store raw data from enocders
 
// -------------------- Debugging ----------------------
#ifdef DEBUG
  int count; // a counter
  unsigned long lastTime; // variables to calculate current operating frequency
  unsigned long newTime;
  int N = 10000; // how often the recorded values are printed to the serial bus
#endif

// -------------------- Setup ----------------------
void setup() {
  // Start the serial bus + the two SPI buses
  Serial.begin(9600);
  SPI.begin();
  SPI2.begin();
 
  // Assign pins SPI pins to correct SERCOM functionality
  pinPeripheral(11, PIO_SERCOM_ALT);
  pinPeripheral(12, PIO_SERCOM);
  pinPeripheral(13, PIO_SERCOM);

  // setup calibraton buttons and interupts
  pinMode(calPins[0], INPUT_PULLUP);
  pinMode(calPins[1], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(calPins[0]), ISR_calA, RISING);
  attachInterrupt(digitalPinToInterrupt(calPins[1]), ISR_calB, RISING);
  
  lastTime = micros(); // populate with an initial time 
}

// Interupt for encoder A 
void ISR_calA(){
  if(state==OPER){
    Serial.println("Calibrating A!!!");
    state = CALA;
    calValue = 0;
    cal_n = 0;
  }
}
// Interupt for encoder B
void ISR_calB(){
  if(state==OPER){
    Serial.println("Calibrating B!!!");
    state = CALB;
    calValue = 0;
    cal_n = 0;
  }
}

// ---------------------- MAIN LOOP ------------------------
void loop() {
  // get data for first encoder
  SPI.beginTransaction(settingsA);
  SPI.transfer(&Data,4);
  SPI.endTransaction();
  results[0] = extract_encoder_value(Data,results[0].err_count);
//  Serial.println(results[0].err_count);
  // update the analog voltage out
  analogWrite(A0,getVoltageOutput(results[0],zeros[0])); 
//  Serial.println(results[1].err_count);
  // get data for second encoder
  SPI2.beginTransaction(settingsA);
  SPI2.transfer(&Data,4);
  SPI2.endTransaction();
  results[1] = extract_encoder_value(Data,results[1].err_count);
  // update the analog voltage out
  analogWrite(A1,getVoltageOutput(results[1],zeros[1])); 

  // state machine loop if calibrating
  if(state>OPER){
    Serial.print(".");
    // when calibrating add results up in cal value
    calValue += (float)results[state-1].val/cal_N;
    
    // if at max number of calibration steps record the value
    if(++cal_n == cal_N){
      zeros[state-1] = round(calValue);
      Serial.print("\nCalibrated ");        
      Serial.print(encChar[state-1]);
      Serial.print("! Zero value:");
      Serial.println(zeros[state-1]);
      state = OPER;
    }
  }

#ifdef DEBUG
  if (++count>=N & state ==OPER){
    // print encoder values
    printEncoderValue(encChar[0],results[0],zeros[0]);
    printEncoderValue(encChar[1],results[1],zeros[1]);

    // print the current processing frequency
    newTime = micros();
    float delta = (float)((newTime-lastTime)/(float)N)*1e-6;
    Serial.print("Frequency: ");
    Serial.print((1.0/delta)*1e-3);
    Serial.println(" KHz");
    lastTime = micros();
    count = 0;
  }
#endif
}

// print the encoder value to the serial bus
void printEncoderValue(String encoder, struct encoder_result res, int zero){
  char str_buffer[80];
  Serial.print("Encoder ");Serial.print(encoder);
  sprintf(str_buffer,": cnts: %d, Volts: ",res.val);
  Serial.print(str_buffer);
  int tmp_out = getVoltageOutput(res,zero);
  Serial.print((float)tmp_out/0xFFF*3.3);  
  if(res.err_count>=10){
    Serial.print("V Error!!!");
  }
  else{
    Serial.print("V No Error");
  }
  Serial.println();
}

// convert encoder cnts into a voltage
int getVoltageOutput(struct encoder_result res,int zero){
  int out = res.val - zero + 0x7FF;
  if(out < 0) out += 0x1000;
  else if(out > 0xFFF) out -= 0x1000;
  out = res.err_count<10 ? out : 0;
  return out;
}

/* Function to read the encoder value from raw bytes from teh data bus
 *  
 * given 4 bytes of data from the SPI bus (32 bits)
 * as per the rls rm-08 data sheet:
 * 
 * bit 0 should always be 1
 * bit 1->12 the encoder value
 * bit 13 is a logical zero
 * bit 14->25 is the same enocder value
 * 
 * to check for no errors:
 * - bit 0 and bit 13 should be 1,0 respectivly (ensures a encoder is connected)
 * - both encoder values should be the same
*/
struct encoder_result extract_encoder_value(byte *data, int err_count){
  bool bit0 = (int)(Data[0] & 0x80);
  
  int value1 = (int)(Data[0] & 0x7F) << 5;
  value1 = value1 | (int)(Data[1] & 0xF8) >> 3;

  bool bit13 = (int)(Data[1] & 0x04);
  int value2 = (int)(Data[1] & 0x03) << 10;
  value2 = value2 | (int)Data[2] << 2;
  value2 = value2 | (int)(Data[3] & 0xC0) >> 6;

  struct encoder_result res;
  res.val = value1;
  res.err_count = err_count;
  if((value1 == value2) & bit0 & !bit13){
    res.err_count = 0;
  }
  else if(err_count<10){
    res.err_count++;
//    Serial.println(res.err_count);
  }
  return res;
}

void printBin16(int *var, uint8_t length) {
  for (int i=0; i<length; i++) { 
    for (unsigned int test = 0x8000; test; test >>= 1) {
      Serial.write(var[i]  & test ? '1' : '0');
    }
  }
}

void printBin8(uint8_t *var, uint8_t length) {
  for (int i=0; i<length; i++) { 
    for (byte test = 0x80; test; test >>= 1) {
      Serial.write(var[i]  & test ? '1' : '0');
    } 
  }
}

void printHex8(uint8_t *data, uint8_t length) // prints 8-bit data in hex with leading zeroes
{
  char tmp[16];
    for (int i=0; i<length; i++) { 
      sprintf(tmp, "%.2X",data[i]); 
      Serial.print(tmp);
    }
}

void printHex16(int *data, uint8_t length) // prints 16-bit data in hex with leading zeroes
{
    char tmp[16];
    for (int i=0; i<length; i++)
    { 
      sprintf(tmp, "%.4X",data[i]); 
      Serial.print(tmp);
    }
}
