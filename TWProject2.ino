#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024
#define arrayLength 4


int writeIndex = 0;
int readIndex = 0;
int data[arrayLength]; // 2 sync + 4 data + 1 checksum
int sync = 170;

void startTimer(int frequencyHz);
void setTimerFrequency(int frequencyHz);
void TC3_Handler();
bool newData = false;

void setup() {

  
  Serial.begin(38400);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  startTimer(10);
  
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  
  
}

void loop() {
  data[0] = 170;
  data[1] = 170; 
  data[2] = 4;
  data[7] = getChecksum(data);

  if (newData){
    if (readIndex <= writeIndex){
      while (readIndex <= writeIndex-1){
        Serial.print("Current index: ");   // change to Serial.write()
        Serial.print(readIndex);
        Serial.print(" = ");
        Serial.println(data[readIndex]);
        readIndex++;
    }  
   }
   else if (readIndex >= writeIndex){
    while (readIndex <= arrayLength-1){
      Serial.print("Current index: ");   // change to Serial.write()
     Serial.print(readIndex);
             Serial.print(" = ");

      Serial.println(data[readIndex]);
      readIndex++;
    }  
    readIndex = 0;
    while (readIndex <= writeIndex-1){
      Serial.print("Current index: ");   // change to Serial.write()
     Serial.print(readIndex);   
             Serial.print(" = ");

      Serial.println(data[readIndex]);
      readIndex++;
    }
   }
   newData = false;
  }
  
}

void startTimer(int frequencyHz) {
   
   REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3) ;
   while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

   TcCount16* TC = (TcCount16*) TC3;

   TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
   while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

   // Use the 16-bit timer
   TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
   while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

   // Use match mode so that the timer counter resets when the count matches the compare register
   TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
   while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

   // Set prescaler to 1024
   TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
   while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

   setTimerFrequency(frequencyHz);

   // Enable the compare interrupt
   TC->INTENSET.reg = 0;
   TC->INTENSET.bit.MC0 = 1;

   NVIC_EnableIRQ(TC3_IRQn);

   TC->CTRLA.reg |= TC_CTRLA_ENABLE;
   while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

void setTimerFrequency (int frequencyHz) 
{
  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;  // = 89.____ setting the compare value = should be 2ms?
  TcCount16* TC = (TcCount16*) TC3;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  
  TC->COUNT.reg = map (TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  
  Serial.println(TC->COUNT.reg);
  Serial.println(TC->CC[0].reg);
  
  while (TC->STATUS.bit.SYNCBUSY == 1);
  
}

void TC3_Handler() {
  
  TcCount16* TC = (TcCount16*) TC3;
  // If this interrupt is due to the compare register matching the timer count
  // we sample the data
  
  
  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    
   
   //digitalWrite(A2, LOW);
    digitalWrite(A3, LOW);
   //digitalWrite(A4, LOW);
    analogReadResolution(12); //SHIFT TO SETUP

    if ( writeIndex  >= arrayLength)
    { 
      writeIndex = 0;
    //flag
    }
    
   data[writeIndex] = analogRead(A2); //Reads the analog value on pin A2 
    writeIndex++; 
    
    data[writeIndex] = analogRead(A3); //Reads the analog value on pin A3 
    writeIndex++;
    

   data[writeIndex] = analogRead(A4); //Reads the analog value on pin A4
   writeIndex++;

   data[writeIndex] = analogRead(A5); //Reads the analog value on pin A5 
   writeIndex++;

  newData = true;
   
  }
}

uint16_t getChecksum( uint12_t *data, int count )
{
   uint16_t sum1 = 0;
   uint16_t sum2 = 0;
   int index;
 
   for( index = 0; index < count; ++index )
   {
     sum1 = (sum1 + data[index]) % 4095; //2^12 = 4096 - CHANGE TO 16
     sum2 = (sum2 + sum1) % 4095;
   }

   return (sum2 << 12) | sum1;
 }
