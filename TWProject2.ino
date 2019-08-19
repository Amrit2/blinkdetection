#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024
#define arrayLength 6 // 2 sync + 1 length + 3 data + 1 checksum
#define syncByte 170
#define dataLength 3
#define channel1 A1
#define channel2 A2
#define channel3 A3

int writeIndex = 3;
int readIndex = 0;
int payLoad[arrayLength]; 

void startTimer(int frequencyHz);
void setTimerFrequency(int frequencyHz);
void TC3_Handler();
bool newData = false;

void setup() {
  Serial.begin(38400);
  pinMode(LED_BUILTIN, OUTPUT); //pin 13
  digitalWrite(LED_BUILTIN, HIGH); 
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  startTimer(10);
  
  pinMode(channel1, OUTPUT);
  pinMode(channel2, OUTPUT);
  pinMode(channel3, OUTPUT);
  
}

void loop() {
  payLoad[0] = syncByte;
  payLoad[1] = syncByte; 
  payLoad[2] = dataLength;

  if (newData){
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(100);
    while (readIndex <= arrayLength){
      Serial.print("Current index: ");   // change to Serial.write()
      Serial.print(readIndex);
      Serial.print(" = ");
      Serial.println(payLoad[readIndex]);
      readIndex++;
    }
    readIndex = 0;
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
   
      digitalWrite(channel1, LOW);
      digitalWrite(channel2, LOW);
      digitalWrite(channel3, LOW);
   
      analogReadResolution(12); //SHIFT TO SETUP

      if (writeIndex >= arrayLength)
      { 
        writeIndex = 3;
      }
    
      payLoad[writeIndex] = analogRead(channel1); //Reads the analog value on pin A2 
      writeIndex++; 
    
      payLoad[writeIndex] = analogRead(channel2); //Reads the analog value on pin A3 
      writeIndex++;

      payLoad[writeIndex] = analogRead(channel3); //Reads the analog value on pin A3 
      writeIndex++;
      newData = true;
    }
    uint16_t sum1 = 0;
    uint16_t sum2 = 0;
    int index;
 
    for( index = 3; index < 6; ++index )
    {
      sum1 = (sum1 + payLoad[index]) % 65536; //CHANGE TO 16: why??????
      sum2 = (sum2 + sum1) % 65536;
    }

    payLoad[writeIndex] = (sum2 << 16) | sum1;

}
