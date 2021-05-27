#include <Stepper.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>      // LCD Library
LiquidCrystal_I2C lcd(0x27,16,2);; // Connect LCD RS, E, D4, D5, D6, D7 to these pins on Arduino.


//  AD9850 Variables

#define  CLOCK A4            // Pin connections for AD9850 Module.
#define  LOAD  A5
#define  DATA  A3            // Note that A6 and A7 are ANALOG ONLY.
#define  RESET A2
#define DDS_CLOCK 125000000  // Timebase of oscillator on AD9850 module.

unsigned long gran=1000L;  // Set initial granulatiry.
unsigned long setFreq=14e6; // Initial Set Frequency

unsigned long oldFreq;     // Holder for previous frequency
int CTS=0;                 // Clear-To-Send flag. 0=encoder not ready (positioned) for poll. 1=ready
#define maxFreq 5.9e7L     // Maximum Output Frequency
#define minFreq 0L         // Minimum Output Frequency
#define maxGran 1e6L       // Max. granularity before cycling down.
#define minGran 1L         // Min. granularity.

#define encPinA 10     // Pin connections for rotary encoder. These pins get uC internal pullup.
#define encPinB 9      // Center pin goes to GND. No cap on rotary pins.
#define encSw 7        // Pin connection for switch on encoder. Uses uC internal pullup. 1 nF cap from here to GND. 
#define longPress 8e4  // Number of cycle iterations to be considered "long" press.
#define shortWait 1e4  // Number of cycles to wait before updating diplayed and output frequency.
#define pulsedelay 1   // Used for DDS module serial timing.

const int stopPB=2;
const int speedINC = 3;
const int speedDEC = 4;
const int reverse=5;

const int motorPIN[] = { 22, 23, 24, 25};
const int direction = -1;
const int stepsPerRevolution = 200;
int speedStep = 5;
int stepMinimum = 10;
int stepMaximum = 300;
int stopType = 0;

int currentSpeed=60;
int currentSPR = stepsPerRevolution;

#define START 1
#define STOP 0
#define CLK 1
#define CTCLK -1
int motorStopState=STOP;
int reverseState=STOP;

Stepper myStepper(stepsPerRevolution, motorPIN[0], motorPIN[1], motorPIN[2], motorPIN[3]);

void setup()
{
byte customChar[] = {
  B10100,
  B10100,
  B10100,
  B11111,
  B10101,
  B10110,
  B10111,
  B00000
};

  pinMode (encPinA, INPUT_PULLUP);  // Enable pullups on all encoder inputs.
  pinMode (encPinB, INPUT_PULLUP);
  pinMode (encSw,   INPUT_PULLUP);
  
  pinMode (DATA,  OUTPUT); // Initialize control pins to AD9850 module as output.
  pinMode (CLOCK, OUTPUT); 
  pinMode (LOAD,  OUTPUT); 
  pinMode (RESET, OUTPUT); 

  pinMode(speedINC, INPUT_PULLUP);
  pinMode(stopPB,INPUT_PULLUP);
  pinMode(speedDEC,INPUT_PULLUP);
  pinMode(reverse, INPUT_PULLUP);
  
  lcd.init();      // initialize the LCD
  lcd.backlight();
  lcd.createChar(0, customChar);
  
  
  
  lcd.clear();
  lcd.print("LeeLAB ");  // Put initial text on screen.
  DispGran(gran);       // Show current granularity
  DispFreq(setFreq);
  AD9850_init();           // Initialize the AD9850 module.
  AD9850_reset();          // Reset the module.
  SetFrequency(setFreq);   // Output the initial frequency.

}


void loop()
{

  oldFreq=setFreq;  // oldFreq is the frequency since before the display and DDS module were last updated.
  
  EncPoll( setFreq, CTS );  // Poll rotoary encoder. setFreq and CTL may be updated.

  EncSwPoll(setFreq, gran); // Poll switch on encoder. setFreq and gran may be updated.
  
  if( oldFreq!=setFreq )    // Don't do updates to display or DDS module unless the freq has changed.
  {
    DispFreq( setFreq );
    SetFrequency( setFreq );
  }

   updateState();
  boolean reverseState = digitalRead(reverse); 

  if(!motorStopState)
  {
    currentSPR =0;
  }else{
    currentSPR =stepsPerRevolution;  
  }
  
   myStepper.setSpeed(currentSpeed);
  if(direction ==1)
  {
    myStepper.step(-currentSPR);
  }else{
    myStepper.step(currentSPR);    
  }
  
  if(reverseState == LOW){
    myStepper.step(-1);
  }
}


// encReadAB
// Returns a 2-bit number based on the current encoder reading -- one bit for each pin.
//
unsigned char encReadAB( void )
{
  return (~(digitalRead( encPinA )*2+digitalRead( encPinB ))&3);
}


//  waitFor
//  Will wait until the ecoder returns the value of "number" or until
//  the shortWait time has elapsed.
int
waitFor( int number )
{
  unsigned int waitT=shortWait;
  
  while( encReadAB()!=number && waitT )
    waitT--;
  return waitT;
}


//  EncPoll
//  Polls the rotary encoder and updates the setFreq if a good rotation has been detected.
//  Uses CTS value to keep track of encoder state between calls. Will only continue processing
//  as long as a valid read is possible. The function terminates when an invalid read is detected
//  in order to wait for a potentially good read next time.
void EncPoll( unsigned long& setFreq, int& CTS )
{
  int ab;
  unsigned long betweenTimer;

  ab=encReadAB();
  
  if( ab==0 )
  {
    CTS=1;
    goto exit;
  }
  else if( ab==2 )
    { // ab==2
      if( CTS==0 )
        goto exit;
      else
      {
        if( !waitFor(3) )
         goto ctsExit;
        else
        { // ab==3
          if( !waitFor(1) )
            goto ctsExit;
          else
          { // ab==1
            if( !waitFor(0) )
              goto ctsExit;
            else
            {
              setFreq=setFreq+gran;
              if( setFreq>maxFreq )
                setFreq=maxFreq;
            }
          } // end of ab==1
        } // end of ab==3
      }
    } // end of ab==2
    else if( ab==1 )
    { // ab==1
      if( CTS==0 )
        goto ctsExit;
      else
      { // CTS==1
        if( !waitFor(3) )
         goto ctsExit;
        else
        { // ab==3
          if( !waitFor(2) )
            goto ctsExit;
          else
          { // ab==2
            if( !waitFor(0) )
              goto ctsExit;
            else
            {
              setFreq=setFreq-gran;
              if( setFreq>maxFreq )  // Need to do this since using unsigned variable.
                setFreq=minFreq;
            }
          } // end of ab==2
        } // end of ab==3
      } // end of CTS==1
    } // end of ab==1
    
    goto exit;
  ctsExit:
    CTS=0;
  exit:
  ;
}


//  EncSwPoll
//  Polls the push button on the rotary encoder. A short press will change the frequency
//  granularity. A long press will reset setFreq to the current level of granularity.
void EncSwPoll( unsigned long& setFreq, unsigned long& gran )
{
  unsigned long oldFreq=setFreq;
  unsigned long swTimer=0;
  
  if( !digitalRead( encSw ))   // Got a button push?
  {
    gran=gran/10l;             // Yes! Bump the granularity.
    if( gran<minGran )         // But cycle back to minimum granularity
      gran=maxGran;            // if max. granularity is exceeded.
    DispGran( gran );
 
    while( !digitalRead( encSw ) && swTimer<longPress )
      swTimer++;               // Detect a long button press
  
    if( swTimer>=longPress )   // If a long press was detected, then:
    {
      gran=gran*10L;           // Undo the previous gran bump.
      if( gran>maxGran )       // If minimum gran exceeded,
        gran=minGran;          // go back to top level of granularity.
      setFreq=gran;            // Set the current freq. to the gran level.
      DispGran( gran );        // Display the updated gran and frequency.
      DispFreq( setFreq );     // Doing it here gives visual feedback of long push.
      SetFrequency(setFreq );
    }
  }
  while( !digitalRead( encSw ))  // Exit after button is released.
    ;
}


//  DispFancyFreq
//  Displays the current setFreq to the LCD with separators.
void DispFreq( unsigned long setFreq )
{
  char oneChar;
  String freqStr="          ";
  String tmpStr;
  unsigned int strLen;
  
  tmpStr=String(setFreq); // tmpStr="12345678"
  strLen=tmpStr.length();
  
  oneChar=tmpStr[strLen-1];
  freqStr.setCharAt(9,oneChar); // Set 1st digit.

  oneChar=tmpStr[strLen-2];
  if( oneChar>='0' )                     // If 2nd digit exists,
    freqStr.setCharAt(8,oneChar); // set 2nd digit.
    
  oneChar=tmpStr[strLen-3];              // If 3rd digit exists,
  if( oneChar>='0' )                     // set 3rd digit.
    freqStr.setCharAt(7,oneChar);
  
  oneChar=tmpStr[strLen-4];              // If 4th digit exists,
  if( oneChar>='0' )                     // set comma and 4th digit.
  {
    freqStr.setCharAt(6,',');
    freqStr.setCharAt(5,oneChar);
  }
  
  oneChar=tmpStr[strLen-5];              // If 5th digit exists,
  if( oneChar>='0' )                     // set 5th digit.
    freqStr.setCharAt(4,oneChar);
    
  oneChar=tmpStr[strLen-6];              // If 6th digit exists,
  if( oneChar>='0' )                     // set 6th digit.
    freqStr.setCharAt(3,oneChar);
 
  oneChar=tmpStr[strLen-7];              // If 7th digit exists,
  if( oneChar>='0' )                     // set comma and 7th digit.
  {
    freqStr.setCharAt(2,',');
    freqStr.setCharAt(1,oneChar);
  }

   oneChar=tmpStr[strLen-8];                // If 8th digit exists,
   if( oneChar>='0' )              // set 8th digit.
     freqStr.setCharAt(0,oneChar);

  
  lcd.setCursor(0,1);
  lcd.print("Freq:         ");
  lcd.setCursor(6,1);
  lcd.print(freqStr);
//  lcd.print((char)0); // Print Hz character

//  lcd.print((char)0); // Print Hz character

}
//  DispFreq
//  Displays the current setFreq to the LCD.
void aDispFreq( unsigned long setFreq )
{
  unsigned long cutFreq;
  cutFreq=setFreq;
  lcd.setCursor(0,1);
  lcd.print("Freq:         ");
  lcd.setCursor(6,1);
  lcd.print(cutFreq);
  lcd.print(" ");
    lcd.print((char)0); // Print Hz character

}


//  DispGran
//  Updates the LCD with the current granularity value.
void DispGran(unsigned long gran)
{
  int caseGran;
  caseGran=log10(gran);  
  lcd.setCursor(11,0);
  switch( int(caseGran) )
  {
    case 0:
      lcd.print("   1");
      break;
    case 1:
      lcd.print("  10");
      break;
    case 2:
      lcd.print(" 100");
      break;
    case 3:
      lcd.print("  1k");
      break;
    case 4:
      lcd.print(" 10k");
      break;
    case 5:
      lcd.print("100k");
      break;
    case 6:
      lcd.print("  1M");
      break;
    default:
      lcd.print("error ");
      break;
  }
  lcd.print((char)0); // Print Hz character

}


//  SetFrequency
//  Serially sends the "frequency" to the DDS module.
void SetFrequency(unsigned long frequency)
{
  unsigned long tuning_word = (frequency * pow(2, 32)) / DDS_CLOCK;
  digitalWrite (LOAD, LOW); 

  shiftOut(DATA, CLOCK, LSBFIRST, tuning_word);
  shiftOut(DATA, CLOCK, LSBFIRST, tuning_word >> 8);
  shiftOut(DATA, CLOCK, LSBFIRST, tuning_word >> 16);
  shiftOut(DATA, CLOCK, LSBFIRST, tuning_word >> 24);
  shiftOut(DATA, CLOCK, LSBFIRST, 0x0);
  digitalWrite (LOAD, HIGH); 
}


void AD9850_init()
{
  digitalWrite(RESET, LOW);  // Initialize DDS module serial pins.
  digitalWrite(CLOCK, LOW);
  digitalWrite(LOAD, LOW);
  digitalWrite(DATA, LOW);
}


void AD9850_reset()
{
  //  Reset the DDS Module.
  //  Reset sequence is:
  //    CLOCK & LOAD = LOW
  //    Pulse RESET high for a few uS (use 5 uS here)
  //    Pulse CLOCK high for a few uS (use 5 uS here)
  //    Set DATA to ZERO and pulse LOAD for a few uS (use 5 uS here)

  // The data sheet diagrams show only RESET and CLOCK being used to reset the device,
  // but I see no output unless I also toggle the LOAD line here.

  digitalWrite(CLOCK, LOW);
  digitalWrite(LOAD, LOW);

  digitalWrite(RESET, LOW);
  delay(pulsedelay);
  digitalWrite(RESET, HIGH);  //pulse RESET
  delay(pulsedelay);
  digitalWrite(RESET, LOW);
  delay(pulsedelay);

  digitalWrite(CLOCK, LOW);
  delay(pulsedelay);
  digitalWrite(CLOCK, HIGH);  //pulse CLOCK
  delay(pulsedelay);
  digitalWrite(CLOCK, LOW);
  delay(pulsedelay);
  digitalWrite(DATA, LOW);    //make sure DATA pin is LOW

  digitalWrite(LOAD, LOW);
  delay(pulsedelay);
  digitalWrite(LOAD, HIGH);  //pulse LOAD
  delay(pulsedelay);
  digitalWrite(LOAD, LOW);
  // Chip is RESET now
}

void updateState()
{
  if(digitalRead(stopPB) ==LOW)
  {
   motorStopState =1-motorStopState;// stop the motor //motorStopState=STOP, STOP=0, START=1
    if(motorStopState  ==STOP)//motorstopstate ==0
    {
      stopMotor();
    }
   delay(500);
  }
  
  if(digitalRead(speedINC) ==LOW)
  {
  motorStopState = START;
  currentSpeed += speedStep;
   if( currentSpeed >= stepMaximum )  
   currentSpeed = stepMaximum ;
  }

  if(digitalRead(speedDEC) ==LOW)
  {
  motorStopState = START;    
   currentSpeed -= speedStep;
   if( currentSpeed <stepMinimum )  
   currentSpeed =stepMinimum ;
  }  
}


void stopMotor()
{
  if(stopType ==0)
  {
    for(int i=0; i<4; i++)
    {
      digitalWrite(motorPIN[i], LOW);
    }
  }
}
