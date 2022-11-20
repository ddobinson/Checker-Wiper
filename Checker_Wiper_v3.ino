

//LIBRARIES
#include <SPI.h>
#include <Wire.h>
#include <MFRC522.h>
#include <MFRC522Extended.h>  //Library extends MFRC522.h to support RATS for ISO-14443-4 PICC.
#include <LCD5110_Graph.h> 
#include <DS3232RTC.h>
#include <TimeLib.h>
//#include <Streaming.h>      //not sure this is needed? Dan
#include "pitches.h"     //if error message involving pitches.h comes up then make sure pitches.h file is in the same directory as the sketch
#include "iTimeAfricaV1.h" //comment out if using V2 (Green) PCB
//#include "iTimeAfricaV2.h" //comment out if using V1 (Blue) PCB  and also change lines 298 according to PCB used)
#include <SoftwareSerial.h>
#include <stdlib.h>
#include <EEPROM.h>

//GLOBAL DEFINITIONS
int tagCount = 0;                                           // Cont that incremetns on eatch tag read
int flashLed = LOW;                                         // LED state to implement flashing
int typeIndex = 0xFF;                                       // Index for pod identity in the pod Type List
unsigned long sqCounter = 0;                                // Couner that increments in intterup routine
unsigned long baseTimeS;                                    // Base time in sec from RTC to withc we sync the ms count
bool battGood = 1;
Prog thisPod ;

// RFID COMPONENT Create MFRC522 instance
MFRC522 mfrc522(RFID_CS, LCD_RST);                            // PH get pins from PCB header file 
MFRC522::MIFARE_Key key;

// LCD COMPONENT
// LCD5110 myGLCD(4,5,6,A1,7); V2.0 green pcb
// LCD5110 myGLCD(4,5,6,7,8); (v1.0 blue PCB)
LCD5110 myGLCD(LCD_CLK, LCD_DIN , LCD_COM , LCD_RST, LCD_CE);   // PH get pins from PCB header file
extern unsigned char TinyFont[];      // for 5110 LCD display
extern unsigned char SmallFont[];     // for 5110 LCD display
extern unsigned char MediumNumbers[]; // for 5110 LCD display
extern unsigned char BigNumbers[];    // for 5110 LCD display

// BLUETOOTH COMPONENT
SoftwareSerial BT(UART_RX, UART_TX); // creates a "virtual" serial port/UART 
int progid = 0;      //define program ID for bluetooth programs defined in global definations (1,2,3,4 etc)           

/*
 * BELOW IS STATION/POD IDENTIFICATION!
 * 
 */

  // variables used for converting epoch time to readable time
unsigned long temp0=0, temp1=0, temp2=0, temp3=0, temp4=0,
              temp5=0, temp6=0, temp7=0, temp8=0, temp9=0,
              temp10=0, hours=0, mins=0, secs=0, MilliS=0;

//epoch time storage variables
unsigned long ss1Start=0, ss1Finish=0, SS1Time=0,   
              ss2Start=0, ss2Finish=0, SS2Time=0,
              ss3Start=0, ss3Finish=0, SS3Time=0,
              ss4Start=0, ss4Finish=0, SS4Time=0,
              ss5Start=0, ss5Finish=0, SS5Time=0,
              ss6Start=0, ss6Finish=0, SS6Time=0,
              SS1TimeMilliS=0, SS2TimeMilliS=0, SS3TimeMilliS=0,
              SS4TimeMilliS=0, SS5TimeMilliS=0, SS6TimeMilliS=0,
              totalRaceTime=0;
              
byte buffer[18];
char myRaceno[17];  

void setup() 
{
    Wire.begin();                                         // Init I2C buss
 
    // Serial.begin(57600);             // PH to use status LED serial needs to be disbaled
    BT.begin(9600);                     // Initialize serial communications with BT (Check default Baud Rate of BT module)
    myGLCD.InitLCD();                                     // initialise LCD
    initIoPins();
    initMfrc522();
    configrePod();
}

int updateCnt =0;
void loop() 
{
    updateLcdScreen();                            // invoke LCD Screen fucntion
    // when setSyncInterval(interval); runs out the status is changed to "timeNeedsSync"
    if ( timeStatus()  == timeNeedsSync ) {      
        setSyncProvider(RTC.get);           // manual resynch is done
    }  
  if ( ! mfrc522.PICC_IsNewCardPresent())     // Look for new cards
      return;
 
  if ( ! mfrc522.PICC_ReadCardSerial())       // Select one of the cards
      return;
  else {    
      if(ProgList[typeIndex].progid == 1){         // Runs  BT program 4
              readCardData();    //If value is 2 then start function
                 delay(1000);
      }      

      if(ProgList[typeIndex].progid == 2){         // Runs   program 4
      
              wipeTimes();    //If value is 4 then invoke wipetimes function
      }
  mfrc522.PICC_HaltA();                       // Halt PICC
  mfrc522.PCD_StopCrypto1();                  // Stop encryption on PCD     
  } 
     
}
/*
// interupt is triggered on up and down flanks, we only want one flank
ISR (PCINT1_vect) {     // handle pin change interrupt for A0 to A5 here
    if ( digitalRead(SQ_WAVE) == HIGH) {
        sqCounter ++;                 
    }
}
*/
void readCardData()
{
  Serial.begin(9600);                     // Initialize serial communications with BT (Check default Baud Rate of BT module)

  // Display Column headings (only if bluetooth connected before configure() completes)
  Serial.println();
  Serial.print("WATCH ID: "); Serial.print("\t");

  byte size = sizeof(buffer);
  byte status;
  byte block;
  
  String string0;
  String string1;
  String string2;
  String string3;
  String string4;
  String fullstring;

/*
 * WATCH NUMBER
 */
 
block = 4;

// Authentication
 
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A,block, &key, &(mfrc522.uid));
   
    if (status != MFRC522::STATUS_OK){ 
//       BT.print(F("PCD_Authenticate() failed: "));
//       BT.println(mfrc522.GetStatusCodeName(status));
         Serial.println(F("Error-ReScan"));
//         BT.println(); 
//         myGLCD.setFont(SmallFont);
 //        myGLCD.print("Error-ReScan"),LEFT,40);
 //       delay (2000);
        return;  }

// Read data from the block into buffer
    
    status = mfrc522.MIFARE_Read(block, buffer, &size);
    if (status != MFRC522::STATUS_OK){
    Serial.println(F("Error-ReScan"));
//         BT.println();
  //       myGLCD.setFont(SmallFont); 
  //       myGLCD.print("Error-ReScan"),LEFT,40);
//         delay (2000);
    return;   }

// Write buffer to serial console
//    BT.print("WATCH NUMBER IS: ");
    for (uint8_t i = 0; i < 16; i++) 
      {
        Serial.write(buffer[i]) ;   //writes data byte by byte to console
      }


//  BT.print("\t") ;             //tab separator
//  BT.println();

  
  string0 = char(buffer[0]);
  string1 = char(buffer[1]);
  string2 = char(buffer[2]);
  string3 = char(buffer[3]);
  string4 = char(buffer[4]);

fullstring = (string0 + string1 + string2 + string3 + string4);  


  fullstring.toCharArray(myRaceno,17);                                    // copy fullstring content into myString
    myGLCD.setFont(SmallFont);
    myGLCD.print("Watch Number:",CENTER,0);          //Instruction
    myGLCD.setFont(BigNumbers);
    myGLCD.print( String(myRaceno),CENTER,10);               // write race number LCD bottom row

    myGLCD.update();
    myGLCD.clrScr();   

  beepsLights();
    tagCount ++; 
 
}
/*
void wipeCard()                               //Wipes data from ALL writeable blocks!
{
byte buffer[] = {0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30};
MFRC522::StatusCode status;

byte dataBlocks[] = {1,2,4,5,6,8,9,10,12,13,14,16,17,18,20,21,22,
                    24,25,26,28,29,30,32,33,34,36,37,38,40,41,42,
                    44,45,46,48,49,50,52,53,54,56,57,58,60,61,62};
                          
    BT.println(F("*** DELETING ALL DATA... ***"));    //shows in serial that it is ready to read
    BT.println();

    
    for(int i = 0; i < sizeof(dataBlocks); i++) {
    status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, (dataBlocks[i]), &key, &(mfrc522.uid));
     if (status != MFRC522::STATUS_OK) {
         BT.println(F("PCD_Authenticate() failed: "));
         return;
     }
      
    status = mfrc522.MIFARE_Write((dataBlocks[i]), buffer, 16);
      if (status != MFRC522::STATUS_OK) {
          BT.println(F("MIFARE_Write() failed: "));
        return;
      }
      else {
      BT.print(F("All blocks deleted"));BT.println(dataBlocks[i]);
      }
    }
      BT.println(F("WATCH DATA DELETED"));
      BT.println(F("PRESENT NEXT CARD FOR DATA WIPE"));
      BT.println();
      beepsLights();
          tagCount ++; 
}
*/

void beepsLightswipeTimes()                                            // Beeps and leds for succesfull scan
{
  int melody[] = {NOTE_A7, NOTE_C7 };        // notes in the melody:
  int noteDurations[] = {8, 8};                        // note durations 4 = quarter note, 8 = eighth note, etc.::
    
    for (int thisNote = 0; thisNote < 2; thisNote++) {
        int noteDuration = 1000 / noteDurations[thisNote];      //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
        tone(BUZZER, melody[thisNote], noteDuration);
        digitalWrite(LED_TOP, LOW);                             // turn  LED on
        digitalWrite(LED_BOT, LOW);                             // turn  LED on

        int pauseBetweenNotes = noteDuration * 1.30;            // the note's duration + 30% seems to work well:
        delay(pauseBetweenNotes);
    
        noTone(BUZZER);                                         // stop the tone playing:
        digitalWrite(LED_TOP, HIGH);
        digitalWrite(LED_BOT, HIGH);
    }
}

void wipeTimes()                               //Wipes TIME data from  writeable blocks!
{
byte buffer[] = {0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30};
MFRC522::StatusCode status;

byte dataBlocks[] = {8,9,12,13,16,17,20,21,24,25,28,29,32,33,36,37}; //dan took out unused datablocks to speed up wipe

                          
    BT.println(F("*** DELETING TIME DATA... ***"));    //shows in serial that it is ready to read
    BT.println();

    
    for(int i = 0; i < sizeof(dataBlocks); i++) {
    status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, (dataBlocks[i]), &key, &(mfrc522.uid));
     if (status != MFRC522::STATUS_OK) {
         BT.println(F("PCD_Authenticate() failed: "));
         return;
     }
      
    status = mfrc522.MIFARE_Write((dataBlocks[i]), buffer, 16);
      if (status != MFRC522::STATUS_OK) {
          BT.println(F("MIFARE_Write() failed: "));
        return;
      }
      else {
      BT.print(F("All time blocks deleted"));BT.println(dataBlocks[i]);
      }
    }
      BT.println(F("TIME DATA DELETED"));
      BT.println(F("PRESENT NEXT CARD FOR DATA WIPE"));
      BT.println();
     beepsLightswipeTimes();
          tagCount ++;
}

unsigned long flashCount = 0;
String getBatteryVoltage() {
    int sensorValue = analogRead(V_MEAS);          // read the input on analog pin A2 for battery voltage reading:
    //Serial.println(String(sensorValue));
//    float voltage   = (float)sensorValue / 155;      // For V2 (Green) PCB Convert the analog reading to a voltage adjusted to Pieters multimeter
    float voltage   = (float)sensorValue / 269;      // For V1 (Blue) PCBConvert the analog reading to a voltage adjusted to Pieters multimeter
    String batMsg   = "Good";
    if (voltage > BATT_GOOD)  {                   // If Voltage higher than 3.8 display "Good" on LCD
        batMsg    = "Good";
        flashLed  = LOW;
        battGood = 1;
    } else if ( voltage > BATT_LOW ) {            // If Voltage lower than 3.8 display "Turn off POD" on LCD and flash lights. POD will work until 3.5V but will stop working after that.
        batMsg = "OK";
        battGood = 1;
        if ( flashCount < sqCounter ){
            flashCount = sqCounter + BATT_OK_FLASH;            // set toggle time to 5s
            if ( flashLed == LOW ) {                 
              flashLed = HIGH;                        // toggle status led previouse state 
            } else {
               flashLed = LOW;
            }
        }
    } else {
        battGood = 0;
        if ( flashCount < sqCounter ){
            flashCount = sqCounter + BATT_LOW_FLASH;            // set toggle time to 1s
            if ( flashLed == LOW ) {                 
              flashLed = HIGH;                        // toggle status led previouse state 
            } else {
              flashLed = LOW;
            }
        }
        batMsg = "PowerOff";             
    }
    char volt[5];
    String messageBattery = dtostrf(voltage,1,2,volt);       // conver float to string
    //Serial.println(messageBattery);
    messageBattery        = messageBattery + "v " + batMsg;  // append status 
    digitalWrite(STATUS_LED, flashLed); 
    return messageBattery;
}

// Displays main Screen Time & Pod station ID
void updateLcdScreen()    
{

    myGLCD.invert (false); //turn oFF inverted screen    
    myGLCD.setFont(SmallFont);
    
    // 1st line of display
    myGLCD.invertText (true); //turn on inverted text
    myGLCD.print((thisPod.progname),CENTER,0);         // Displays POD ID 
    myGLCD.invertText (false); //turn off inverted text

    //2nd line  Display

    myGLCD.print("Hold watch",CENTER,10);          //Instruction

    // 3rd Line
    myGLCD.print("until beeps",CENTER,20);          //Instruction

    // 4th Line
 //   int temp    = RTC.temperature();
 //   int mC      = ((temp*100) % 400 ) /100;
  //  String msg  = String(temp/4);
 //   msg         = msg + "." + mC + "C";
//    myGLCD.print(msg,LEFT,30);     
    myGLCD.print("Last Watch: ",LEFT,30);    
    myGLCD.print( String(myRaceno),RIGHT,30);

    // 5th Line
    String messageBattery = getBatteryVoltage();  
    myGLCD.print(messageBattery,CENTER,40);

    myGLCD.update();
    myGLCD.clrScr();     
            
}

void beepsLights()                                            // Beeps and leds for succesfull scan
{
     int length = 4;
    int melody[length] = {NOTE_A7, NOTE_A7, NOTE_A7, NOTE_A7};        // these are the only clear notes with buzzer used!
    int noteDurations[length] = {8, 8, 8, 8}; 
    for (int thisNote = 0; thisNote < 4; thisNote++) {
        int noteDuration = 1000 / noteDurations[thisNote];      //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
        tone(BUZZER, melody[thisNote], noteDuration);
        digitalWrite(LED_TOP, LOW);                             // turn  LED on
        digitalWrite(LED_BOT, LOW);                             // turn  LED on

        int pauseBetweenNotes = noteDuration * 1.30;            // the note's duration + 30% seems to work well:
        delay(pauseBetweenNotes);
    
        noTone(BUZZER);                                         // stop the tone playing:
        digitalWrite(LED_TOP, HIGH);
        digitalWrite(LED_BOT, HIGH);
    }
}

// These functions are to define the POD role and sync ms with seconds

void configrePod() {
    //turn on all LEDS for config mode warning
    digitalWrite(STATUS_LED, LOW);    
    digitalWrite(LED_TOP, LOW);                       
    digitalWrite(LED_BOT, LOW);
    myGLCD.invert (true);
    // read last pod type index from eeprom and select that identiry as default
    typeIndex = EEPROM.read(ADDRESS_PROG);
    unsigned long ConfigTimeoutMs = millis() + CONFIG_TYME_MS;
    unsigned long msLeft = CONFIG_TYME_MS;
    while ( millis() < ConfigTimeoutMs ) {
        // check for card read and incremetn typeIndex
        if ( mfrc522.PICC_IsNewCardPresent()) {    // Look for new cards
            typeIndex++;
        }
        if (typeIndex >= LIST_SIZE1) {
            typeIndex = 0;                          // if uninitilized set to 0
        }
        thisPod = ProgList[typeIndex];
        // display curretn pod id and start countdown to applying config
        //Serial.println("array size = " + sizeof(thisPod));
        msLeft = ConfigTimeoutMs - millis();
        configDisplay (msLeft );
        delay (100);
//        Serial.println(String(typeIndex) + "   "+ String(msLeft));
//        Serial.println(ProgList[typeIndex].progid);
//        Serial.println(ProgList[typeIndex].progname);

    }
    // save pod type in EEPROM for next boot
    EEPROM.write(ADDRESS_PROG, typeIndex);
    //turn off all LEDS for config mode warning
    digitalWrite(STATUS_LED, HIGH);
    digitalWrite(LED_TOP, HIGH);                       
    digitalWrite(LED_BOT, HIGH);  

  if(ProgList[typeIndex].progid == 2){         // Runs BT program 2

  // Display Column headings (only if bluetooth connected before configure() completes)
  BT.print("FIRST NAME "); BT.print("\t");
  BT.print("SURNAME "); BT.print("\t");
  BT.print("WATCH NUMBER "); BT.print("\t");
  BT.print("SS 1 "); BT.print("\t");
  BT.print("SS 2 "); BT.print("\t");
  BT.print("SS 3 "); BT.print("\t");
  BT.print("SS 4 "); BT.print("\t");
  BT.print("SS 5 "); BT.print("\t");
  BT.print("SS 6 "); BT.print("\t");
  BT.print("TOTAL"); BT.print("\t");
  BT.print("Stages Done"); BT.print("\t");
  BT.println();
    }
}


// Display fucntion during config loop
void configDisplay( unsigned long timeLeftMs)   {
//    myGLCD.invert (true);
     // 1st line of display
    myGLCD.setFont(SmallFont);
    String msg = VERSION;
    msg = "Watch Editor";
    myGLCD.print(msg,CENTER,0);       

   //2nd line  Display
    myGLCD.setFont(SmallFont);          
    myGLCD.print("WARNING!",CENTER,10);        

    //3rd line  Display
    myGLCD.setFont(SmallFont);          
    myGLCD.print((thisPod.progname),CENTER,20);        
 
    // 4th Line
    myGLCD.setFont(SmallFont);
    myGLCD.print("Swipe2Change",CENTER,30);

    // 5th Line
    String timeMsg = "Done in ";
    unsigned long ms = (timeLeftMs % 1000)/100;
    timeMsg = timeMsg + (timeLeftMs/1000) + "." + ms + "s";
    myGLCD.setFont(SmallFont);
    myGLCD.print(timeMsg,LEFT,40);

    myGLCD.update();
    myGLCD.clrScr();
}

// All Genral Purpouse IO pin Setup stuffs 
void initIoPins() {
    // Input Pins
    pinMode(SQ_WAVE, INPUT);

    // Output Pins
    pinMode(LED_TOP, OUTPUT);
    pinMode(LED_BOT, OUTPUT);
    pinMode(STATUS_LED, OUTPUT);
    // set pins start condition: LOW = LED on, HIGH = LED off
    digitalWrite(LED_TOP, HIGH);                       
    digitalWrite(LED_BOT, HIGH);                       
    digitalWrite(STATUS_LED, LOW);                     
}

// All RFID init Stuffs
void initMfrc522() {
    SPI.begin();                                          // Init SPI bus
    mfrc522.PCD_Init();                                   // Init MFRC522 card
 //   mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);       // Enhance the MFRC522 Receiver Gain to maximum value of some 48 dB (default is 33dB)

    for (byte i = 0; i < 6; i++) {                // Prepare the key (used both as key A and as key B) using FFFFFFFFFFFFh which is the default at chip delivery from the factory
        key.keyByte[i] = 0xFF;
    }
}
