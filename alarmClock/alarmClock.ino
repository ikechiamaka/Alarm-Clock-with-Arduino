#include <Encoder.h>  // This library allows the use of a rotary encoder
#include <Wire.h>     // Required for I2C communication with the RTC
#include "RTClib.h"   // Library for the DS3231 real-time clock
#include <TM1637.h>   // Library for the 4-digit 7-segment LED display
#include <EEPROM.h>   // Library for reading/writing data to the microcontroller's EEPROM

#include "SoftwareSerial.h"
#include "Adafruit_Soundboard.h"
//encoder connected to pins 2 and 3 (and ground)
#define buton 6
#define CLK 9  //Pins for TM1637
#define DIO 8
#define buzzer 7  // buzzer
// soundboard pins and setup
#define SFX_RST 10
#define SFX_RX 11
#define SFX_TX 12
const int ACT = 13;
// Use pins 2 and 3 to communicate with DFPlayer Mini
// static const uint8_t PIN_MP3_TX = 10;  // Connects to module's RX
// static const uint8_t PIN_MP3_RX = 11;  // Connects to module's TX
// SoftwareSerial softwareSerial(PIN_MP3_RX, PIN_MP3_TX);
SoftwareSerial ss = SoftwareSerial(SFX_TX, SFX_RX);
Adafruit_Soundboard sfx = Adafruit_Soundboard( &ss, NULL, SFX_RST);

TM1637 tm1637(CLK, DIO);

// Create the Player object


// audio track names on soundboard
char alarm[] =              "SOUNDS~1WAV";

// Date and time functions using a DS3231 RTC connected via I2C and Wire lib
Encoder knob(3, 2);
RTC_DS3231 rtc;

const int wirePin = 4;
const int endPin = 5;

int hours, minutes;
int addr = 0;
int addr1 = 1;

// These variables are for the push button routine
int buttonstate = 0;             //flag to see if the button has been pressed, used internal on the subroutine only
int pushlengthset = 2500;        // value for a long push in mS
int pushlength = pushlengthset;  // set default pushlength
int pushstart = 0;               // sets default push value for the button going low
int pushstop = 0;                // sets the default value for when the button goes back high


// Variables for storing values in EEPROM
byte value;
byte value1;

// Variable for storing the rotation of the knob
int knobval;  // value for the rotation of the knob

// Flag to indicate if the button has been pressed
boolean buttonflag = false;  // default value for the button flag

// Variables for storing the current time and the alarm time
int sethourstemp;
int setminstemp;
int setalarmhourstemp, setalarmminstemp;
byte Previoussetalarmhourstemp, Previoussetalarmminstemp;

// Flags for indicating if the alarm is set and if it is on
bool alarmSet = false;
bool alarmOn = false;

// Pin for the LED
int ledPin = 10;

void setup() {
  // Initialize the 4-digit LED display
  tm1637.init();
  tm1637.set(5);

  // Start serial communication at 9600 baud
  Serial.begin(9600);
  ss.begin(9600);

  // Wait for half a second
  delay(500);

  // Start the real-time clock
  rtc.begin();

  // Configure the digital pins as inputs or outputs
  pinMode(ACT, INPUT);
  pinMode(wirePin, INPUT);
  pinMode(endPin, INPUT);
  pinMode(buton, INPUT);      //push button on encoder connected
  digitalWrite(buton, HIGH);  //Pull at 1 high
  pinMode(ledPin, OUTPUT);
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW);  //set in 0 logical level
  tone(buzzer, 4000, 50);

}  //end "setup()"

void playAudio( char* trackname, int playing ) {
  // stop track if one is going
  if (playing == 0) {
    sfx.stop();
  }

  // now go play
  if (sfx.playTrack(trackname)) {
    sfx.unpause();
  }
}


void loop() {
  // Get current time from real-time clock
  DateTime now = rtc.now();
  hours = now.hour(), DEC;
  minutes = now.minute(), DEC;

// find out of the audio board is playing audio
  int playing = digitalRead(ACT);

  // Display the current time on the LED display
  tm1637.point(POINT_ON);
  if ((hours / 10) == 0) tm1637.display(0, 17);
  else
    tm1637.display(0, hours / 10);  // hour
  tm1637.display(1, hours % 10);
  tm1637.display(2, minutes / 10);  // minutes
  tm1637.display(3, minutes % 10);  //
  delay(500);

  // Get the length of the push from the encoder
  pushlength = pushlengthset;
  pushlength = getpushlength();
  delay(10);


  // Check if the knob has been pressed for more than 3 seconds to enter alarm mode
  if (pushlength > pushlengthset) {
    Serial.println("in alarm mode..");
    tone(buzzer, 2000, 50);

    // Get the current time
    DateTime now = rtc.now();

    // Check if the alarm is already set
    if (alarmSet) {
      // Turn off alarm
      alarmOn = false;
      digitalWrite(ledPin, LOW);
    } else {
      // Set new alarm time
      tone(buzzer, 2000, 50);
      setAlarm();
      pushlength = pushlengthset;
    }
  }

  // Check if the knob has been pressed for a short time to enter setting the clock mode
  if (pushlength < pushlengthset) {
    Serial.println(pushlength);
    tone(buzzer, 1000, 50);

    // Get the current time
    DateTime now = rtc.now();
    sethourstemp = now.hour(), DEC;
    setminstemp = now.minute(), DEC;
    setclock();
    pushlength = pushlengthset;
  };

  // Check if alarm time is same as current time
  if (hours == setalarmhourstemp && minutes == setalarmminstemp && alarmOn) {
    digitalWrite(ledPin, HIGH);
    Serial.println("Alarm ON");
    playAudio(alarm, playing);

   
  } else {
    digitalWrite(ledPin, LOW);
    Serial.println("Alarm OFF");
  }

  tm1637.point(POINT_OFF);
  if ((hours / 10) == 0) tm1637.display(0, 17);
  else
    tm1637.display(0, hours / 10);  // hour
  tm1637.display(1, hours % 10);
  tm1637.display(2, minutes / 10);  // minutes
  tm1637.display(3, minutes % 10);  //
  delay(500);


}  // end loop()




void setAlarm() {
  // Code for setting alarm time using push buttons and/or a knob
  setalarmhours();
  delay(500);
  setalarmmins();
  delay(500);
  alarmSet = true;
  alarmOn = true;

  Serial.print("Alarm set to : ");
  Serial.print(setalarmhourstemp);
  Serial.print(" : ");
  Serial.println(setalarmminstemp);
  EEPROM.write(addr, setalarmhourstemp);
  EEPROM.write(addr1, setalarmminstemp);

  if (EEPROM.read(addr) != setalarmhourstemp) {
    EEPROM.write(addr, setalarmhourstemp);
  }

  if (EEPROM.read(addr1) != setalarmminstemp) {
    EEPROM.write(addr1, setalarmminstemp);
  }
  value = EEPROM.read(addr);
  value1 = EEPROM.read(addr1);
  Serial.print(addr);
  Serial.print("==");
  Serial.print(value, DEC);
  Serial.println();
  Serial.print(addr1);
  Serial.print("==");
  Serial.print(value1, DEC);
  Serial.println();
  addr = addr + 1;
  addr1 = addr1 + 1;
  if (addr == EEPROM.length()) {
    addr = 0;
  }
  if (addr1 == EEPROM.length()) {
    addr1 = 0;
  }

  delay(100);
}








// subroutine to return the length of the button push.
int getpushlength() {
  buttonstate = digitalRead(buton);
  if (buttonstate == LOW && buttonflag == false) {
    pushstart = millis();
    buttonflag = true;
  };

  if (buttonstate == HIGH && buttonflag == true) {
    pushstop = millis();
    pushlength = pushstop - pushstart;
    buttonflag = false;
  };
  return pushlength;
}

int sethours() {
  tm1637.display(0, sethourstemp / 10);  // hour
  tm1637.display(1, sethourstemp % 10);
  tm1637.display(2, 16);  // minutes
  tm1637.display(3, 16);  //
  pushlength = pushlengthset;
  pushlength = getpushlength();
  if (pushlength != pushlengthset) {
    return sethourstemp;
  }
  knob.write(0);
  delay(50);
  knobval = knob.read();
  if (knobval < -1) {
    knobval = -1;
  }
  if (knobval > 1) {
    knobval = 1;
  }
  sethourstemp = sethourstemp + knobval;
  if (sethourstemp < 0) {
    sethourstemp = 23;
  }
  if (sethourstemp > 23) {
    sethourstemp = 0;
  }
  sethours();
}

int setalarmhours() {
  tm1637.display(0, setalarmhourstemp / 10);  // hour
  tm1637.display(1, setalarmhourstemp % 10);
  tm1637.display(2, 16);  // minutes
  tm1637.display(3, 16);  //
  pushlength = pushlengthset;
  pushlength = getpushlength();
  if (pushlength != pushlengthset) {
    return setalarmhourstemp;
  }
  knob.write(0);
  delay(50);
  knobval = knob.read();
  if (knobval < -1) {
    knobval = -1;
  }
  if (knobval > 1) {
    knobval = 1;
  }
  setalarmhourstemp = setalarmhourstemp + knobval;
  if (setalarmhourstemp < 0) {
    setalarmhourstemp = 23;
  }
  if (setalarmhourstemp > 23) {
    setalarmhourstemp = 0;
  }

  setalarmhours();
}
int setalarmmins() {
  tm1637.display(0, 16);  // hour
  tm1637.display(1, 16);
  tm1637.display(2, setalarmminstemp / 10);  // minutes
  tm1637.display(3, setalarmminstemp % 10);  //
  pushlength = pushlengthset;
  pushlength = getpushlength();
  if (pushlength != pushlengthset) {
    return setalarmminstemp;
  }
  knob.write(0);
  delay(50);
  knobval = knob.read();
  if (knobval < -1) {
    knobval = -1;
  }
  if (knobval > 1) {
    knobval = 1;
  }
  setalarmminstemp = setalarmminstemp + knobval;
  if (setalarmminstemp < 0) {
    setalarmminstemp = 59;
  }
  if (setalarmminstemp > 59) {
    setalarmminstemp = 0;
  }

  setalarmmins();
}

int setmins() {
  tm1637.display(0, 16);  // hour
  tm1637.display(1, 16);
  tm1637.display(2, setminstemp / 10);  // minutes
  tm1637.display(3, setminstemp % 10);  //
  pushlength = pushlengthset;
  pushlength = getpushlength();
  if (pushlength != pushlengthset) {
    return setminstemp;
  }
  knob.write(0);
  delay(50);
  knobval = knob.read();
  if (knobval < -1) {
    knobval = -1;
  }
  if (knobval > 1) {
    knobval = 1;
  }
  setminstemp = setminstemp + knobval;
  if (setminstemp < 0) {
    setminstemp = 59;
  }
  if (setminstemp > 59) {
    setminstemp = 0;
  }
  setmins();
}

//sets the clock
void setclock() {
  sethours();
  delay(500);
  setmins();
  delay(500);
  rtc.adjust(DateTime(2017, 4, 1, sethourstemp, setminstemp, 0));
  delay(500);
}