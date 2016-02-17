/* 
*  This script sends control signals to the Raycus fibre lasers DB25.
*  This program should be read in conjuction with the supporting schematic
*  and compatability document. No serial input is needed. The machine
*  operates using one dial and one 3 position switch. Information is output 
*  to an LED and to a 0-2V multimeter. The analog output A14 is used to drive 
*  the multimeter display.
*
*  Additionally, it controlls the output signals on the DB25 port for the laser.
*  The teensy pin configuration is as folllows:
*
* Pin Number |  I/O - type  |  Comments
* ------------------------------------------------------
*   1        | Input-Pullup | Laser is active ON when low (as long as 2 is HIGH)
*   2        | Input-Pullup | Input for pulse period when low (as long as 1 is HIGH)
*   3        | Output       | External LED. LED is on when HIGH.
*   4        | Input        | Alarm input LSB (00 = temp; 01 = norm; 10 = reflection; 11 = MO)
*   5        | Input        | Alarm input MSB
*   6        | Output       | Laser power level LSB (bit 0) - Active LOW
*   7        | Output       | Laser power level bit 1 - Active LOW
*   8        | Output       | Laser power level bit 2 - Active LOW
*   9        | Output       | Laser power level bit 3 - Active LOW
*   10       | Output       | Laser power level bit 4 - Active LOW
*   11       | Output       | Laser power level bit 5 - Active LOW
*   12       | Output       | Laser power level bit 6 - Active LOW
*   13       | Output       | Teensy on board LED. LED is on when HIGH.
*   14       | Output       | Laser power level MSB (bit 7) - Active LOW
*   15       | Output       | MO signal - Active LOW
*   16       | Output       | Laser on/off signal - Active LOW
*   17       | Output       | Laser pulses - Active LOW
*   18       | Output       | Laser Guide - Active LOW
*   22       | Input-Pullup | TTL laser on/off signal : Active high (ie, no BNC = ON all the time)
*   A14      | Output       | Analog output to display panel
*
*  This script was written by Adam Chrimes on behalf of OptoTech Pty. Ltd.
*  Copyright 2015.
*/

// ----------- includes --------------------------
#include <Metro.h> //Metro time signal generator - used for non-interrupt actions
#include <Bounce2.h> //Switch debouncing - used to manage electrical bouncing on the action switch

// --------- definitions and macros --------------
// Pin definitions:
#define ActionSwitchPin1 1
#define ActionSwitchPin2 2
#define ExternalLEDPin 3
#define AlarmLSBPin 4
#define AlarmMSBPin 5
#define LaserPowerBit0Pin 6
#define LaserPowerBit1Pin 7
#define LaserPowerBit2Pin 8
#define LaserPowerBit3Pin 9
#define LaserPowerBit4Pin 10
#define LaserPowerBit5Pin 11
#define LaserPowerBit6Pin 12
#define InternalLEDPin 13
#define LaserPowerBit7Pin 14
#define LaserMOSignalPin 15
#define LaserONSignalPin 16
#define LaserPulsesPin 17
#define LaserGuidePin 18
#define LaserTriggerPin 22
#define DialInputPin A9
#define DisplayOutputPin A14

// -------- Global variables ----------------------
IntervalTimer LaserPulseTimer; //Timer class for running the laser pulses
IntervalTimer AlarmLEDFlashTimer; //Timer for flashing the LED when there is an alarm
float LaserPulsePeriod = 12.5; //25 microseconds period = 40 kHz
Metro serialMetro = Metro(1000); //Timer for serial output
Metro alarmMetro = Metro(100); //Timer for sensing the alarm signals
uint8_t OperatingMode = 7; //Sets the operating mode of the laser system
uint8_t EnableLaserTrigger = 0; //Disables the laser trigger input reading: =0 is disable and !=0 is enable
uint8_t LaserPower = 0; //Stores the laser power level
float DisplayMultiplier = 1.240909; //Converts the desired mV number to ADC units
Bounce ActionBounce1 = Bounce();
Bounce ActionBounce2 = Bounce();
String AlarmState = "NONE";

// -------- Setup functions -----------------------
void setup() 
{
  Serial.begin(1); //Start the teensy serial port for output debug messages

  //define all pin directions and starting states
  pinMode(ActionSwitchPin1, INPUT_PULLUP); //Action switch set as input
  pinMode(ActionSwitchPin2, INPUT_PULLUP); //Action switch set as input
  pinMode(ExternalLEDPin, OUTPUT); //External LED set as output
  digitalWriteFast(ExternalLEDPin, LOW); //External LED turned OFF
  pinMode(AlarmLSBPin, INPUT); //Alarm pin set as input
  pinMode(AlarmMSBPin, INPUT); //Alarm pin set as input
  pinMode(LaserPowerBit0Pin, OUTPUT); //Laser power bit 0 set as output
  digitalWriteFast(LaserPowerBit0Pin, HIGH); //Laser power bit0 turned off
  pinMode(LaserPowerBit1Pin, OUTPUT); //Laser power bit 1 set as output
  digitalWriteFast(LaserPowerBit1Pin, HIGH); //Laser power bit1 turned off
  pinMode(LaserPowerBit2Pin, OUTPUT); //Laser power bit 2 set as output
  digitalWriteFast(LaserPowerBit2Pin, HIGH); //LAser power bit 2 turned off
  pinMode(LaserPowerBit3Pin, OUTPUT); //Laser power bit 3 set as output
  digitalWriteFast(LaserPowerBit3Pin, HIGH); //Laser power bit 3 turned off
  pinMode(LaserPowerBit4Pin, OUTPUT); //Laser power bit 4 set as output
  digitalWriteFast(LaserPowerBit4Pin, HIGH); //Laser power bit 4 turned off
  pinMode(LaserPowerBit5Pin, OUTPUT); //Laser power bit 5 set as output
  digitalWriteFast(LaserPowerBit5Pin, HIGH); //Laser power bit 5 turned off
  pinMode(LaserPowerBit6Pin, OUTPUT); //Laser power bit 6 set as output
  digitalWriteFast(LaserPowerBit6Pin, HIGH); //Laser power bit 6 turned off
  pinMode(LaserPowerBit7Pin, OUTPUT); //Laser power bit 7 set as output
  digitalWriteFast(LaserPowerBit7Pin, HIGH); //Laser power bit 7 turned off
  pinMode(InternalLEDPin, OUTPUT); //Internal LED set as ouput
  digitalWriteFast(InternalLEDPin, LOW); //Internal LED turned OFF
  pinMode(LaserMOSignalPin, OUTPUT); //Laser MO signal pin set as output
  digitalWriteFast(LaserMOSignalPin, HIGH); //Laser MO signal pin turned off
  pinMode(LaserONSignalPin, OUTPUT); //Laser on/off signal set as output
  digitalWriteFast(LaserONSignalPin, HIGH); //Laser on/off signal set to OFF
  pinMode(LaserPulsesPin, OUTPUT); //Laser pulses pin set as output
  digitalWriteFast(LaserPulsesPin, HIGH); //Laser pulses pin turned OFF
  pinMode(LaserGuidePin, OUTPUT); //Laser guide pin set as output
  digitalWriteFast(LaserGuidePin, LOW); //Laser guide turned ON - RED led pointer guide.
  pinMode(LaserTriggerPin, INPUT_PULLUP); //Laser trigger set as input:pullup

  analogWriteResolution(12); //12-bit DAC ouput on pin A14
  analogReadResolution(8); //8-bit input resolution for dial
  analogReference(DEFAULT); //Can be set to EXTERNAL);
  pinMode(DisplayOutputPin, OUTPUT); //Display pin set as output
  pinMode(DialInputPin, INPUT); //Dial input set as input

  //set up the switch debouncing options
  ActionBounce1.attach(ActionSwitchPin1); //attach the action switch 1 to the debouncer
  ActionBounce1.interval(5); //sets up the delay interval before it confirms a switch
  ActionBounce2.attach(ActionSwitchPin2); //attach the action switch 2 to the debouncer
  ActionBounce2.interval(5); //sets up the delay interval before it confirms a switch

  //Start the laser pulse generator with low level interrupts (time critical!)
  LaserPulseTimer.begin(LaserPulseGenerator, LaserPulsePeriod);

}

// -------- MAIN loop function ------------------------
void loop() {
  //If it is time: Send the debug output to the serial port
  if (serialMetro.check() == 1) {
    //Send serial output
    SerialOutputFunction(OperatingMode);
    //Toggle the LED
    if (digitalReadFast(InternalLEDPin) == HIGH)
      digitalWriteFast(InternalLEDPin, LOW); //Internal LED turned OFF
    else
      digitalWriteFast(InternalLEDPin, HIGH); //Internal LED turned ON
  }
  //Update the switch values (debouncer is not interrupt based!)
  ActionBounce1.update();
  ActionBounce2.update();

  /*Get the current operating mode:
   * Mode 0 = Laser ON, everything OK
   * Mode 1 = Laser IDLE, setting power level
   * Mode 2 = Laser OFF, setting pulse timer
   * Mode 3 = Temperature Alarm
   * Mode 4 = Reflection Alarm
   * Mode 5 = MO Alarm
   * ...
   * Mode 6-127 = RESERVED
   */
  OperatingMode = GetModeFunction(OperatingMode);
  ModeRunFunction(OperatingMode);
}

// -------- Pulse generator function --------------------
void LaserPulseGenerator(void) {
  //Toggles the LaserPulsesPin
  if (digitalReadFast(LaserPulsesPin) == HIGH)
    digitalWriteFast(LaserPulsesPin, LOW);
  else
    digitalWriteFast(LaserPulsesPin, HIGH);
  if (EnableLaserTrigger == 0)
    digitalWriteFast(LaserONSignalPin, HIGH); //Disable the ON signal
  else
    digitalWriteFast(LaserONSignalPin, LOW); //Activate the ON signal
}

// --------- Serial output function (debug only)----------
void SerialOutputFunction(uint8_t Mode) {
  Serial.print("Alarm: ");
  Serial.print(AlarmState);
  Serial.print(" | Laser Power: ");
  Serial.print(LaserPower);
  Serial.print(" | Mode: ");
  Serial.println(Mode);
}

// -------- Mode checking function ----------------------
uint8_t GetModeFunction(uint8_t CurrentMode) {
  //Check the alarm bits first:
  if (digitalReadFast(AlarmLSBPin) == HIGH) {
    //This is now certainly an alarm state. Read the other pin.
    if (digitalReadFast(AlarmMSBPin) == HIGH){
      // MO Alarm!!!
      return 5;
    }
    else {
      // Reflection Alarm!!
      return 4;
    }
  }
  else {
    //This still might be an alarm state. Read the other pin.
    if (digitalReadFast(AlarmMSBPin) == HIGH) {
      // Normal operation. Continue to check the status switch.
    }
    else {
      // Temperature Alarm!!
      return 3;
    }
  }

  //If the alarm is OK then check the switch state:
  uint8_t NewMode = 7; //Temporary variable to store the current switch state
  if (ActionBounce1.read() == LOW){
    //If this pin is low then the laser should be in MODE 0
    NewMode = 0;
  }
  else if (ActionBounce2.read() == LOW) {
    //If this pin is low then the laser should be in MODE 2
    NewMode = 2;
  }
  else {
    //If both pins are high: Laser should be in MODE 1
    NewMode = 1;
  }
  
  //If the mode was 1 or 2 and is now 0:
  if ((NewMode == 0) & ((CurrentMode == 1) | (CurrentMode == 2))) {
    //Run the startup sequence for the laser
    /* Sequence from the user manual:
     * The two signal are all TTL level. Before switch on the laser on/off 
     * signal, the MO signal must be switched on first, or the equipment can 
     * be damaged. The MO signalmust be 5ms earlier than laser on/off signal.
     */
     digitalWriteFast(LaserMOSignalPin, LOW); //Activate the MO signal
     delay(5); //Delay for 5 ms
     //EnableLaserTrigger = 1; //Enables the laser trigger input reading: =0 is disable and !=0 is enable
     digitalWriteFast(LaserONSignalPin, LOW); //Activate the ON signal
     
  }
  
  return NewMode;
}

//--------- Mode Runing Function -------------------
void ModeRunFunction(uint8_t Mode) {
  /*Get the current operating mode:
   * Mode 0 = Laser ON, everything OK
   * Mode 1 = Laser IDLE, setting power level
   * Mode 2 = Laser OFF, setting pulse timer
   * Mode 3 = Temperature Alarm
   * Mode 4 = Reflection Alarm
   * Mode 5 = MO Alarm
   * ...
   * Mode 6-127 = RESERVED
   */
  if (Mode == 0) {
    //Mode 0 = Laser ON, everything OK
    digitalWriteFast(LaserMOSignalPin, LOW); //Activate the MO signal
    //EnableLaserTrigger = 1; //Enables the laser trigger input reading: =0 is disable and !=0 is enable
    digitalWriteFast(LaserONSignalPin, LOW); //Activate the ON signal
    SetPowerLevel(); //Make sure the power level is correct (read the dial)
    digitalWriteFast(ExternalLEDPin, HIGH); //External LED turned ON
  }
  else if (Mode == 1) {
    //Mode 1 = Laser IDLE, setting power level
    EnableLaserTrigger = 0; //Disables the laser trigger input reading: =0 is disable and !=0 is enable
    digitalWriteFast(LaserONSignalPin, HIGH); //Disable the ON signal
    delay(1); //Delay for 1 ms : do not want to destroy the laser.
    digitalWriteFast(LaserMOSignalPin, HIGH); //Disable the MO signal 
    SetPowerLevel();
    digitalWriteFast(ExternalLEDPin, LOW); //External LED turned OFF
  }
  else if (Mode == 2) {
    //Mode 2 = Laser OFF, setting pulse timer
    EnableLaserTrigger = 0; //Disables the laser trigger input reading: =0 is disable and !=0 is enable
    digitalWriteFast(LaserONSignalPin, HIGH); //Disable the ON signal
    delay(1); //Delay for 1 ms : do not want to destroy the laser.
    digitalWriteFast(LaserMOSignalPin, HIGH); //Disable the MO signal 
    SetPulseRate();
    digitalWriteFast(ExternalLEDPin, LOW); //External LED turned OFF
  }
  else if (Mode == 3) {
    //Mode 3 = Temperature Alarm
    AlarmActions(Mode);
  }
  else if (Mode == 4) {
    //Mode 4 = Reflection Alarm
    AlarmActions(Mode);
  }
  else if (Mode == 5) {
    //Mode 5 = MO Alarm
    AlarmActions(Mode);
  }
  else {
    //Mode 6-127 = RESERVED
    AlarmActions(Mode);
  }
}

void SetPowerLevel(void) {
  //The power level is stored in the variable "LaserPower"
  //First: read the dial and store in the laser power variable
  LaserPower = analogRead(DialInputPin); //it already comes as an 8-bit value
  //Display this power level on the output display.
  //Output display shows power level in watts where 255d = 30watt = 300 mV
  DisplayOutput((float)(1.17641 * (float)LaserPower)); //1.17641 converts 8-bit power to mV
  //Convert the digital 8-bit value to pin outs:
  uint8_t TemporaryLaserPower = LaserPower;
  //Bit 7 (MSB)
  if (TemporaryLaserPower >= 128) {
    //Bit 7 is 128 or more in value
    digitalWriteFast(LaserPowerBit7Pin, LOW); //Laser power bit 7 turned ON
    TemporaryLaserPower = TemporaryLaserPower - 128;
  }
  else{
    digitalWriteFast(LaserPowerBit7Pin, HIGH); //Laser power bit 7 turned OFF
  }
  //Bit 6
  if (TemporaryLaserPower >= 64) {
    //Bit 6 is 64 or more in value
    digitalWriteFast(LaserPowerBit6Pin, LOW); //Laser power bit 6 turned ON
    TemporaryLaserPower = TemporaryLaserPower - 64;
  }
  else{
    digitalWriteFast(LaserPowerBit6Pin, HIGH); //Laser power bit 6 turned OFF
  }
  //Bit 5
  if (TemporaryLaserPower >= 32) {
    //Bit 5 is 32 or more in value
    digitalWriteFast(LaserPowerBit5Pin, LOW); //Laser power bit 5 turned ON
    TemporaryLaserPower = TemporaryLaserPower - 32;
  }
  else{
    digitalWriteFast(LaserPowerBit5Pin, HIGH); //Laser power bit 5 turned OFF
  }
  //Bit 4
  if (TemporaryLaserPower >= 16) {
    //Bit 4 is 16 or more in value
    digitalWriteFast(LaserPowerBit4Pin, LOW); //Laser power bit 4 turned ON
    TemporaryLaserPower = TemporaryLaserPower - 16;
  }
  else{
    digitalWriteFast(LaserPowerBit4Pin, HIGH); //Laser power bit 4 turned OFF
  }
  //Bit 3
  if (TemporaryLaserPower >= 8) {
    //Bit 3 is 8 or more in value
    digitalWriteFast(LaserPowerBit3Pin, LOW); //Laser power bit 3 turned ON
    TemporaryLaserPower = TemporaryLaserPower - 8;
  }
  else{
    digitalWriteFast(LaserPowerBit3Pin, HIGH); //Laser power bit 3 turned OFF
  }
  //Bit 2
  if (TemporaryLaserPower >= 4) {
    //Bit 2 is 4 or more in value
    digitalWriteFast(LaserPowerBit2Pin, LOW); //Laser power bit 2 turned ON
    TemporaryLaserPower = TemporaryLaserPower - 4;
  }
  else{
    digitalWriteFast(LaserPowerBit2Pin, HIGH); //Laser power bit 2 turned OFF
  }
  //Bit 1
  if (TemporaryLaserPower >= 2) {
    //Bit 1 is 2 or more in value
    digitalWriteFast(LaserPowerBit1Pin, LOW); //Laser power bit 1 turned ON
    TemporaryLaserPower = TemporaryLaserPower - 2;
  }
  else{
    digitalWriteFast(LaserPowerBit1Pin, HIGH); //Laser power bit 1 turned OFF
  }
  //Bit 0
  if (TemporaryLaserPower >= 1) {
    //Bit 0 is 1 or more in value
    digitalWriteFast(LaserPowerBit0Pin, LOW); //Laser power bit 0 turned ON
    TemporaryLaserPower = TemporaryLaserPower - 1;
  }
  else{
    digitalWriteFast(LaserPowerBit0Pin, HIGH); //Laser power bit 0 turned OFF
  }
}

void SetPulseRate(void) {
  //Read in the new value for the pulse frequency from the dial
  uint8_t PulseFrequency = analogRead(DialInputPin); //it already comes as an 8-bit value
  //Convert from ADC 8-bit to frequency
  //Remember that 255 = 65 kHz and 0 = 25 kHz
  float Frequency = (0.156863 * PulseFrequency) + 25; // F = 0.156862 * ADC + 25 .. calculated
  //Convert it from frequency to period / 2
  LaserPulsePeriod = 500 / Frequency;
  //Start the laser pulse generator with low level interrupts (time critical!)
  LaserPulseTimer.begin(LaserPulseGenerator, LaserPulsePeriod);
  //Put the frequency value on the display (30Hz = 300mV)
  DisplayOutput(Frequency * 10);
}

void AlarmActions(uint8_t Mode) {
  AlarmLEDFlashTimer.begin(AlarmLEDFlasher, 1000000);
  DisplayOutput(0); //sets the display output to ZERO
  EnableLaserTrigger = 0; //Disables the laser trigger input reading: =0 is disable and !=0 is enable
  digitalWriteFast(LaserONSignalPin, HIGH); //Disable the ON signal
  delay(1); //Delay for 1 ms : do not want to destroy the laser.
  digitalWriteFast(LaserMOSignalPin, HIGH); //Disable the MO signal
  //Serial.print("ALARM: ");
  if (Mode == 3) {
    //Serial.println("Temperature");
    AlarmState = "Temperature";
  }
  else if (Mode == 4) {
    //Serial.println("High Reflection");
    AlarmState = "High Reflection";
  }
  else if (Mode == 5) {
    //Serial.println("MO");
    AlarmState = "MO";
  }
  else {
    //Serial.println("Reserved Mode");
    AlarmState = "Reserved Mode";
  }
}

void DisplayOutput(float Value) {
  Value = Value * DisplayMultiplier; //converts mV reading to 14-bit ADC value
  analogWrite(DisplayOutputPin, (int)Value); //Set the output display to the value
}

void AlarmLEDFlasher(void) {
  if (digitalReadFast(ExternalLEDPin) == HIGH)
    digitalWriteFast(ExternalLEDPin, LOW); //External LED turned OFF
  else
    digitalWriteFast(ExternalLEDPin, HIGH); //External LED turned ON
}

