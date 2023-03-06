/*  UPDATED FOOTSWITCH

  Current PIN use:
  - 2       FS 1
  - 3       FS 2

  - 7       INPUT L1    !TIP
  - 8       INPUT L2    !RING   (2ch [STEREO] FS ONLY)

  - A2      (analog) INPUT R1    !TIP  (5V OUTPUT)
  - A1      (analog) INPUT R2    !RING (ANALOGREAD here)

  - 16      2x FASTLEDS (WS2812B)

  Functions:

  Buttons
  - NoteOn/Off
  - Corresponding LED on/off on received NoteOn/Off

  INPUTS
  - Right input for external footswitch (soon: for expression pedal (potentiometer))
  - Left input for external footswitch

*/

//////////////////
////// LIBRARIES

// #include <pitchToFrequency.h>
// #include <pitchToNote.h>
// #include <frequencyToNote.h>
#include <MIDIUSB.h>

//////// FASTLED library
#include <FastLED.h>

////////////////////////////////////////
//////////////////// !! VARIABLES !! ///

// !! MIDI SETTINGS !! //
#define CHANNEL 10 // MIDI channel to be used (1-16). default: 10

byte midiCh = CHANNEL - 1;
byte note = 34;         // Lowest note to be used for buttons and LEDs (0-127)
byte expressionCC = 11; // Lowest MIDI CC to be used

///////////////
//// Buttons
const byte nButtons = 3;                    //*number of buttons (2 buttons + 2 encoder buttons + 1 digital crossfader)
const byte buttonPin[nButtons] = {3, 2, 7}; //* the number of the pushbutton pins in the desired order
int buttonCstate[nButtons] = {};            // stores the button current value
int buttonPstate[nButtons] = {};            // stores the button previous value

unsigned long lastDebounceTime[nButtons] = {}; // the last time the pin was toggled
const byte debounceDelay = 13;                 // the debounce time in ms; increase if the output flickers (default = 13)

/////////////////////////////////////////////
// Potentiometers

const byte powerPoti = A2; // for OUTPUT of 5v to TIP
const byte pinPoti = A1;
const int analogMax = 1020;
const int analogMin = 49;

const byte NPots = 1;           //* number of potis
byte potPin[NPots] = {pinPoti}; //* Pin where the potentiometer is
int potCState[NPots] = {};      // Current state of the pot
int potPState[NPots] = {};      // Previous state of the pot
int potVar = 0;                 // Difference between the current and previous state of the pot

byte midiCState[NPots] = {}; // Current state of the midi value
byte midiPState[NPots] = {}; // Previous state of the midi value

const int TIMEOUT = 300;         //* Amount of time the potentiometer will be read after it exceeds the varThreshold
const int varThreshold = 10;     //* Threshold for the potentiometer signal variation
boolean potMoving = true;        // If the potentiometer is moving
unsigned long PTime[NPots] = {}; // Previously stored time
unsigned long timer[NPots] = {}; // Stores the time that has elapsed since the timer was reset

/////////////////
// rxMidi

byte rxUSB = 0;      // usb header
byte rxChannel = 0;  // midi channel
byte rxType = 0;     // midi data type
byte rxPitch = 0;    // midi pitch
byte rxVelocity = 0; // midi velocity

/////////////////////////////////////////////////
// LEDS

// const int powerLed = 7; // powerLed pin (pushbuttonswitch LED)

// WS2812B LEDS for midi
#define NUM_LEDS 2  // Number of addressable LEDS
#define DATA_PIN 15 // LED PIN
#define LED_TYPE WS2812B
CRGB leds[NUM_LEDS];

////////////////////////////////////
/////////////////// !! SETUP !! ////

void setup()
{
  // Serial.begin(9600); // turns on serial readout for debugging
  // Serial.begin(31250);  // Set MIDI baud rate:

  // Power LED
  //   pinMode(powerLed, OUTPUT); // sets power led pin to output mode, if using one
  //   digitalWrite (powerLed, HIGH); // turns powerLed ON

  // buttons
  for (int i = 0; i < nButtons; i++)
  {
    pinMode(buttonPin[i], INPUT_PULLUP); // sets pullup resistor mode for button pins
  }

  // poti
  pinMode(powerPoti, OUTPUT);
  digitalWrite(powerPoti, HIGH); // turns second pin to 5V power source for expression pedal (TIP)

  // trying INPUT_PULLUP, instead of INPUT to prevent floating input when expression pedal is not connected
  pinMode(pinPoti, INPUT_PULLUP); // read expression pedal on this pin (RING)

  // addressable RGB LEDS
  FastLED.addLeds<LED_TYPE, DATA_PIN>(leds, NUM_LEDS);
  FastLED.clear();           // all addressable LEDs off during setup
  FastLED.setBrightness(15); // Brightness (0-255)
  FastLED.show();
}

/////////////////////////////////
//////////// !! LOOP !! /////////

void loop()
{
  buttons(); // 2xbutton input to midi out

  potentiometers();

  refreshMidi(); // get latest received midi data

  rxMidiLeds(); // received Midi Notes to LED control (2xRGB)

  // serialDebug();
}

///////////////////////////////////
/////////////// FUNCTIONS /////////

// BUTTONS
void buttons()
{

  for (int i = 0; i < nButtons; i++)
  {

    buttonCstate[i] = digitalRead(buttonPin[i]);

    if ((millis() - lastDebounceTime[i]) > debounceDelay)
    {

      if (buttonPstate[i] != buttonCstate[i])
      {
        lastDebounceTime[i] = millis();

        if (buttonCstate[i] == LOW)
        {

          noteOn(midiCh, note + i, 127); // channel, note, velocity
          MidiUSB.flush();
        }

        else
        {

          noteOn(midiCh, note + i, 0); // channel, note, velocity
          MidiUSB.flush();             // send midi buffer (after each note)
        }
        buttonPstate[i] = buttonCstate[i];
      }
    }
  }
}

/////////////////////////////////////////////
// POTENTIOMETERS
void potentiometers()
{

  for (int i = 0; i < NPots; i++)
  { // Loops through all the potentiometers

    potCState[i] = analogRead(potPin[i]); // Reads the pot and stores it in the potCState variable
    // Serial.println(potCState[i]);

    midiCState[i] = map(potCState[i], analogMin, analogMax, 0, 127); // Maps the reading of the potCState to a value usable in midi

    potVar = abs(potCState[i] - potPState[i]); // Calculates the absolute value between the difference between the current and previous state of the pot

    if (potVar > varThreshold)
    {                      // Opens the gate if the potentiometer variation is greater than the threshold
      PTime[i] = millis(); // Stores the previous time
    }

    timer[i] = millis() - PTime[i]; // Resets the timer 11000 - 11000 = 0ms

    if (timer[i] < TIMEOUT)
    { // If the timer is less than the maximum allowed time it means that the potentiometer is still moving
      potMoving = true;
    }
    else
    {
      potMoving = false;
    }

    if (potMoving == true)
    { // If the potentiometer is still moving, send the change control
      if (midiPState[i] != midiCState[i])
      {

        // use if using with ATmega328 (uno, mega, nano...)
        // MIDI.sendControlChange(expressionCC+i, midiCState[i], midiCh);

        // use if using with ATmega32U4 (micro, pro micro, leonardo...)
        controlChange(midiCh, expressionCC + i, midiCState[i]); // manda control change (channel, CC, value)
        MidiUSB.flush();

        // Serial.println(midiCState);
        potPState[i] = potCState[i]; // Stores the current reading of the potentiometer to compare with the next
        midiPState[i] = midiCState[i];
      }
    }
  }
}

/////////////////////////
///////////// midiLeds
void refreshMidi()
{
  // MidiUSB library commands
  midiEventPacket_t rx;
  rx = MidiUSB.read();

  // Bitshift to separate rx.byte1 (byte = 8bit) into first 4bit (TYPE), and second 4bit (Channel)
  rxChannel = rx.byte1 & B00001111; // get channel

  if (rxChannel == midiCh)
  {

    // Bitshift to separate rx.byte1 (byte = 8bit) into first 4bit (TYPE), and second 4bit (Channel)
    rxType = rx.byte1 >> 4;
    rxPitch = rx.byte2 - note; // Received pitch - midi-button-and-led first note setting
    rxVelocity = rx.byte3;     // Velocity variable - can be used, for example, for brightness
    rxUSB = rx.header;         // get usb header
  }
}

void rxMidiLeds()
{

  //////////////////
  // NOTEON received

  if (rxChannel == midiCh && rxType == 9)
  { // if receiving NoteON on correct channel

    // cycle through FASTLEDs
    for (int i = 0; i < NUM_LEDS; i++)
    { // cycle through all addressable LEDs

      if (rxType == 9 && rxPitch == i)
      { // if receiving noteON AND pitch matches LED
        int mapvelocity = map(rxVelocity, 0, 127, 0, 255);
        leds[i] = CHSV(mapvelocity, 255, 255); // control led by hue
        FastLED.show();
      }
    }
  }

  ///////////////////
  // NOTEOFF received

  if (rxChannel == midiCh && rxType == 8)
  { // if receiving NoteOFF on correct channel

    // cycle through FASTLEDs
    for (int i = 0; i < NUM_LEDS; i++)
    { // cycle through all addressable LEDs
      if (rxType == 8 && rxPitch == i)
      {                          // if receiving noteOFF AND pitch  matches LED
        leds[i] = CHSV(0, 0, 0); // turn LED off
        FastLED.show();
      }
    }
  }
}

void serialDebug()
{

  // DEBUG raw rxMidi data
  if (rxUSB != 0)
  {
    Serial.print("USB-Header: ");
    Serial.print(rxUSB, HEX);

    Serial.print(" / Type: ");
    switch (rxType)
    {
    case 8:
      Serial.print("NoteOFF");
      break;
    case 9:
      Serial.print("NoteON");
      break;
    case 11:
      Serial.print("CC");
      break;
    default:
      Serial.print("[");
      Serial.print(rxType);
      Serial.print("]");
      break;
    }

    Serial.print(" / Channel: ");
    Serial.print(rxChannel);

    Serial.print(" / Pitch: ");
    Serial.print(rxPitch);

    Serial.print(" / Velocity: ");
    Serial.println(rxVelocity);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////
// Arduino (pro)micro midi functions MIDIUSB Library for sending CCs and noteON and noteOFF
void noteOn(byte channel, byte pitch, byte velocity)
{
  midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOn);
}

void noteOff(byte channel, byte pitch, byte velocity)
{
  midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOff);
}

void controlChange(byte channel, byte control, byte value)
{
  midiEventPacket_t event = {0x0B, 0xB0 | channel, control, value};
  MidiUSB.sendMIDI(event);
}

/////////////////////////////////////////////////////////////////////////////////////
// MIDI messages via serial bus

// Sends a midi signal on channel 1 (0x90)
// cmd = message type and channel,
void midinoteOn(int cmd, int pitch, int velocity)
{
  Serial.write(cmd);
  Serial.write(pitch);
  Serial.write(velocity);
}
