/*  UPDATED FOOTSWITCH

  Current PIN use:
  - 2       FS 1
  - 3       FS 2

  - 7       INPUT L1    !TIP
  - 8       INPUT L2    !RING   (used with [STEREO] FS ONLY)

  - A2      INPUT R    !TIP  (used as 5V OUTPUT in HIGH mode)
  - A1      INPUT R    !RING (ANALOGREAD here)

  - 16      2x FASTLEDS (WS2812B)

  Buttons
  - NoteOn/Off
  - Corresponding LED on/off on received NoteOn/Off

  INPUTS
  - Right input for expression pedal
  - Left input for external footswitch

*/

#include <MIDIUSB.h>
#include <FastLED.h>

// !! MIDI SETTINGS !! //
const byte MIDI_BASE_NOTE = 34; // Lowest note to be used for buttons and LEDs (0-127)
const byte MIDI_EXPRESSION_CC = 11;
const byte MIDI_CHANNEL = 10 - 1; // def: channel 10

///////////////
//// Buttons
const byte BUTTON_NUM = 4;                         // number of buttons
const byte BUTTON_PINS[BUTTON_NUM] = {3, 2, 7, 8}; // the number of the pushbutton pins in the desired order
const byte BUTTON_DEBOUNCE_DELAY = 13;             // the debounce time in ms; increase if the output flickers (default = 13)

int buttonCurrentState[BUTTON_NUM] = {};               // stores the button current value
int buttonPrevState[BUTTON_NUM] = {};                  // stores the button previous value
unsigned long buttonPrevDebounceTime[BUTTON_NUM] = {}; // the last time the pin was toggled

/////////////////////////////////////////////
// Potentiometers

const byte POT_POWER_PIN = A2; // for OUTPUT of 5v to TIP
const byte POT_PIN = A1;
const int POT_ANALOG_MAX = 1020;
const int POT_ANALOG_MIN = 49;
const byte POT_NUM = 1;              //* number of potis
const int POT_TIMEOUT = 300;         //* Amount of time the potentiometer will be read after it exceeds the POT_CHANGE_THRESHOLD
const int POT_CHANGE_THRESHOLD = 10; //* Threshold for the potentiometer signal variation

byte potPins[POT_NUM] = {POT_PIN};       //* Pin where the potentiometer is
int potCurrentState[POT_NUM] = {};       // Current state of the pot
int potPrevState[POT_NUM] = {};          // Previous state of the pot
int potVar = 0;                          // Difference between the current and previous state of the pot
boolean potMoving = true;                // If the potentiometer is moving
unsigned long potPrevTime[POT_NUM] = {}; // Previously stored time
unsigned long potTime[POT_NUM] = {};     // Stores the time that has elapsed since the potTime was reset

byte potMidiCurrentState[POT_NUM] = {}; // Current state of the midi value
byte potMidiPrevState[POT_NUM] = {};    // Previous state of the midi value

/////////////////
// Received Midi

byte rxUSB = 0;      // usb header
byte rxChannel = 0;  // midi channel
byte rxType = 0;     // midi data type
byte rxPitch = 0;    // midi pitch
byte rxVelocity = 0; // midi velocity

/////////////////////////////////////////////////
// LEDS

// WS2812B LEDS for midi
#define LED_NUM 2       // Number of addressable LEDS
#define LED_DATA_PIN 15 // LED PIN
#define LED_TYPE WS2812B
CRGB leds[LED_NUM];

////////////////////////////////////
/////////////////// !! SETUP !! ////

void setup()
{
  // Serial.begin(9600); // turns on serial readout for debugging
  // Serial.begin(31250); // Set MIDI baud rate for DIN5 Midi output port

  // buttons
  for (int i = 0; i < BUTTON_NUM; i++)
  {
    pinMode(BUTTON_PINS[i], INPUT_PULLUP); // sets pullup resistor mode for button pins
  }

  // poti
  pinMode(POT_POWER_PIN, OUTPUT);
  digitalWrite(POT_POWER_PIN, HIGH); // turns second pin to 5V power source for expression pedal (TIP)

  // trying INPUT_PULLUP, instead of INPUT to prevent floating input when expression pedal is not connected
  pinMode(POT_PIN, INPUT_PULLUP); // read expression pedal on this pin (RING)

  // addressable RGB LEDS
  FastLED.addLeds<LED_TYPE, LED_DATA_PIN>(leds, LED_NUM);
  FastLED.clear();           // all addressable LEDs off during setup
  FastLED.setBrightness(15); // Brightness (0-255)
  FastLED.show();
}

/////////////////////////////////
//////////// !! LOOP !! /////////

void loop()
{
  buttons();

  potentiometers();

  rxMidiRefresh();

  rxMidiToLeds();

  // serialDebug();
}

///////////////////////////////////
/////////////// FUNCTIONS /////////

void buttons()
{

  for (int i = 0; i < BUTTON_NUM; i++)
  {

    buttonCurrentState[i] = digitalRead(BUTTON_PINS[i]);

    if ((millis() - buttonPrevDebounceTime[i]) > BUTTON_DEBOUNCE_DELAY)
    {

      if (buttonPrevState[i] != buttonCurrentState[i])
      {
        buttonPrevDebounceTime[i] = millis();

        if (buttonCurrentState[i] == LOW)
        {
          noteOn(MIDI_CHANNEL, MIDI_BASE_NOTE + i, 127); // channel, note, velocity
        }
        else
        {
          noteOff(MIDI_CHANNEL, MIDI_BASE_NOTE + i, 0); // channel, note, velocity
        }
        MidiUSB.flush();

        buttonPrevState[i] = buttonCurrentState[i];
      }
    }
  }
}

void potentiometers()
{

  for (int i = 0; i < POT_NUM; i++)
  { // Loops through all the potentiometers

    potCurrentState[i] = analogRead(potPins[i]); // Reads the pot and stores it in the potCurrentState variable
    // Serial.println(potCurrentState[i]);

    potMidiCurrentState[i] = map(potCurrentState[i], POT_ANALOG_MIN, POT_ANALOG_MAX, 0, 127); // Maps the reading of the potCurrentState to a value usable in midi

    potVar = abs(potCurrentState[i] - potPrevState[i]); // Calculates the absolute value between the difference between the current and previous state of the pot

    if (potVar > POT_CHANGE_THRESHOLD)
    {                            // Opens the gate if the potentiometer variation is greater than the threshold
      potPrevTime[i] = millis(); // Stores the previous time
    }

    potTime[i] = millis() - potPrevTime[i]; // Resets the potTime 11000 - 11000 = 0ms

    if (potTime[i] < POT_TIMEOUT)
    { // If the potTime is less than the maximum allowed time it means that the potentiometer is still moving
      potMoving = true;
    }
    else
    {
      potMoving = false;
    }

    if (potMoving == true)
    { // If the potentiometer is still moving, send the change control
      if (potMidiPrevState[i] != potMidiCurrentState[i])
      {
        controlChange(MIDI_CHANNEL, MIDI_EXPRESSION_CC + i, potMidiCurrentState[i]); // manda control change (channel, CC, value)
        MidiUSB.flush();

        // Serial.println(potMidiCurrentState);
        potPrevState[i] = potCurrentState[i]; // Stores the current reading of the potentiometer to compare with the next
        potMidiPrevState[i] = potMidiCurrentState[i];
      }
    }
  }
}

void rxMidiRefresh()
{
  midiEventPacket_t rx; // from MidiUSB library
  rx = MidiUSB.read();

  // Bitshift to separate rx.byte1 (byte = 8bit) into first 4bit (TYPE), and second 4bit (Channel)
  rxChannel = rx.byte1 & B00001111; // get channel

  if (rxChannel == MIDI_CHANNEL)
  {

    // Bitshift to separate rx.byte1 (byte = 8bit) into first 4bit (TYPE), and second 4bit (Channel)
    rxType = rx.byte1 >> 4;
    rxPitch = rx.byte2 - MIDI_BASE_NOTE; // Received pitch - midi-button-and-led first MIDI_BASE_NOTE setting
    rxVelocity = rx.byte3;               // Velocity variable - can be used, for example, for brightness
    rxUSB = rx.header;                   // get usb header
  }
}

void rxMidiToLeds()
{
  if (rxChannel != MIDI_CHANNEL)
  {
    return;
  }

  // NOTE ON
  if (rxType == 9)
  { // if receiving NoteON on correct channel

    // cycle through FASTLEDs
    for (int i = 0; i < LED_NUM; i++)
    { // cycle through all addressable LEDs

      if (rxType == 9 && rxPitch == i)
      { // if receiving noteON AND pitch matches LED
        int mapvelocity = map(rxVelocity, 0, 127, 0, 255);
        leds[i] = CHSV(mapvelocity, 255, 255); // control led by hue
        FastLED.show();
      }
    }
  }
  // NOTE OFF
  else if (rxType == 8)
  { // if receiving NoteOFF on correct channel

    // cycle through FASTLEDs
    for (int i = 0; i < LED_NUM; i++)
    { // cycle through all addressable LEDs
      if (rxType == 8 && rxPitch == i)
      {                          // if receiving noteOFF AND pitch  matches LED
        leds[i] = CHSV(0, 0, 0); // turn LED off
        FastLED.show();
      }
    }
  }
}

// DEBUG raw rxMidi data
void serialDebug()
{

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
void serialMidiCommand(int cmd, int pitch, int velocity)
{
  Serial.write(cmd);
  Serial.write(pitch);
  Serial.write(velocity);
}
