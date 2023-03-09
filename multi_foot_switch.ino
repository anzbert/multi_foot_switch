/*  UPDATED FOOTSWITCH

  Current PIN use:
  - 2       FS 1
  - 3       FS 2

  - 7       INPUT L1    !TIP
  - 8       INPUT L2    !RING   (used with [STEREO] FS ONLY)

  - A2      INPUT R    !TIP  (used as 5V OUTPUT in HIGH mode)
  - A1      INPUT R    !RING (ANALOGREAD here)

  - 16      2x FASTLEDS (WS2812B)

  - 1 (TX)  DIN5 Midi Out port through Serial1 Hardware Pin
  - 5       DIN5_POWER_PIN (used as 5V OUTPUT in HIGH mode)

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

const byte DIN5_POWER_PIN = 5;

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
const int POT_CHANGE_THRESHOLD = 10; //* Threshold for the potentiometer signal variation
const int POT_TIMEOUT = 300;         //* Amount of time the potentiometer will be read after it exceeds the POT_CHANGE_THRESHOLD

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

byte rxUsbHeader = 0;
byte rxChannel = 0;
byte rxMidiMessageType = 0;
byte rxPitch = 0;
byte rxVelocity = 0;

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
  Serial1.begin(31250); // Set DIN5 out port MIDI baud rate on hardware pin (Serial1)

  for (int i = 0; i < BUTTON_NUM; i++)
  {
    pinMode(BUTTON_PINS[i], INPUT_PULLUP); // sets pullup resistor mode for button pins
  }

  pinMode(POT_PIN, INPUT_PULLUP); // INPUT_PULLUP, instead of INPUT to prevent floating input when expression pedal is not connected (RING)

  pinMode(POT_POWER_PIN, OUTPUT);
  digitalWrite(POT_POWER_PIN, HIGH); // as 5V power source for expression pedal (TIP)

  pinMode(DIN5_POWER_PIN, OUTPUT);
  digitalWrite(DIN5_POWER_PIN, HIGH); // as 5V power source for DIN5 Midi

  FastLED.addLeds<LED_TYPE, LED_DATA_PIN>(leds, LED_NUM);
  FastLED.clear();           // all LEDs off during setup
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

    if ((millis() - buttonPrevDebounceTime[i]) > BUTTON_DEBOUNCE_DELAY && buttonPrevState[i] != buttonCurrentState[i])
    {
      buttonPrevDebounceTime[i] = millis();

      if (buttonCurrentState[i] == LOW)
      {
        noteOn(MIDI_CHANNEL, MIDI_BASE_NOTE + i, 127);
      }
      else
      {
        noteOff(MIDI_CHANNEL, MIDI_BASE_NOTE + i, 0);
      }

      MidiUSB.flush();

      buttonPrevState[i] = buttonCurrentState[i];
    }
  }
}

void potentiometers()
{

  for (int i = 0; i < POT_NUM; i++)
  {
    potCurrentState[i] = analogRead(potPins[i]);
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
        controlChange(MIDI_CHANNEL, MIDI_EXPRESSION_CC + i, potMidiCurrentState[i]);
        MidiUSB.flush();

        // Serial.println(potMidiCurrentState);
        potPrevState[i] = potCurrentState[i];
        potMidiPrevState[i] = potMidiCurrentState[i];
      }
    }
  }
}

void rxMidiRefresh()
{
  midiEventPacket_t rx = MidiUSB.read(); // from MidiUSB library

  // Bitshift to separate rx.byte1 (byte = 8bit) into first 4bit (TYPE), and second 4bit (Channel)
  rxChannel = rx.byte1 & B00001111; // get channel

  if (rxChannel == MIDI_CHANNEL)
  {

    // Bitshift to separate rx.byte1 (byte = 8bit) into first 4bit (TYPE), and second 4bit (Channel)
    rxMidiMessageType = rx.byte1 >> 4;
    rxPitch = rx.byte2 - MIDI_BASE_NOTE; // Received pitch - midi-button-and-led first MIDI_BASE_NOTE setting
    rxVelocity = rx.byte3;               // Velocity variable - can be used, for example, for brightness
    rxUsbHeader = rx.header;             // get usb header
  }
}

void rxMidiToLeds()
{
  // Reject incorrect channel messages
  if (rxChannel != MIDI_CHANNEL)
  {
    return;
  }

  // NOTE ON
  if (rxMidiMessageType == 9)
  {
    for (int i = 0; i < LED_NUM; i++)
    {
      // if receiving noteON AND pitch matches LED
      if (rxMidiMessageType == 9 && rxPitch == i)
      {
        leds[i] = CHSV(map(rxVelocity, 0, 127, 0, 255), 255, 255); // control led on by hue
        FastLED.show();
      }
    }
  }
  // NOTE OFF
  else if (rxMidiMessageType == 8)
  {
    for (int i = 0; i < LED_NUM; i++)
    {
      if (rxMidiMessageType == 8 && rxPitch == i)
      {
        leds[i] = CHSV(0, 0, 0); // turn LED off
        FastLED.show();
      }
    }
  }
}

// DEBUG raw rxMidi data
void serialDebug()
{

  if (rxUsbHeader != 0)
  {
    Serial.print("USB-Header: ");
    Serial.print(rxUsbHeader, HEX);

    Serial.print(" / Type: ");
    switch (rxMidiMessageType)
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
      Serial.print(rxMidiMessageType);
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
// MIDI Send Commands

void noteOn(byte channel, byte pitch, byte velocity)
{
  midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOn);
  serialMidiCommand(0x90 | channel, pitch, velocity);
}

void noteOff(byte channel, byte pitch, byte velocity)
{
  midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOff);
  serialMidiCommand(0x80 | channel, pitch, velocity);
}

void controlChange(byte channel, byte control, byte value)
{
  midiEventPacket_t event = {0x0B, 0xB0 | channel, control, value};
  MidiUSB.sendMIDI(event);
  serialMidiCommand(0xB0 | channel, control, value);
}

// Send MIDI commands via hardware serial pin1 (Tx)
void serialMidiCommand(int cmd, int byte2, int byte3)
{
  Serial1.write(cmd);
  Serial1.write(byte2);
  Serial1.write(byte3);
}
