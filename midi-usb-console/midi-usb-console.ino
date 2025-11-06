// comment out those define if you don't want one of the midi transports
#define BLUETOOTH_MIDI
#define USB_MIDI
#define TRS_MIDI

#ifdef USB_MIDI
#include "MIDIUSB.h"
#endif

#ifdef BLUETOOTH_MIDI
#include <BLEMIDI_Transport.h>
static const char *const SERVICE_UUID = "03b80e5a-ede8-4b33-a751-6ce34ec4c700";
static const char *const CHARACTERISTIC_UUID = "7772e5db-3868-4112-a1a9-f2669d106bf3";
#include <hardware/BLEMIDI_ArduinoBLE.h>

BLEMIDI_CREATE_INSTANCE("ble-midi-console", MidiBluetooth)
#endif

// number of digital and analog channels
#define DIGITAL_CHANNELS_NUMBER 11
#define ANALOG_CHANNELS_NUMBER 8

// pin number for the midi activity led
#define LED_MIDI_PIN 13

// minimum time the led stays on after a control change
#define MIN_LED_TIME_MS 20

/**
 * @brief Structure to map a pin to a cc number
 */
struct pin_cc
{
  uint8_t pin;
  uint8_t cc;
};

// index for pin and cc in the pin_cc structure, both analog (potentiomenters, 0..127) and digital (switches, 0..1)
struct pin_cc analog_channels[ANALOG_CHANNELS_NUMBER] = {{A0, 10}, {A1, 11}, {A2, 12}, {A3, 13}, {A4, 14}, {A5, 15}, {A6, 16}, {A7, 17}};
struct pin_cc digital_channels[DIGITAL_CHANNELS_NUMBER] = {{9, 100}, {8, 101}, {7, 102}, {6, 103}, {5, 104}, {4, 105}, {3, 106}, {2, 107}, {12, 108}, {11, 109}, {10, 110}};

// current values of the channels
uint8_t digital_value[DIGITAL_CHANNELS_NUMBER] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t analog_value[ANALOG_CHANNELS_NUMBER] = {0, 0, 0, 0, 0, 0, 0, 0};

// last time a control change was sent
unsigned long lastControlChange = 0;
// flag to indicate if the led is on because of a control change
bool controlChangeLed = false;

// smoothing factor for analog inputs
const float SMOOTHING = 0.05;

// status of bluetooth midi connection
bool bluetoothMidiConnected = false;

/**
 * @brief Send a midi control change on all available transports
 *
 * @param channel midi channel (0..15)
 * @param control control number (0..127)
 * @param value control value (0..127)
 */
void controlChange(byte channel, byte control, byte value)
{

#ifdef USB_MIDI
  sendControlChangeUSB(channel, control, value);
#endif

#ifdef TRS_MIDI
  sendControlChangeTRS(channel, control, value);
#endif

  if (bluetoothMidiConnected) // when bluetooth midi is connected, activity led is inverted
  {

#ifdef BLUETOOTH_MIDI
    sendControlChangeBluetooth(channel, control, value);
#endif

    digitalWrite(LED_MIDI_PIN, 0);
  }
  else
  {

    digitalWrite(LED_MIDI_PIN, 1);
  }

  lastControlChange = millis();
  controlChangeLed = 1;
}

/**
 * @brief Send a midi control change over usb midi
 *
 * @param channel midi channel (0..15)
 * @param control control number (0..127)
 * @param value control value (0..127)
 */
void sendControlChangeUSB(byte channel, byte control, byte value)
{
  midiEventPacket_t event = {0x0B, 0xB0 | channel, control, value};
  MidiUSB.sendMIDI(event);
}

/**
 * @brief Send a midi control change over trs midi
 *
 * @param channel midi channel (0..15)
 * @param control control number (0..127)
 * @param value control value (0..127)
 */
void sendControlChangeTRS(byte channel, byte control, byte value)
{
  byte status = 0xB0 | ((channel - 1) & 0x0F);
  Serial1.write(status);
  Serial1.write(control);
  Serial1.write(value);
}

/**
 * @brief Send a midi control change over bluetooth midi
 *
 * @param channel midi channel (0..15)
 * @param control control number (0..127)
 * @param value control value (0..127)
 */
void sendControlChangeBluetooth(byte channel, byte control, byte value)
{
  MidiBluetooth.sendControlChange(control, value, channel);
}

/**
 * @brief Initialize digital inputs
 *
 */
void initDigitals()
{
  uint8_t tmpValue;
  for (int i = 0; i < DIGITAL_CHANNELS_NUMBER; i++)
  {
    pinMode(digital_channels[i].pin, INPUT_PULLUP);
    tmpValue = digitalRead(digital_channels[i].pin);
    digital_value[i] = tmpValue;
    controlChange(0, digital_channels[i].cc, tmpValue);
  }

  pinMode(LED_MIDI_PIN, OUTPUT);
}

/**
 * @brief Read digital inputs and send midi control changes if value changed
 *
 */
void readDigitals()
{
  uint8_t tmpValue;
  for (int i = 0; i < DIGITAL_CHANNELS_NUMBER; i++)
  {
    tmpValue = digitalRead(digital_channels[i].pin);

    if (tmpValue != digital_value[i])
    {
      digital_value[i] = tmpValue;
      controlChange(0, digital_channels[i].cc, tmpValue);
    }
  }
}

/**
 * @brief Initialize analog inputs
 *
 */
void initAnalog()
{
  uint16_t rawValue;
  float filtered;

  for (int i = 0; i < ANALOG_CHANNELS_NUMBER; i++)
  {
    // read 10-bit value, reduce to 7 bits (midi goes from 0 to 127 so 7 bits)
    rawValue = analogRead(analog_channels[i].pin) >> 3;

    analog_value[i] = rawValue;
    controlChange(0, analog_channels[i].cc, rawValue);
  }
}

/**
 * @brief Read analog inputs and send midi control changes if value changed
 *
 */
void readAnalogs()
{
  uint16_t rawValue;
  uint16_t newValue;

  float filtered;

  for (int i = 0; i < ANALOG_CHANNELS_NUMBER; i++)
  {
    // read 10-bit value, reduce to 7 bits (midi goes from 0 to 127 so 7 bits)
    rawValue = analogRead(analog_channels[i].pin) >> 3;

    // exponential smoothing
    filtered = (float)analog_value[i] * (1.0 - SMOOTHING) + (float)rawValue * SMOOTHING;

    if (filtered > 64)
      newValue = ceil(filtered); // to be sure to reach 127, round to upper value
    else
      newValue = floor(filtered); // to be sure to reach 0, round to lower value

    // only send if value changed (integer step)
    if (newValue != analog_value[i])
    {
      analog_value[i] = newValue;
      controlChange(0, analog_channels[i].cc, newValue);
    }
  }
}

/**
 * @brief Arduino setup function
 *
 */
void setup()
{
  Serial.begin(115200);

#ifdef BLUETOOTH_MIDI
  // bluetooth callbacks for connection status
  BLEMidiBluetooth.setHandleConnected([]()
                                      {
    digitalWrite(LED_MIDI_PIN, 1);
    bluetoothMidiConnected = true; });

  BLEMidiBluetooth.setHandleDisconnected([]()
                                         {
    digitalWrite(LED_MIDI_PIN, 0);
    bluetoothMidiConnected = false; });

  MidiBluetooth.begin();
#endif

  // wait for midi connection before init --> init send current value of all controls
  delay(2000);

#ifdef TRS_MIDI
  Serial1.begin(31250);
#endif

  initDigitals();
  initAnalog();
}

/**
 * @brief Arduino main loop
 *
 */
void loop()
{

#ifdef BLUETOOTH_MIDI
  MidiBluetooth.read();
#endif

  readDigitals();
  readAnalogs();

  // turn off midi activity led after a minimum time
  if (millis() - lastControlChange > MIN_LED_TIME_MS && controlChangeLed == 1)
  {
    // when bluetooth midi is connected, activity led is inverted
    if (bluetoothMidiConnected)
      digitalWrite(LED_MIDI_PIN, 1);
    else
      digitalWrite(LED_MIDI_PIN, 0);

    controlChangeLed = 0;
  }
}