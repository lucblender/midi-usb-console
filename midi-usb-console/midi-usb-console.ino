// comment out those define if you don't want one of the midi transports
#define BLUETOOTH_MIDI
#define USB_MIDI
#define TRS_MIDI

#include <FlashStorage.h>

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
#define DIGITAL_CHANNELS_NUMBER 10
#define ANALOG_CHANNELS_NUMBER 8

// pin number for the midi activity led
#define LED_MIDI_PIN 13

#define SHIFT_SWITCH_PIN 12

// minimum time the led stays on after a control change
#define MIN_LED_TIME_MS 20

/**
 * @brief Structure to map a pin to a cc number
 */
struct pin_cc
{
  uint8_t pin;
  uint8_t cc;
  uint8_t cc_shift;
};

struct flash_config
{
  uint8_t analog_cc[ANALOG_CHANNELS_NUMBER];
  uint8_t analog_cc_shift[ANALOG_CHANNELS_NUMBER];
  uint8_t digital_cc[DIGITAL_CHANNELS_NUMBER];
};

FlashStorage(flashCCStore, struct flash_config);

// index for pin and cc in the pin_cc structure, both analog (potentiomenters, 0..127) and digital (switches, 0..1)
struct pin_cc analog_channels[ANALOG_CHANNELS_NUMBER] = {
    {A0, 16, 120}, {A1, 17, 121}, {A2, 18, 122}, {A3, 19, 123}, {A4, 20, 124}, {A5, 21, 125}, {A6, 22, 126}, {A7, 23, 127}};
struct pin_cc digital_channels[DIGITAL_CHANNELS_NUMBER] = {

    {9, 58}, {8, 59}, {7, 55}, {6, 71}, {5, 60}, {4, 42}, {3, 43}, {2, 44}, {11, 41}, {10, 45}};

// current values of the channels
uint8_t digital_value[DIGITAL_CHANNELS_NUMBER] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t analog_value[ANALOG_CHANNELS_NUMBER] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t analog_value_shifted[ANALOG_CHANNELS_NUMBER] = {0, 0, 0, 0, 0, 0, 0, 0};

// last time a control change was sent
unsigned long lastControlChange = 0;
// flag to indicate if the led is on because of a control change
bool controlChangeLed = false;

// smoothing factor for analog inputs
const float SMOOTHING = 0.05;

// status of bluetooth midi connection
bool bluetoothMidiConnected = false;

PinStatus shiftSwitchState = LOW;
PinStatus shiftSwitchStateOld = LOW;

bool analog_value_keepup[ANALOG_CHANNELS_NUMBER] = {false, false, false, false, false, false, false, false};

String inputString = "";     // a string to hold incoming data
bool stringComplete = false; // whether the string is complete

const String PREFIX = "lx-csl ";
const String DISCOVERY = "discovery";
const String CONFIG = "config";
const String CONFIG_OK = "OK";
const String CONFIG_ERR = "ERR";

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
  byte status = 0xB0 | ((channel) & 0x0F);
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

  pinMode(SHIFT_SWITCH_PIN, INPUT_PULLUP);

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
      if (tmpValue == 0)
        controlChange(0, digital_channels[i].cc, 0);
      else
        controlChange(0, digital_channels[i].cc, 127);
    }
  }
  shiftSwitchState = digitalRead(SHIFT_SWITCH_PIN);

  if (shiftSwitchState != shiftSwitchStateOld)
  {
    shiftSwitchStateOld = shiftSwitchState;
    for (int i = 0; i < ANALOG_CHANNELS_NUMBER; i++)
    {
      analog_value_keepup[i] = true;
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
    analog_value_shifted[i] = rawValue;
    controlChange(0, analog_channels[i].cc, rawValue);
    controlChange(0, analog_channels[i].cc_shift, rawValue);
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
    if (shiftSwitchState == LOW)
    {
      filtered = (float)analog_value[i] * (1.0 - SMOOTHING) + (float)rawValue * SMOOTHING;
    }
    else
    {
      filtered = (float)analog_value_shifted[i] * (1.0 - SMOOTHING) + (float)rawValue * SMOOTHING;
    }

    if (filtered > 64)
      newValue = ceil(filtered); // to be sure to reach 127, round to upper value
    else
      newValue = floor(filtered); // to be sure to reach 0, round to lower value

    uint8_t valueToCompoare;
    if (shiftSwitchState == LOW)
    {
      valueToCompoare = analog_value[i];
    }
    else
    {
      valueToCompoare = analog_value_shifted[i];
    }

    // only send if value changed (integer step)
    if (newValue != valueToCompoare)
    {
      if ((analog_value_keepup[i] == false) || (analog_value_keepup[i] == true && abs(rawValue - valueToCompoare) < 30))
      {
        analog_value_keepup[i] = false;
        if (shiftSwitchState == LOW)
        {
          analog_value[i] = newValue;
          controlChange(0, analog_channels[i].cc, newValue);
        }
        else
        {
          analog_value_shifted[i] = newValue;
          controlChange(0, analog_channels[i].cc_shift, newValue);
        }
      }
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
  inputString.reserve(200); // reserve some memory for efficiency

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

  initFlashCCStore();

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

  handleSerialMessage();
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

void handleSerialMessage()
{
  readSerial();

  if (stringComplete)
  {
    if (inputString.startsWith(PREFIX))
    {
      // process the line here...
      inputString.remove(0, PREFIX.length()); // remove the prefix
      inputString.trim();                     // remove any leading/trailing whitespace

      if (inputString.startsWith(DISCOVERY))
      {
        handleDiscovery();
      }
      if (inputString.startsWith(CONFIG))
      {
        handleConfig(inputString);
      }
    }

    // clear the string for the next line
    inputString = "";
    stringComplete = false;
  }
}

void readSerial()
{

  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    // Add it to the inputString
    inputString += inChar;

    // Check for end of line
    if (inChar == '\n')
    {
      stringComplete = true;
    }
  }
}

void handleDiscovery()
{
  String answer;

  answer += "a:";
  answer += String(ANALOG_CHANNELS_NUMBER);
  answer += ";d:";
  answer += String(DIGITAL_CHANNELS_NUMBER);
  answer += ";";
  for (int i = 0; i < ANALOG_CHANNELS_NUMBER; i++)
  {
    answer += "ac:";
    answer += String(analog_channels[i].cc);
    answer += ",";
    answer += String(analog_channels[i].cc_shift);
    answer += ";";
  }
  for (int i = 0; i < DIGITAL_CHANNELS_NUMBER; i++)
  {
    answer += "dc:";
    answer += String(digital_channels[i].cc);
    answer += ";";
  }

  Serial.print(PREFIX);
  Serial.print(DISCOVERY);
  Serial.print(" ");
  Serial.println(answer);
}

void handleConfig(String &inputString)
{
  // parse config message
  // example lx-csl config a:8;d:10;ac:16,120;ac:17,121;ac:18,122;ac:19,123;ac:20,124;ac:21,125;ac:22,126;ac:23,127;dc:58;dc:59;dc:55;dc:71;dc:60;dc:42;dc:43;dc:44;dc:41;dc:45
  inputString.remove(0, CONFIG.length()); // remove the config keyword
  inputString.trim();                     // remove any leading/trailing whitespace

  // Parse the message
  int analogChannels = 0;
  int digitalChannels = 0;
  int analogIndex = 0;
  int digitalIndex = 0;

  // Split by semicolon
  int startPos = 0;
  int endPos = 0;

  while (endPos != -1)
  {
    endPos = inputString.indexOf(';', startPos);
    String token;
    if (endPos == -1)
    {
      token = inputString.substring(startPos);
    }
    else
    {
      token = inputString.substring(startPos, endPos);
    }
    token.trim();

    if (token.startsWith("a:"))
    {
      analogChannels = token.substring(2).toInt();
    }
    else if (token.startsWith("d:"))
    {
      digitalChannels = token.substring(2).toInt();
    }
    else if (token.startsWith("ac:"))
    {
      if (analogIndex < ANALOG_CHANNELS_NUMBER)
      {
        String ccValues = token.substring(3);
        int commaPos = ccValues.indexOf(',');
        if (commaPos != -1)
        {
          int cc = ccValues.substring(0, commaPos).toInt();
          int ccShift = ccValues.substring(commaPos + 1).toInt();

          // Validate CC values are in valid MIDI range (0-127)
          if (cc >= 0 && cc <= 127 && ccShift >= 0 && ccShift <= 127)
          {
            analog_channels[analogIndex].cc = cc;
            analog_channels[analogIndex].cc_shift = ccShift;
          }
        }
        analogIndex++;
      }
    }
    else if (token.startsWith("dc:"))
    {
      if (digitalIndex < DIGITAL_CHANNELS_NUMBER)
      {
        int cc = token.substring(3).toInt();

        // Validate CC value is in valid MIDI range (0-127)
        if (cc >= 0 && cc <= 127)
        {
          digital_channels[digitalIndex].cc = cc;
        }
        digitalIndex++;
      }
    }

    startPos = endPos + 1;
  }

  // Only write to flash if the channel counts match
  if (analogChannels == ANALOG_CHANNELS_NUMBER && digitalChannels == DIGITAL_CHANNELS_NUMBER)
  {
    writeFlashCCStore();
    Serial.print(PREFIX);
    Serial.print(CONFIG);
    Serial.print(" ");
    Serial.println(CONFIG_OK);
  }
  else
  {
    Serial.print(PREFIX);
    Serial.print(CONFIG);
    Serial.print(" ");
    Serial.println(CONFIG_ERR);
  }
}

void initFlashCCStore()
{
  bool isFlashEmpty = true;
  flash_config storedConfig = flashCCStore.read();

  for (int i = 0; i < ANALOG_CHANNELS_NUMBER; i++)
  {
    if (storedConfig.analog_cc[i] != 0x00)
      isFlashEmpty = false;
    if (storedConfig.analog_cc_shift[i] != 0x00)
      isFlashEmpty = false;
  }
  for (int i = 0; i < DIGITAL_CHANNELS_NUMBER; i++)
  {
    if (storedConfig.digital_cc[i] != 0x00)
      isFlashEmpty = false;
  }

  if (isFlashEmpty == true)
  {
    writeFlashCCStore();
  }
  else
  {
    for (int i = 0; i < ANALOG_CHANNELS_NUMBER; i++)
    {
      analog_channels[i].cc = storedConfig.analog_cc[i];
      analog_channels[i].cc_shift = storedConfig.analog_cc_shift[i];
    }
    for (int i = 0; i < DIGITAL_CHANNELS_NUMBER; i++)
    {
      digital_channels[i].cc = storedConfig.digital_cc[i];
    }
  }
}

void writeFlashCCStore()
{
  flash_config storeConfig;

  for (int i = 0; i < ANALOG_CHANNELS_NUMBER; i++)
  {
    storeConfig.analog_cc[i] = analog_channels[i].cc;
    storeConfig.analog_cc_shift[i] = analog_channels[i].cc_shift;
  }
  for (int i = 0; i < DIGITAL_CHANNELS_NUMBER; i++)
  {
    storeConfig.digital_cc[i] = digital_channels[i].cc;
  }

  flashCCStore.write(storeConfig);
  initFlashCCStore();
}