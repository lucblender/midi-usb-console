/*
 * MIDIUSB_test.ino
 *
 * Created: 4/6/2015 10:47:08 AM
 * Author: gurbrinder grewal
 * Modified by Arduino LLC (2015)
 */

#include "MIDIUSB.h"

#include <BLEMIDI_Transport.h>
static const char *const SERVICE_UUID = "03b80e5a-ede8-4b33-a751-6ce34ec4c700";
static const char *const CHARACTERISTIC_UUID = "7772e5db-3868-4112-a1a9-f2669d106bf3";
#include <hardware/BLEMIDI_ArduinoBLE.h>


BLEMIDI_CREATE_INSTANCE("ble-midi-console", MIDI)

#define PIN_INDEX 0
#define CC_CHANNEL_INDEX 1

#define DIGITAL_CHANNELS_NUMBER 11
#define ANALOG_CHANNELS_NUMBER 8

#define LED_MIDI_PIN 13

#define MIN_LED_TIME_MS 20

uint8_t analog_channels[ANALOG_CHANNELS_NUMBER][2] = { { A0, 10 }, { A1, 11 }, { A2, 12 }, { A3, 13 }, { A4, 14 }, { A5, 15 }, { A6, 16 }, { A7, 17 } };
uint8_t digital_channels[DIGITAL_CHANNELS_NUMBER][2] = { { 9, 100 }, { 8, 101 }, { 7, 102 }, { 6, 103 }, { 5, 104 }, { 4, 105 }, { 3, 106 }, { 2, 107 }, { 12, 108 }, { 11, 109 }, { 10, 110 } };

uint8_t digital_value[DIGITAL_CHANNELS_NUMBER] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t analog_value[ANALOG_CHANNELS_NUMBER] = { 0, 0, 0, 0, 0, 0, 0, 0 };

unsigned long lastControlChange = 0;
bool controlChangeLed = false;

const float SMOOTHING = 0.05;

bool bluetoothMidiConnected = false;


// First parameter is the event type (0x0B = control change).
// Second parameter is the event type, combined with the channel.
// Third parameter is the control number number (0-119).
// Fourth parameter is the control value (0-127).

void controlChange(byte channel, byte control, byte value) {
  midiEventPacket_t event = { 0x0B, 0xB0 | channel, control, value };
  MidiUSB.sendMIDI(event);
  sendCC(channel, control, value);

  if (bluetoothMidiConnected) {
    MIDI.sendControlChange(control, value, channel);

    digitalWrite(LED_MIDI_PIN, 0);
  } else {

    digitalWrite(LED_MIDI_PIN, 1);
  }

  lastControlChange = millis();
  controlChangeLed = 1;
}

void sendCC(byte channel, byte control, byte value) {
  byte status = 0xB0 | ((channel - 1) & 0x0F);
  Serial1.write(status);
  Serial1.write(control);
  Serial1.write(value);
}

void initDigitals() {
  uint8_t tmpValue;
  for (int i = 0; i < DIGITAL_CHANNELS_NUMBER; i++) {
    pinMode(digital_channels[i][PIN_INDEX], INPUT_PULLUP);
    tmpValue = digitalRead(digital_channels[i][PIN_INDEX]);
    digital_value[i] = tmpValue;
    controlChange(0, digital_channels[i][CC_CHANNEL_INDEX], tmpValue);
  }

  pinMode(LED_MIDI_PIN, OUTPUT);
}

void readDigitals() {
  uint8_t tmpValue;
  for (int i = 0; i < DIGITAL_CHANNELS_NUMBER; i++) {
    tmpValue = digitalRead(digital_channels[i][PIN_INDEX]);

    if (tmpValue != digital_value[i]) {
      digital_value[i] = tmpValue;
      controlChange(0, digital_channels[i][CC_CHANNEL_INDEX], tmpValue);
    }
  }
}


void initAnalog() {
  uint16_t rawValue;
  float filtered;

  for (int i = 0; i < ANALOG_CHANNELS_NUMBER; i++) {
    // read 10-bit value, reduce to 7 bits
    rawValue = analogRead(analog_channels[i][PIN_INDEX]) >> 3;


    analog_value[i] = rawValue;
    controlChange(0, analog_channels[i][CC_CHANNEL_INDEX], rawValue);
  }
}

void readAnalogs() {
  uint16_t rawValue;
  float filtered;

  for (int i = 0; i < ANALOG_CHANNELS_NUMBER; i++) {
    // read 10-bit value, reduce to 7 bits
    rawValue = analogRead(analog_channels[i][PIN_INDEX]) >> 3;


    // exponential smoothing
    filtered = (float)analog_value[i] * (1.0 - SMOOTHING) + (float)rawValue * SMOOTHING;
    uint16_t newValue;

    if (filtered > 64)
      newValue = ceil(filtered);  // to be sure to reach 127
    else
      newValue = floor(filtered);  // to be sure to reach 0

    // only send if value changed (integer step)
    if (newValue != analog_value[i]) {
      analog_value[i] = newValue;
      controlChange(0, analog_channels[i][CC_CHANNEL_INDEX], newValue);
    }
  }
}


void setup() {
  Serial.begin(115200);

  BLEMIDI.setHandleConnected([]() {
    digitalWrite(LED_MIDI_PIN, 1);
    bluetoothMidiConnected = true;
  });

  BLEMIDI.setHandleDisconnected([]() {
    digitalWrite(LED_MIDI_PIN, 0);
    bluetoothMidiConnected = false;
  });

  MIDI.begin();
  delay(2000);
  Serial1.begin(31250);
  initDigitals();
  initAnalog();
}



void loop() {

  if (MIDI.read()) {
    Serial.println(MIDI.getType());
  }

  readDigitals();
  readAnalogs();

  if (millis() - lastControlChange > MIN_LED_TIME_MS && controlChangeLed == 1) {


    if (bluetoothMidiConnected)
      digitalWrite(LED_MIDI_PIN, 1);
    else
      digitalWrite(LED_MIDI_PIN, 0);
    controlChangeLed = 0;
  }
}