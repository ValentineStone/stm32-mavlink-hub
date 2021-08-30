#include <Arduino.h>
#include "ardupilotmega/mavlink.h"
#include "mavlink_helpers.h"

HardwareSerial SerialA(PA3, PA2);
HardwareSerial SerialB(PA12, PA11);
HardwareSerial SerialC(PA10, PA9);

struct Led_t {
  uint32_t blinked = 0;
  uint32_t blink_duration = 20;
  bool blinking = false;
  bool blinking_off = false;
  void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
  }
  void loop() {
    if (blinking && millis() - blinked > blink_duration) {
      if (blinking_off) {
        blinking_off = false;
        blinking = false;
      } else {
        digitalWriteFast(PinName::PC_13, LOW);
        blinked = millis();
        blinking_off = true;
      }
    }
  }
  void blink() {
    if (!blinking) {
      digitalWriteFast(PinName::PC_13, HIGH);
      blinked = millis();
      blinking = true;
    }
  }
};

struct Input_t {
  inline static uint8_t send_buffer[280];
  mavlink_channel_t channel;
  HardwareSerial& input;
  HardwareSerial& output;
  Led_t& led;
  mavlink_status_t status;
  mavlink_message_t message;
  void loop () {
    while (input.available()) {
      if (mavlink_parse_char(channel, input.read(), &message, &status)) {
        uint16_t length = mavlink_msg_to_send_buffer(send_buffer, &message);
        output.write(send_buffer, length);
        led.blink();
      }
    }
  }
};

Led_t Led;
Input_t InputA { MAVLINK_COMM_0, SerialA, SerialC, Led };
Input_t InputB { MAVLINK_COMM_1, SerialB, SerialC, Led };

void setup() {
  SerialA.begin(57600);
  SerialB.begin(115200);
  SerialC.begin(57600);
  Led.setup();
}

void loop() {
  InputA.loop();
  InputB.loop();
  while (SerialC.available())
    SerialA.write(SerialC.read());
  Led.loop();
}