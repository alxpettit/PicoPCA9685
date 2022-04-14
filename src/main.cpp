#include "PCA9685.h"

PCA9685 pwmController;         // Library using B010101 (A5-A0) i2c address, and default Wire @400kHz

void setup() {
    Serial.begin(115200);               // Begin Serial and Wire interfaces
    // If it's allowed to default to default, it will disrupt serial comm I think
    //Wire.setSCL(14u);
    //Wire.setSDA(15u);
    Wire.begin();
    pinMode(PIN_LED, OUTPUT);
    pwmController.resetDevices();       // Resets all PCA9685 devices on i2c line

    pwmController.init();               // Initializes module using default totem-pole driver mode, and default phase balancer

    pwmController.setPWMFrequency(500); // Set PWM freq to 500Hz (default is 200Hz, supports 24Hz to 1526Hz)

    randomSeed(analogRead(0));          // Use white noise for our randomness
}

bool led_state = false;
void loop() {
    uint16_t pwms[12];
    for (unsigned short & pwm : pwms) {
        pwm = random(0, 4096);
    }
    pwmController.setChannelsPWM(0, 12, pwms);
    Serial.println(pwms[0]);
    Serial.println("UwU");
    digitalWrite(PIN_LED, led_state);
    led_state = !led_state;
    delay(100);

    // NOTE: Many chips use a BUFFER_LENGTH size of 32, and in that case writing 12
    // channels will take 2 i2c transactions because only 7 channels can fit in a single
    // i2c buffer transaction at a time. This may cause a slight offset flicker between
    // the first 7 and remaining 5 channels, but can be offset by experimenting with a
    // channel update mode of PCA9685_ChannelUpdateMode_AfterAck. This will make each
    // channel update immediately upon sending of the Ack signal after each PWM command
    // is executed rather than at the Stop signal at the end of the i2c transaction.
}



//#include <Arduino.h>
//#include <PCA9685.h>
//#include <Wire.h>
//
//// You can import the full Pico SDK. Its installation is handled by PlatformIO :)
//#include <pico.h>
//
//// https://community.element14.com/products/arduino/arduino-projects/b/blog/posts/arduino-nano-rp2040-connect---mqtt
//
//// This doesn't work
//// It seems to keep using I2C0, which uses up the TX/RX pins and causes the LED on pin 25 to flash
//// 1 1 1 1 2 2 2 2 1 1 1 1 2 2 2 2 1 1 1 1...
//// slow, fast, slow fast...
//
//// WHY?
//
//#define PIN_TEST PIN_LED
//
//PCA9685* pwmController;
//PCA9685_ServoEval* pwmServo1;
//
//__attribute__((unused)) void setup() {
//    pinMode(PIN_TEST, OUTPUT);
//    Wire.begin();
//    pwmServo1 = new PCA9685_ServoEval();
//    pwmController = new PCA9685(Wire);
//    Serial.begin(115200);
//    pwmController->resetDevices();
//    pwmController->init();
//    pwmController->setPWMFreqServo(); // 50Hz
//}
//
//bool status = false;
//__attribute__((unused)) void loop() {
//    digitalWrite(PIN_TEST, status);
//    status = !status;
//    for (int angle=0;angle<180;angle++) {
//        for (int i = 0; i <= 15; ++i) {
//            pwmController->setChannelPWM(i, pwmServo1->pwmForAngle(angle));
//        }
//        delay(1000);
//    }
//    delay(1000);
//    Serial.println("uwu");
//}
