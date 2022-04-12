#include <Arduino.h>
#include <PCA9685.h>
#include <Wire.h>

// You can import the full Pico SDK. Its installation is handled by PlatformIO :)
#include <pico.h>

// https://community.element14.com/products/arduino/arduino-projects/b/blog/posts/arduino-nano-rp2040-connect---mqtt

// This doesn't work
// It seems to keep using I2C0, which uses up the TX/RX pins and causes the LED on pin 25 to flash
// 1 1 1 1 2 2 2 2 1 1 1 1 2 2 2 2 1 1 1 1...
// slow, fast, slow fast...

// WHY?
// I've tried just about everything... I need an example of someone getting I2C working with this...
MbedI2C I2C(MBED_I2C1);

PCA9685 pwmController(I2C);
PCA9685_ServoEval pwmServo1;

__attribute__((unused)) void setup() {
    Serial.begin(115200);
    pwmController.resetDevices();
    pwmController.init();
    pwmController.setPWMFreqServo(); // 50Hz

    pwmController.setChannelPWM(0, pwmServo1.pwmForAngle(90));
}

__attribute__((unused)) void loop() {
    Serial.println("uwu");
}
