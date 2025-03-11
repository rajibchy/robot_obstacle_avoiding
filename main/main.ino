/*
* MIT License
* 
* Copyright (c) 2025 Rajib Chowdhury (https://github.com/rajibchy/robot_obstacle_avoiding)
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

// Rajib chy
// 12:24 AM 3/8/2025
// Robot Obstacle Avoidance with Low Power Management (Arduino Uno)

#include "robot.h"
// #include <Wire.h>

// Create an instance of the robot
static advanced_robot robot;

void setup() {
// #ifndef USE_ADAFRUIT_V2
//   TCCR1B = (TCCR1B & 0b11111000) | 0x01;  // Set Timer1 to fast PWM (8 kHz)
// #endif //!USE_ADAFRUIT_V2
  robot.begin();
}

void loop() {
  robot.loop();
}


// void setup() {
//   Serial.begin(115200);
//   while (!Serial);
//   Serial.println("\nI2C Scanner");

//   Wire.begin();
// }

// void loop() {
//   Serial.println("Scanning...");

//   for (byte address = 1; address < 127; address++) {
//     Wire.beginTransmission(address);
//     if (Wire.endTransmission() == 0) {
//       Serial.print("I2C device found at 0x");
//       Serial.println(address, HEX);
//       delay(500);
//     }
//   }
//   Serial.println("Done");
//   delay(5000);
// }