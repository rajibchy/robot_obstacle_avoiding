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
// 1:20 PM 3/11/2025
#include "sensor.h"
#include <Arduino.h>
sensor_t::sensor_t(const uint8_t& trigger, const uint8_t& echo, const float& time_out) {

  _echo = echo;
  _trigger = trigger;
  _time_out = time_out;
  _servo_look = new Servo();

  pinMode(_trigger, OUTPUT);  // Set trigger pin as output
  pinMode(_echo, INPUT);   // Set echo pin as input
}
void sensor_t::attach(int pin) {
  _servo_look->attach(pin);
}
sensor_t::~sensor_t() {
  delete _servo_look;
}

int sensor_t::get_distance() {
  unsigned long pulse_time;  // Variable to store the time it takes for the ultrasonic pulse to return

  // Trigger the ultrasonic sensor to send a pulse
  digitalWrite(_trigger, HIGH);  // Set trigger pin high to start sending the pulse
  delayMicroseconds(10);         // Wait for 10 microseconds to ensure the pulse is sent properly
  digitalWrite(_trigger, LOW);   // Set trigger pin low to stop sending the pulse

  // Measure the time it takes for the echo pin to go high (i.e., the time for the pulse to return)
  pulse_time = pulseIn(_echo, HIGH, _time_out);  // Read the pulse width from the echo pin

  // Calculate and return the distance based on the pulse time
  return pulse_time * 340 / 2 / 10000;  // Distance in cm (speed of sound is 340 m/s, divide by 2 for round trip, and by 10000 to convert to cm)
}
int sensor_t::get_fornt_distance(unsigned long ms) {
  look_fornt();
  delay(ms);
  return get_distance();
}
int sensor_t::get_right_distance(unsigned long ms) {
  look_right();
  delay(ms);
  return get_distance();
}
int sensor_t::get_left_distance(unsigned long ms) {
  look_left();
  delay(ms);
  return get_distance();
}

void sensor_t::look_fornt() {
  // Look to the fornt and measure distance
  _servo_look->write(90);
}

void sensor_t::look_right() {
  // Look to the right and measure distance
  _servo_look->write(180);
}

void sensor_t::look_left() {
  // Look to the left and measure distance
  _servo_look->write(0);
}
