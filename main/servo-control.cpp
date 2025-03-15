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
// 10:33 PM 3/14/2025
#include "servo-control.h"
#include <Arduino.h>
servo_ctrl_t::servo_ctrl_t() {
  _servo_look = new Servo();
}

servo_ctrl_t::~servo_ctrl_t() {
  delete _servo_look;
}

void servo_ctrl_t::attach(int pin) {
  _servo_look->attach(pin);
}
void servo_ctrl_t::write(int value) {
  _servo_look->write(value);
}
void servo_ctrl_t::look_fornt() {
  // Look to the fornt and measure distance
  _servo_look->write(90);
}

void servo_ctrl_t::look_right() {
  // Look to the right and measure distance
  _servo_look->write(180);
}

void servo_ctrl_t::look_left() {
  // Look to the left and measure distance
  _servo_look->write(0);
}