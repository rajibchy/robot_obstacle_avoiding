/*
* MIT License
* 
* Copyright (c) 2025 Rajib Chowdhury
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
// 3:11 PM 3/8/2025
#include "robot-position.h"
constexpr float _move_distance = 10.0;  // Define distance moved in each step (e.g., 10 cm per move)

float robot_position::get_x_position() const {
  return _x_position;
}

float robot_position::get_y_position() const {
  return _y_position;
}

void robot_position::move_left() {
  _x_position -= _move_distance;
}

void robot_position::move_right() {
  _x_position += _move_distance;
}

void robot_position::move_forward() {
   _y_position += _move_distance;
}

void robot_position::move_backward() {
  _y_position -= _move_distance;
}

float robot_position::calculate_distance_to_start() {
  return sqrt(_x_position * _x_position + _y_position * _y_position);
}