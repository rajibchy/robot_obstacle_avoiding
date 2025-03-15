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
// 10:30 PM 3/14/2025


#ifndef _servo_ctrl_h_
#define _servo_ctrl_h_

#include <Servo.h>

/**
 * @class servo_ctrl_t
 * @brief A class to control a servo motor for adjusting sensor direction.
 * 
 * This class provides an interface to attach a servo motor to a GPIO pin
 * and control its position for looking in different directions.
 */
class servo_ctrl_t {
public:

  /**
   * @brief Constructs a new servo_ctrl_t object.
   * 
   * Initializes the servo control class and prepares it for attachment to a pin.
   */
  servo_ctrl_t();

  /**
   * @brief Destroys the servo_ctrl_t object.
   * 
   * Ensures proper cleanup of resources allocated to the servo.
   */
  ~servo_ctrl_t();

  /**
   * @brief Attaches the servo to a specified pin.
   * 
   * This function associates a servo with a GPIO pin, enabling control over its position.
   * 
   * @param pin The GPIO pin where the servo is connected.
   */
  void attach(int pin);

  /**
   * @brief Writes a value to the servo to set its position.
   * 
   * If the provided value is less than 200, it is treated as an angle (0° - 180°).
   * Otherwise, it is treated as a pulse width in microseconds (500 - 2500 µs).
   * 
   * @param value The desired servo position, either as an angle or pulse width.
   */
  void write(int value);

  /**
   * @brief Adjusts the sensor to look forward.
   * 
   * This function sets the servo to a predefined forward-facing position.
   */
  void look_fornt();

  /**
   * @brief Adjusts the sensor to look right.
   * 
   * Moves the servo to make the sensor face right.
   */
  void look_right();

  /**
   * @brief Adjusts the sensor to look left.
   * 
   * Moves the servo to make the sensor face left.
   */
  void look_left();

private:
  /**
   * @brief Pointer to the Servo object controlling the sensor direction.
   * 
   * This object handles the low-level operations for moving the servo motor.
   */
  Servo* _servo_look;
};

#endif  //!_servo_ctrl_h_