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
// 11:26 PM 3/10/2025


#ifndef _robot_sensor_h_
#define _robot_sensor_h_

#include <Servo.h>

/**
 * @class sensor_t
 * @brief Ultrasonic sensor interface with servo control.
 *
 * This class provides an interface for an ultrasonic distance sensor, allowing it to measure distances
 * to obstacles and adjust its direction using a servo motor.
 */
class sensor_t {
public:
  /**
   * @brief Constructs a sensor_t object.
   * 
   * Initializes the ultrasonic sensor with trigger and echo pins, as well as the timeout value.
   * Sets up the pins for proper input/output configurations.
   * 
   * @param trigger The GPIO pin used for the trigger signal.
   * @param echo The GPIO pin used to receive the echo signal.
   * @param time_out The maximum time (in microseconds) to wait for an echo response.
   */
  sensor_t(const uint8_t& trigger, const uint8_t& echo, const float& time_out);

  /**
   * @brief Destructor for sensor_t.
   * 
   * Cleans up dynamically allocated resources.
   */
  ~sensor_t();

private:
  uint8_t _echo;       ///< Echo pin for ultrasonic sensor.
  float _time_out;     ///< Timeout for ultrasonic sensor measurement.
  uint8_t _trigger;    ///< Trigger pin for ultrasonic sensor.
  Servo* _servo_look;  ///< Servo to control the looking direction.

public:
  /**
   * @brief Attaches the servo to a specified pin.
   * 
   * @param pin The GPIO pin where the servo is connected.
   */
  void attach(int pin);

  /**
   * @brief Measures the distance to the nearest object using an ultrasonic sensor.
   * 
   * @return int The measured distance in centimeters.
   */
  int get_distance();
  int get_target_distance(int servo_position, unsigned long ms);

  /**
   * @brief Measures the front distance after adjusting the sensor.
   * 
   * @param ms Delay time in milliseconds before taking the measurement.
   * @return int The measured distance in centimeters.
   */
  int get_fornt_distance(unsigned long ms);

  /**
   * @brief Measures the right-side distance after adjusting the sensor.
   * 
   * @param ms Delay time in milliseconds before taking the measurement.
   * @return int The measured distance in centimeters.
   */
  int get_right_distance(unsigned long ms);

  /**
   * @brief Measures the left-side distance after adjusting the sensor.
   * 
   * @param ms Delay time in milliseconds before taking the measurement.
   * @return int The measured distance in centimeters.
   */
  int get_left_distance(unsigned long ms);

  /**
   * @brief Adjusts the sensor to look forward.
   */
  void look_fornt();

  /**
   * @brief Adjusts the sensor to look right.
   */
  void look_right();

  /**
   * @brief Adjusts the sensor to look left.
   */
  void look_left();
};

#endif  //!_robot_sensor_h_