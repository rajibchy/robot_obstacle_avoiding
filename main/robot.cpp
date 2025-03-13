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
// 11:40 AM 3/10/2025
#include "robot.h"
#include <LowPower.h>

constexpr uint8_t _max_dist = 120;  ///< Maximum distance (cm) for obstacle detection
constexpr uint8_t _stop_dist = 40;  ///< Minimum distance (cm) to stop before an obstacle (10 inch)
constexpr uint8_t _closed_obstacle = 20;
constexpr float _time_out = 2 * (_max_dist + 10) / 100 / 340 * 1000000;  ///< Timeout for ultrasonic sensor measurement

constexpr uint8_t _motor_speed = 165;  ///< Motor speed (0 to 255)
constexpr uint8_t _motor_offset = 0;   ///< Motor offset to balance motor power
constexpr uint8_t _turn_speed = 50;    ///< Speed boost for turning


constexpr float _r1 = 100000.0;                ///< Resistor R1 in the voltage divider (100kΩ) resistors.
constexpr float _r2 = 10000.0;                 ///< Resistor R2 in the voltage divider (10kΩ) resistors.
constexpr uint8_t _battery_pin = A0;           ///< Analog pin to read battery voltage.
constexpr float _low_battery_threshold = 3.3;  ///< Voltage level to trigger low battery warning.


constexpr uint8_t ROBOT_IDLE = 0;        ///< Robot is idle, engine stopped, no movement.
constexpr uint8_t ROBOT_MOVE = 0;        ///< Robot moves forward without changing direction.
constexpr uint8_t ROBOT_TURN_LEFT = 1;   ///< Robot turns left.
constexpr uint8_t ROBOT_TURN_RIGHT = 2;  ///< Robot turns right.
constexpr uint8_t ROBOT_GO_BACK = 3;     ///< Robot go backward.

advanced_robot::advanced_robot() {

#ifdef USE_ADAFRUIT_V2
  _afms = new Adafruit_MotorShield();
  _right_front = _afms->getMotor(1);
  _left_front = _afms->getMotor(2);
  _left_back = _afms->getMotor(3);
  _right_back = _afms->getMotor(4);
#else
  _right_front = new AF_DCMotor(1, MOTOR12_8KHZ);
  _left_front = new AF_DCMotor(2, MOTOR12_8KHZ);
  _left_back = new AF_DCMotor(3, MOTOR34_8KHZ);
  _right_back = new AF_DCMotor(4, MOTOR34_8KHZ);
#endif  //!USE_ADAFRUIT_V2

  _rp = new robot_position();
  _sensor = new sensor_t(A0, A1, _time_out);
}

advanced_robot::~advanced_robot() {

#ifdef USE_ADAFRUIT_V2
  delete _afms;
  delete _rp;
#else
  delete _right_back;
  delete _right_front;
  delete _left_front;
  delete _left_back;
#endif  //!USE_ADAFRUIT_V2

  delete _sensor;
}

void advanced_robot::begin() {

#ifdef DEBUG_MODE
  Serial.begin(9600);  // Start serial communication
#endif                 //!DEBUG_MODE

#ifdef USE_ADAFRUIT_V2
  if (!_afms->begin()) {  // Check if the motor shield is detected
    while (1) {}          // Stop execution if motor shield is not found
  }
#endif  //!USE_ADAFRUIT_V2

  // Set motor speeds
  _right_back->setSpeed(_motor_speed);
  _right_front->setSpeed(_motor_speed);
  _left_front->setSpeed(_motor_speed);
  _left_back->setSpeed(_motor_speed);

  // Ensure motors are stopped
  stop_move();

  _sensor->attach(10);  // Attach servo to pin 10
}

bool advanced_robot::go_to_start_position() {
  // Check if the battery is low
  bool battery_is_low = battery_calculate();

  if (battery_is_low) {
    // If the battery is low, return to the start position
    return_to_start();

    // Put the system to sleep for 8 seconds to conserve power
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  // Sleep for 8 seconds to save power
    return true;
  }

  // Return false if the battery is not low
  return false;
}

void advanced_robot::loop() {

  if (go_to_start_position()) return;

  if (_is_first_time) {
    _is_first_time = false;
    _sensor->look_fornt();  // Look straight ahead
    delay(600);             // Wait before measuring distance
  } else {
    if (!_is_moving_forward) {
      delay(500);  // Wait before measuring distance
    } else {
      delay(200);  // Wait before measuring distance
    }
  }


  int distance = _sensor->get_distance();  // Get the current distance from the obstacle

  // Move forward if no obstacle is too close
  if (distance >= _stop_dist) {
    if (!_is_moving_forward) {
      move_forward();
    }
  }

  // Keep checking until the distance is less than _stop_dist
  while (true) {

    if (distance <= _stop_dist) break;

    if (go_to_start_position()) return;

    if (!_is_moving_forward) {
      move_forward();
    }

    delay(10);  // Wait before re-checking distance

    distance = _sensor->get_distance();
  }

  int duration_offset = 0;

  if (_is_moving_forward) {

    if (distance <= _closed_obstacle) {
      // close to obstacle
      stop_move();
      move_backward(800);
      try_move_left_or_right();
      return;
    }

    stop_move();
    move_backward(10);
    delay(400);
    duration_offset = -100;
    _is_moving_forward = false;
  }

  uint8_t turn_dir = check_direction(distance);  // Check left and right for the best direction to turn

  // Turn based on the best direction
  switch (turn_dir) {
    case ROBOT_TURN_LEFT: turn_left(600 + duration_offset); break;
    case ROBOT_GO_BACK: move_backward(700 + duration_offset); break;
    case ROBOT_TURN_RIGHT: turn_right(600 + duration_offset); break;
    case ROBOT_MOVE: move_forward(); break;
  }

  if (turn_dir == ROBOT_GO_BACK) {
    try_move_left_or_right();
  }
}

void advanced_robot::autonomous_navigation() {

  // Continuously check surroundings and navigate based on obstacles
  while (true) {

    // Stop navigation if the robot reaches the start position
    if (go_to_start_position()) break;

    int straight, right, left;

    // Measure distances in three directions (front, right, left)
    describe_distance(straight, right, left);

    // Decide the best movement direction based on obstacle detection
    uint8_t turn_dir = calculate_direction(straight, right, left);

    // Execute the movement based on the calculated direction
    switch (turn_dir) {
      case ROBOT_TURN_LEFT:
        turn_left(400);  // Turn left for 400ms
        break;

      case ROBOT_GO_BACK:
        move_backward(700);  // Move backward for 700ms
        break;

      case ROBOT_TURN_RIGHT:
        turn_right(400);  // Turn right for 400ms
        break;
      default:
        // Move forward with reduced sensor checks to save battery
        delay(2000);  // Delay to reduce sensor polling rate while moving
        break;
    }
  }
}

bool advanced_robot::battery_calculate() {

#ifdef USE_BATTERY_STATUS
  int raw_value = analogRead(_battery_pin);  // Read raw ADC value from battery pin

  // Convert raw ADC value to actual voltage using the voltage divider formula
  float voltage = (raw_value / 1023.0) * 5.0 * ((_r1 + _r2) / _r2);

  if (voltage < _low_battery_threshold) {
    // Low Battery! Charge now.
    return true;
  }
#endif  //!USE_BATTERY_STATUS

  return false;
}

void advanced_robot::return_to_start() {
  // Calculate the distance to the start position
  float distance_to_start = _rp->calculate_distance_to_start();

  // Loop until the robot reaches the start position
  while (distance_to_start > 1.0) {  // Threshold set to 1.0 to avoid floating-point errors
    // Adjust movement based on the robot's current position relative to the starting point
    if (_rp->get_x_position() > 0) {
      turn_left(400);  // Move left if the robot is too far to the right
    } else if (_rp->get_x_position() < 0) {
      turn_right(400);  // Move right if the robot is too far to the left
    } else if (_rp->get_y_position() > 0) {
      move_backward(400);  // Move backward if the robot is too far down
    } else if (_rp->get_y_position() < 0) {
      move_forward();  // Move forward if the robot is too far up
    }

    // Recalculate the distance to the start after each movement
    distance_to_start = _rp->calculate_distance_to_start();

    // Delay to allow for smoother movement
    delay(1000);
  }

  // Stop the robot once it reaches the start position
  stop_move();
}

void advanced_robot::move_forward() {
  _rp->move_forward();
  _is_moving_forward = true;
  _right_back->run(FORWARD);
  _left_back->run(FORWARD);
  _left_front->run(FORWARD);
  _right_front->run(FORWARD);
  accelerate();  // Smooth acceleration when moving forward
}

void advanced_robot::move_backward(int duration) {
  _rp->move_backward();
  _right_back->run(BACKWARD);   // Move the right back motor in the backward direction
  _right_front->run(BACKWARD);  // Move the right front motor in the backward direction
  _left_front->run(BACKWARD);   // Move the left front motor in the backward direction
  _left_back->run(BACKWARD);    // Move the left back motor in the backward direction
  accelerate();
  delay(duration);  // Move for the specified duration (in milliseconds)
  stop_move();      // Stop the motors to halt the movement
}

void advanced_robot::stop_move() {
  reset_motor_speed();         // Reset the motor speed after movement
  _right_back->run(RELEASE);   // Release the right back motor
  _right_front->run(RELEASE);  // Release the right front motor
  _left_front->run(RELEASE);   // Release the left front motor
  _left_back->run(RELEASE);    // Release the left back motor
}

void advanced_robot::turn_left(int duration) {
  _rp->move_left();
  _right_front->run(FORWARD);  // Move the right front motor forward
  _right_back->run(FORWARD);   // Move the right back motor forward

#ifndef USE_ONE_WHEEL_TURN
  _left_front->run(BACKWARD);  // Move the left front motor backward
  _left_back->run(BACKWARD);   // Move the left back motor backward
#endif
  set_turn_speed(_turn_speed, ROBOT_TURN_LEFT);  // Set the turning speed based on _turn_speed
  delay(duration);                               // Turn for the specified duration (in milliseconds)
  stop_move();                                   // Stop the motors to halt the turn
}

void advanced_robot::turn_right(int duration) {
  _rp->move_right();

#ifndef USE_ONE_WHEEL_TURN
  _right_front->run(BACKWARD);  // Move the right front motor backward
  _right_back->run(BACKWARD);   // Move the right back motor backward
#endif

  _left_front->run(FORWARD);                      // Move the left front motor forward
  _left_back->run(FORWARD);                       // Move the left back motor forward
  set_turn_speed(_turn_speed, ROBOT_TURN_RIGHT);  // Set the turning speed based on _turn_speed
  delay(duration);                                // Turn for the specified duration (in milliseconds)
  stop_move();                                    // Stop the motors to halt the turn
}

void advanced_robot::set_turn_speed(uint8_t turn_speed, uint8_t turn_type) {
  uint8_t speed = 0;
  uint8_t step = 20;         // Incremental step for smoother acceleration
  uint16_t delay_time = 20;  // Milliseconds per step
#ifdef USE_ONE_WHEEL_TURN

  uint8_t target_speed = _motor_speed + turn_speed;

  while (speed < target_speed) {

    speed = min(speed + step, target_speed);

    if (turn_type == ROBOT_TURN_RIGHT) {

      _left_front->setSpeed(speed);
      _left_back->setSpeed(speed);

    } else {

      _right_front->setSpeed(speed);
      _right_back->setSpeed(speed);
    }

    delay(delay_time);
  }

#else

  while (speed < _motor_speed) {

    speed = min(speed + step, _motor_speed);

    if (turn_type == ROBOT_TURN_RIGHT) {

      _left_front->setSpeed(speed + turn_speed);  // Set speed for left front motor with offset
      _left_back->setSpeed(speed + turn_speed);   // Set speed for left back motor with offset
      _right_front->setSpeed(speed);              // Set speed for right front motor
      _right_back->setSpeed(speed);               // Set speed for right back motor

    } else {

      _right_front->setSpeed(speed + turn_speed);  // Set speed for right front motor
      _right_back->setSpeed(speed + turn_speed);   // Set speed for right back motor
      _left_front->setSpeed(speed);                // Set speed for left front motor with offset
      _left_back->setSpeed(speed);                 // Set speed for left back motor with offset
    }

    delay(delay_time);
  }

#endif  //!USE_ONE_WHEEL_TURN
}

void advanced_robot::reset_motor_speed() {
  // _right_back->setSpeed(_motor_speed);   // Reset speed for right back motor
  // _right_front->setSpeed(_motor_speed);  // Reset speed for right front motor
  // _left_front->setSpeed(_motor_speed);   // Reset speed for left front motor with offset
  // _left_back->setSpeed(_motor_speed);    // Reset speed for left back motor with offset
  // Ensure all motors fully stop
  _right_back->setSpeed(0);
  _right_front->setSpeed(0);
  _left_front->setSpeed(0);
  _left_back->setSpeed(0);
}

void advanced_robot::accelerate() {

  uint8_t speed = 0;
  uint8_t step = 20;         // Incremental step for smoother acceleration
  uint16_t delay_time = 20;  // Milliseconds per step

  while (true) {

    if (speed >= _motor_speed) break;

    speed = min(speed + step, _motor_speed);

    _right_back->setSpeed(speed);   // Gradually increase speed for right back motor
    _right_front->setSpeed(speed);  // Gradually increase speed for right front motor
    _left_front->setSpeed(speed);   // Gradually increase speed for left front motor with offset
    _left_back->setSpeed(speed);    // Gradually increase speed for left back motor with offset
    delay(delay_time);              // Delay for smoother transition

  }

}

void advanced_robot::decelerate() {

  uint8_t speed = _motor_speed;
  uint8_t step = 20;         // Incremental step for smooth deceleration
  uint16_t delay_time = 20;  // Milliseconds per step

  while (speed > 0) {

    speed = max(0, speed - step);  // Prevent underflow

    _right_back->setSpeed(speed);   // Gradually decrease speed for right back motor
    _right_front->setSpeed(speed);  // Gradually decrease speed for right front motor
    _left_front->setSpeed(speed);   // Gradually decrease speed for left front motor
    _left_back->setSpeed(speed);    // Gradually decrease speed for left back motor

    if (speed == 0) return;

    delay(delay_time);  // Delay for smoother transition
    
  }

  // Ensure all motors fully stop
  reset_motor_speed();
}

void advanced_robot::describe_distance(int &straight, int &right, int &left) {

  // Look to the right and measure distance
  right = _sensor->get_right_distance(700);

  // Look to the left and measure distance
  left = _sensor->get_left_distance(800);

  // Reset servo position to the center (optional for better responsiveness)
  _sensor->look_fornt();

  if (straight == 0) {
    delay(400);

    straight = _sensor->get_distance();  // Measure forward distance
  }
}
void advanced_robot::try_move_left_or_right() {

  while (true) {
    // Look to the right and measure distance
    int right = _sensor->get_right_distance(700);

    // Look to the left and measure distance
    int left = _sensor->get_left_distance(800);

    // If both sides are blocked, reverse
    if (right <= _stop_dist && left <= _stop_dist) {

      if (right > _closed_obstacle || left > _closed_obstacle) {

        if (right > left) {
          turn_right(10);
        } else {
          turn_left(10);
        }
      }

      move_backward(500);
      continue;
    }

    int8_t move_left = 0;
    // If both right and left sides have large open space, prefer turning left
    if (right > left) {
      turn_right(400);
    } else {
      move_left = 1;
      turn_left(400);
    }

    while (true) {

      int straight = _sensor->get_fornt_distance(600);

      if (straight <= _stop_dist) {
        if (move_left < 0) {
          move_backward(500);
          break;
        }
        if (move_left == 1) {
          turn_left(100);
        } else {
          turn_right(100);
        }
        move_left = -1;
        continue;
      }
      return;
    }
  }
}
uint8_t advanced_robot::check_direction(int straight_distance) {

  int right, left;

  // Measure distances in three directions (front, right, left)
  describe_distance(straight_distance, right, left);

  return calculate_direction(straight_distance, right, left);
}

uint8_t advanced_robot::calculate_direction(const int &straight, const int &right, const int &left) const {

  // Move forward if there is enough space ahead
  if (straight > _stop_dist) {
    return ROBOT_MOVE;
  }

  // If both right and left sides have large open space, prefer turning towards the side with more space
  if (right >= _max_dist && left >= _max_dist) {

    if (right > left) {
      return ROBOT_TURN_RIGHT;
    }

    return ROBOT_TURN_LEFT;
  }

  // If both sides are blocked, reverse
  if (right <= _stop_dist && left <= _stop_dist) {
    return ROBOT_GO_BACK;
  }

  // Prefer turning towards the side with more space
  if (right >= left) {
    return ROBOT_TURN_LEFT;
  }

  if (right < left) {
    return ROBOT_TURN_RIGHT;
  }

  // Default case: Reverse as a fallback
  return ROBOT_GO_BACK;
}