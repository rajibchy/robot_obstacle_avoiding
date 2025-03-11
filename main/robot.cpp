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


constexpr uint8_t _trig = A0;                                            ///< Trigger pin for ultrasonic sensor
constexpr uint8_t _echo = A1;                                            ///< Echo pin for ultrasonic sensor
constexpr uint8_t _max_dist = 150;                                       ///< Maximum distance (cm) for obstacle detection
constexpr uint8_t _stop_dist = 50;                                       ///< Minimum distance (cm) to stop before an obstacle
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
  _right_front = new AF_DCMotor(1, MOTOR12_1KHZ);
  _left_front = new AF_DCMotor(2, MOTOR12_1KHZ);
  _left_back = new AF_DCMotor(3, MOTOR34_1KHZ);
  _right_back = new AF_DCMotor(4, MOTOR34_1KHZ);
#endif  //!USE_ADAFRUIT_V2

  _rp = new robot_position();
  _servo_look = new Servo();
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
  delete _servo_look;
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
  _left_front->setSpeed(_motor_speed + _motor_offset);
  _left_back->setSpeed(_motor_speed + _motor_offset);

  // Ensure motors are stopped
  stop_move();

  _servo_look->attach(10);  // Attach servo to pin 10
  pinMode(_trig, OUTPUT);   // Set trigger pin as output
  pinMode(_echo, INPUT);    // Set echo pin as input
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
    _servo_look->write(90);  // Look straight ahead
    delay(600);              // Wait before measuring distance
  } else {
    delay(200);  // Wait before measuring distance
  }


  int distance = get_distance();  // Get the current distance from the obstacle

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

    delay(10);  // Wait before re-checking distance

    distance = get_distance();
  }

  if (_is_moving_forward) {

    if (distance <= 10) {
      // close to obstacle
      stop_move();
      move_backward(800);
    } else {
      stop_move();
      move_backward(10);
      _is_moving_forward = false;
    }
  }

  uint8_t turn_dir = check_direction(distance);  // Check left and right for the best direction to turn

  // Turn based on the best direction
  switch (turn_dir) {
    case ROBOT_TURN_LEFT: turn_left(600); break;
    case ROBOT_GO_BACK: move_backward(700); break;
    case ROBOT_TURN_RIGHT: turn_right(600); break;
    case ROBOT_MOVE: move_forward(); break;
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
  accelerate();  // Smooth acceleration when moving forward
  _right_back->run(FORWARD);
  _right_front->run(FORWARD);
  _left_front->run(FORWARD);
  _left_back->run(FORWARD);
}

void advanced_robot::move_backward(int duration) {
  _rp->move_backward();
  _right_back->run(BACKWARD);   // Move the right back motor in the backward direction
  _right_front->run(BACKWARD);  // Move the right front motor in the backward direction
  _left_front->run(BACKWARD);   // Move the left front motor in the backward direction
  _left_back->run(BACKWARD);    // Move the left back motor in the backward direction
  delay(duration);              // Move for the specified duration (in milliseconds)
  reset_motor_speed();          // Reset the motor speed after movement
  stop_move();                  // Stop the motors to halt the movement
}

void advanced_robot::stop_move() {
  _right_back->run(RELEASE);   // Release the right back motor
  _right_front->run(RELEASE);  // Release the right front motor
  _left_front->run(RELEASE);   // Release the left front motor
  _left_back->run(RELEASE);    // Release the left back motor
}

void advanced_robot::turn_left(int duration) {
  _rp->move_left();
  set_turn_speed(_turn_speed);  // Set the turning speed based on _turn_speed
  _right_front->run(FORWARD);   // Move the right front motor forward
  _right_back->run(FORWARD);    // Move the right back motor forward

  _left_front->run(BACKWARD);  // Move the left front motor backward
  _left_back->run(BACKWARD);   // Move the left back motor backward
  delay(duration);             // Turn for the specified duration (in milliseconds)
  reset_motor_speed();         // Reset the motor speed after the turn
  stop_move();                 // Stop the motors to halt the turn
}

void advanced_robot::turn_right(int duration) {
  _rp->move_right();
  set_turn_speed(_turn_speed);  // Set the turning speed based on _turn_speed
  _right_front->run(BACKWARD);  // Move the right front motor backward
  _right_back->run(BACKWARD);   // Move the right back motor backward

  _left_front->run(FORWARD);  // Move the left front motor forward
  _left_back->run(FORWARD);   // Move the left back motor forward
  delay(duration);            // Turn for the specified duration (in milliseconds)
  reset_motor_speed();        // Reset the motor speed after the turn
  stop_move();                // Stop the motors to halt the turn
}

void advanced_robot::set_turn_speed(uint8_t speed) {
  _right_back->setSpeed(_motor_speed + speed);                  // Set speed for right back motor
  _right_front->setSpeed(_motor_speed + speed);                 // Set speed for right front motor
  _left_front->setSpeed(_motor_speed + _motor_offset + speed);  // Set speed for left front motor with offset
  _left_back->setSpeed(_motor_speed + _motor_offset + speed);   // Set speed for left back motor with offset
}

void advanced_robot::reset_motor_speed() {
  _right_back->setSpeed(_motor_speed);                  // Reset speed for right back motor
  _right_front->setSpeed(_motor_speed);                 // Reset speed for right front motor
  _left_front->setSpeed(_motor_speed + _motor_offset);  // Reset speed for left front motor with offset
  _left_back->setSpeed(_motor_speed + _motor_offset);   // Reset speed for left back motor with offset
}

void advanced_robot::accelerate() {
  for (uint8_t speed = 0; speed <= _motor_speed; speed += 20) {
    if (speed > _motor_speed) speed = _motor_speed;  // Ensure speed does not exceed limit
    _right_back->setSpeed(speed);                    // Gradually increase speed for right back motor
    _right_front->setSpeed(speed);                   // Gradually increase speed for right front motor
    _left_front->setSpeed(speed + _motor_offset);    // Gradually increase speed for left front motor with offset
    _left_back->setSpeed(speed + _motor_offset);     // Gradually increase speed for left back motor with offset
    delay(20);                                       // Delay for smoother transition
  }
}

void advanced_robot::decelerate() {
  for (int8_t speed = _motor_speed; speed >= 0; speed -= 20) {
    if (speed < 0) speed = 0;                      // Ensure speed does not go below 0
    _right_back->setSpeed(speed);                  // Gradually decrease speed for right back motor
    _right_front->setSpeed(speed);                 // Gradually decrease speed for right front motor
    _left_front->setSpeed(speed + _motor_offset);  // Gradually decrease speed for left front motor with offset
    _left_back->setSpeed(speed + _motor_offset);   // Gradually decrease speed for left back motor with offset
    delay(20);                                     // Delay for smoother transition
  }
}

int advanced_robot::get_distance() {
  unsigned long pulse_time;  // Variable to store the time it takes for the ultrasonic pulse to return

  // Trigger the ultrasonic sensor to send a pulse
  digitalWrite(_trig, HIGH);  // Set trigger pin high to start sending the pulse
  delayMicroseconds(10);      // Wait for 10 microseconds to ensure the pulse is sent properly
  digitalWrite(_trig, LOW);   // Set trigger pin low to stop sending the pulse

  // Measure the time it takes for the echo pin to go high (i.e., the time for the pulse to return)
  pulse_time = pulseIn(_echo, HIGH, _time_out);  // Read the pulse width from the echo pin

  // Calculate and return the distance based on the pulse time
  return pulse_time * 340 / 2 / 10000;  // Distance in cm (speed of sound is 340 m/s, divide by 2 for round trip, and by 10000 to convert to cm)
}

void advanced_robot::describe_distance(int &straight, int &right, int &left) {

  // Look to the right and measure distance
  _servo_look->write(180);
  delay(300);
  right = get_distance();

  // Look to the left and measure distance
  _servo_look->write(0);
  delay(600);
  left = get_distance();

  // Reset servo position to the center (optional for better responsiveness)
  _servo_look->write(90);

  if (straight == 0) {
    delay(400);

    straight = get_distance();  // Measure forward distance
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
  if (straight >= _stop_dist) {
    return ROBOT_MOVE;
  }

  // If both right and left sides have large open space, prefer turning left
  if (right >= 200 && left >= 200) {
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