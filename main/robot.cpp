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


constexpr uint8_t _trig = 2;                                             ///< Trigger pin for ultrasonic sensor
constexpr uint8_t _echo = 13;                                            ///< Echo pin for ultrasonic sensor
constexpr uint8_t _max_dist = 150;                                       ///< Maximum distance (cm) for obstacle detection
constexpr uint8_t _stop_dist = 50;                                       ///< Minimum distance (cm) to stop before an obstacle
constexpr float _time_out = 2 * (_max_dist + 10) / 100 / 340 * 1000000;  ///< Timeout for ultrasonic sensor measurement

constexpr uint8_t _motor_speed = 55;   ///< Motor speed (0 to 255)
constexpr uint8_t _motor_offset = 10;  ///< Motor offset to balance motor power
constexpr uint8_t _turn_speed = 50;    ///< Speed boost for turning


constexpr uint8_t _battery_pin = A0;           ///< Analog pin to read battery voltage.
constexpr float _r1 = 100000.0;                ///< Resistor R1 in the voltage divider (100kΩ) resistors.
constexpr float _r2 = 10000.0;                 ///< Resistor R2 in the voltage divider (10kΩ) resistors.
constexpr float _low_battery_threshold = 3.3;  ///< Voltage level to trigger low battery warning.


constexpr uint8_t ROBOT_IDLE = 0;          ///< Robot is idle, engine stopped, no movement.
constexpr uint8_t ROBOT_MOVE = 0;          ///< Robot moves forward without changing direction.
constexpr uint8_t ROBOT_TURN_LEFT = 1;     ///< Robot turns left.
constexpr uint8_t ROBOT_TURN_RIGHT = 2;    ///< Robot turns right.
constexpr uint8_t ROBOT_GO_BACK = 3;  ///< Robot go backward.

advanced_robot::advanced_robot() {
  _afms = new Adafruit_MotorShield();
  _rp = new robot_position();
  _servo_look = new Servo();
  _right_back = _afms->getMotor(1);
  _right_front = _afms->getMotor(2);
  _left_front = _afms->getMotor(3);
  _left_back = _afms->getMotor(4);
}

advanced_robot::~advanced_robot() {
  delete _afms;
  delete _rp;
  delete _servo_look;
}

void advanced_robot::begin() {
  Serial.begin(9600);     // Start serial communication
  if (!_afms->begin()) {  // Check if the motor shield is detected
    while (1) {}          // Stop execution if motor shield is not found
  }

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

  _servo_look->write(90);  // Look straight ahead
  delay(750);              // Wait before measuring distance


  int distance = get_distance();  // Get the current distance from the obstacle

  // Move forward if no obstacle is too close
  if (distance >= _stop_dist) {
    move_forward();
  }

  // Keep checking until the distance is less than _stop_dist
  while (distance >= _stop_dist) {

    if (go_to_start_position()) return;

    delay(250);  // Wait before re-checking distance
    distance = get_distance();
  }

  decelerate();  // Smooth stop when obstacle is too close
  stop_move();   // Ensure the robot stops after deceleration

  uint8_t turn_dir = check_direction();  // Check left and right for the best direction to turn

  // Turn based on the best direction
  switch (turn_dir) {
    case ROBOT_TURN_LEFT: turn_left(400); break;
    case ROBOT_GO_BACK: move_backward(700); break;
    case ROBOT_TURN_RIGHT: turn_right(400); break;
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
  int raw_value = analogRead(_battery_pin);  // Read raw ADC value from battery pin

  // Convert raw ADC value to actual voltage using the voltage divider formula
  float voltage = (raw_value / 1023.0) * 5.0 * ((_r1 + _r2) / _r2);

  if (voltage < _low_battery_threshold) {
    Serial.println("Low Battery! Charge now.");  // Warning message
    return true;
  }
  Serial.print("Battery Voltage: ");
  Serial.println(voltage);  // Print the battery voltage
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
  _right_back->run(FORWARD);    // Move the right back motor forward
  _right_front->run(FORWARD);   // Move the right front motor forward
  _left_front->run(BACKWARD);   // Move the left front motor backward
  _left_back->run(BACKWARD);    // Move the left back motor backward
  delay(duration);              // Turn for the specified duration (in milliseconds)
  reset_motor_speed();          // Reset the motor speed after the turn
  stop_move();                  // Stop the motors to halt the turn
}

void advanced_robot::turn_right(int duration) {
  _rp->move_right();
  set_turn_speed(_turn_speed);  // Set the turning speed based on _turn_speed
  _right_back->run(BACKWARD);   // Move the right back motor backward
  _right_front->run(BACKWARD);  // Move the right front motor backward
  _left_front->run(FORWARD);    // Move the left front motor forward
  _left_back->run(FORWARD);     // Move the left back motor forward
  delay(duration);              // Turn for the specified duration (in milliseconds)
  reset_motor_speed();          // Reset the motor speed after the turn
  stop_move();                  // Stop the motors to halt the turn
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
  for (uint8_t speed = 0; speed <= _motor_speed; speed++) {
    _right_back->setSpeed(speed);                  // Gradually increase speed for right back motor
    _right_front->setSpeed(speed);                 // Gradually increase speed for right front motor
    _left_front->setSpeed(speed + _motor_offset);  // Gradually increase speed for left front motor with offset
    _left_back->setSpeed(speed + _motor_offset);   // Gradually increase speed for left back motor with offset
    delay(20);                                     // Delay for smoother transition
  }
}

void advanced_robot::decelerate() {
  for (int8_t speed = _motor_speed; speed >= 0; speed--) {
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
  delay(300);
  left = get_distance();

  // Reset servo position to the center (optional for better responsiveness)
  _servo_look->write(90);
  delay(300);

  straight = get_distance();  // Measure forward distance
}

uint8_t advanced_robot::check_direction() {
  int distances[2] = { 0, 0 };  // Array to store the distances for both directions (right and left)

  // Move servo to the right to check the distance
  _servo_look->write(180);        // Look to the right
  delay(500);                     // Wait for the servo to reach the right position
  distances[0] = get_distance();  // Measure the distance to the right

  // Move servo to the left to check the distance
  _servo_look->write(0);          // Look to the left
  delay(1000);                    // Wait for the servo to reach the left position
  distances[1] = get_distance();  // Measure the distance to the left

  // Decision logic based on the measured distances
  if (distances[0] >= 200 && distances[1] >= 200) {
    // If both sides have large open space
    return ROBOT_TURN_LEFT;  // Turn left (0)
  }

  if (distances[0] <= _stop_dist && distances[1] <= _stop_dist) {
    // If both sides are blocked
    return ROBOT_GO_BACK;  // Reverse (1)
  }

  if (distances[0] >= distances[1]) {
    // If the right side has more space
    return ROBOT_TURN_LEFT;  // Turn left (0)
  }

  if (distances[0] < distances[1]) {
    // If the left side has more space
    return ROBOT_TURN_RIGHT;  // Turn right (2)
  }

  return ROBOT_GO_BACK;  // Default turn direction reverse
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