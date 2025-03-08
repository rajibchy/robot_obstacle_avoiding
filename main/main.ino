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
// 12:24 AM 3/8/2025
// Obstacle Avoiding Robot

#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include <LowPower.h>
#include "robot-position.h"

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


/**
 * Class to control the obstacle-avoiding robot with motor shield and ultrasonic sensor.
 */
class obstacle_avoiding_robot {
private:
  robot_position *_rp;
  Adafruit_MotorShield *_afms;     ///< Motor shield object
  Adafruit_DCMotor *_right_back;   ///< Right back motor
  Adafruit_DCMotor *_right_front;  ///< Right front motor
  Adafruit_DCMotor *_left_front;   ///< Left front motor
  Adafruit_DCMotor *_left_back;    ///< Left back motor
  Servo *_servo_look;              ///< Servo to control the looking direction

public:
  /**
   * Constructor to initialize the motor shield and components.
   */
  obstacle_avoiding_robot() {
    _afms = new Adafruit_MotorShield();
    _rp = new robot_position();
    _servo_look = new Servo();
    _right_back = _afms->getMotor(1);
    _right_front = _afms->getMotor(2);
    _left_front = _afms->getMotor(3);
    _left_back = _afms->getMotor(4);
  }

  ~obstacle_avoiding_robot() {
    delete _afms;
    delete _rp;
    delete _servo_look;
  }

  /**
   * Setup the robot's components: Motor shield, servo, and ultrasonic sensor.
   */
  void begin() {
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

  /**
  * Function to check if the battery is low and if so, return the robot to its start position.
  * It puts the robot in sleep mode if the battery is low after starting the return process.
  *
  * @return true if the robot successfully starts returning to the start position, false otherwise.
  */
  bool go_to_start_position() {
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
  /**
   * Main loop to control robot's movement.
   * The robot moves forward if no obstacle is detected. 
   * If an obstacle is detected, it stops and checks the direction to avoid the obstacle.
   */
  void loop() {

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
      case 0: turn_left(400); break;
      case 1: turn_left(700); break;
      case 2: turn_right(400); break;
    }
  }

private:
  /**
  * @brief Reads battery voltage and checks for low battery condition.
  * 
  * This function:
  * - Reads the analog value from the battery pin.
  * - Converts it to actual voltage using the voltage divider formula.
  * - Prints a low battery warning if voltage is below the threshold.
  */
  bool battery_calculate() {
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

  /**
  * Function to drive the robot back to the start position.
  * The robot will move step-by-step, adjusting its position along the X and Y axes.
  * It will stop when the robot reaches its starting point (within a defined threshold).
  */
  void return_to_start() {
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
  /**
   * Moves the robot forward by running all motors in the forward direction.
   */
  void move_forward() {
    _rp->move_forward();
    accelerate();  // Smooth acceleration when moving forward
    _right_back->run(FORWARD);
    _right_front->run(FORWARD);
    _left_front->run(FORWARD);
    _left_back->run(FORWARD);
  }

  /**
  * @brief Moves the robot backward for a specified duration.
  * 
  * This function makes the robot move backward by activating all four motors
  * (left and right front and back motors) to run in the backward direction.
  * It also ensures that the motor speed is reset after the movement, and the
  * movement is stopped once the specified duration has passed.
  * 
  * @param duration The time, in milliseconds, for which the robot will move backward.
  *                 The robot will move for the entire specified duration before stopping.
  * 
  * @note After the movement is completed, the motor speeds are reset, and the motors
  *       are stopped to ensure that the robot doesn't continue moving.
  */
  void move_backward(int duration) {
    _rp->move_backward();
    _right_back->run(BACKWARD);   // Move the right back motor in the backward direction
    _right_front->run(BACKWARD);  // Move the right front motor in the backward direction
    _left_front->run(BACKWARD);   // Move the left front motor in the backward direction
    _left_back->run(BACKWARD);    // Move the left back motor in the backward direction
    delay(duration);              // Move for the specified duration (in milliseconds)
    reset_motor_speed();          // Reset the motor speed after movement
    stop_move();                  // Stop the motors to halt the movement
  }

  /**
  * @brief Stops the robot by releasing all motors.
  * 
  * This function halts the robot's movement by releasing control of all the motors. 
  * It ensures that all four motors (right front, right back, left front, and left back) 
  * are stopped, effectively bringing the robot to a halt. The motors are not actively 
  * powered but are released from their running state.
  * 
  * @note Calling this function does not immediately stop the motors in their tracks, 
  *       but it ensures that the motors no longer receive commands to continue moving.
  *       For a smoother stop, you may want to use deceleration methods beforehand.
  */
  void stop_move() {
      _right_back->run(RELEASE);    // Release the right back motor
      _right_front->run(RELEASE);   // Release the right front motor
      _left_front->run(RELEASE);    // Release the left front motor
      _left_back->run(RELEASE);     // Release the left back motor
  }

  /**
  * @brief Turns the robot left for a specified duration.
  * 
  * This function moves the robot in a leftward direction by running the left 
  * and right motors in opposite directions, resulting in a turn. The turn 
  * speed is controlled by the `_turn_speed` variable, and the robot will 
  * continue turning for the specified duration. After the movement, the 
  * motor speeds are reset, and the motors are stopped.
  * 
  * @param duration The time, in milliseconds, for which the robot will turn left.
  *                 The robot will continue turning for the entire specified duration 
  *                 before stopping.
  * 
  * @note After the turn is completed, the motor speeds are reset, and the motors 
  *       are stopped to ensure that the robot does not continue turning.
  */
  void turn_left(int duration) {
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

  /**
  * @brief Turns the robot right for a specified duration.
  * 
  * This function moves the robot in a rightward direction by running the left 
  * and right motors in opposite directions, causing the robot to turn right. 
  * The turn speed is controlled by the `_turn_speed` variable, and the robot 
  * will continue turning for the specified duration. After the movement, the 
  * motor speeds are reset, and the motors are stopped.
  * 
  * @param duration The time, in milliseconds, for which the robot will turn right.
  *                 The robot will continue turning for the entire specified duration 
  *                 before stopping.
  * 
  * @note After the turn is completed, the motor speeds are reset, and the motors 
  *       are stopped to ensure that the robot does not continue turning.
  */
  void turn_right(int duration) {
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

  /**
  * @brief Sets the turn speed for all motors.
  * 
  * This function adjusts the speed of the motors during a turn by adding a specified 
  * `speed` value to the current motor speeds. The left and right motors are adjusted 
  * separately for smoother turns.
  * 
  * @param speed The speed to be added to the motors during turns. This value will 
  *              increase the turn speed, making the robot turn faster. It is added 
  *              to the default motor speed (`_motor_speed`).
  * 
  * @note The left motors are also offset by `_motor_offset` to ensure that turning 
  *       is smooth and effective.
  */
  void set_turn_speed(uint8_t speed) {
    _right_back->setSpeed(_motor_speed + speed);                  // Set speed for right back motor
    _right_front->setSpeed(_motor_speed + speed);                 // Set speed for right front motor
    _left_front->setSpeed(_motor_speed + _motor_offset + speed);  // Set speed for left front motor with offset
    _left_back->setSpeed(_motor_speed + _motor_offset + speed);   // Set speed for left back motor with offset
  }

  /**
  * @brief Resets motor speed to the default speed.
  * 
  * This function resets the motor speeds to their default values, ensuring that 
  * the motors are running at the base speed (`_motor_speed`) for regular movement, 
  * without any turn-specific speed adjustments.
  */
  void reset_motor_speed() {
    _right_back->setSpeed(_motor_speed);                  // Reset speed for right back motor
    _right_front->setSpeed(_motor_speed);                 // Reset speed for right front motor
    _left_front->setSpeed(_motor_speed + _motor_offset);  // Reset speed for left front motor with offset
    _left_back->setSpeed(_motor_speed + _motor_offset);   // Reset speed for left back motor with offset
  }

  /**
  * @brief Gradually accelerates motors to smoothen the start of movement.
  * 
  * This function smoothly increases the motor speeds from 0 to the desired speed (`_motor_speed`) 
  * to provide a smoother start to the robot's movement. The gradual acceleration prevents sudden jerks 
  * and ensures that the robot starts moving more smoothly.
  */
  void accelerate() {
    for (uint8_t speed = 0; speed <= _motor_speed; speed++) {
      _right_back->setSpeed(speed);                  // Gradually increase speed for right back motor
      _right_front->setSpeed(speed);                 // Gradually increase speed for right front motor
      _left_front->setSpeed(speed + _motor_offset);  // Gradually increase speed for left front motor with offset
      _left_back->setSpeed(speed + _motor_offset);   // Gradually increase speed for left back motor with offset
      delay(20);                                     // Delay for smoother transition
    }
  }

  /**
  * @brief Gradually decelerates motors to smoothen the stop of movement.
  * 
  * This function gradually decreases the motor speeds from the current speed 
  * (`_motor_speed`) to 0, ensuring that the robot slows down smoothly and does 
  * not come to an abrupt stop. The deceleration helps prevent the robot from 
  * jerking or skidding when stopping.
  */
  void decelerate() {
    for (uint8_t speed = _motor_speed; speed >= 0; speed--) {
      _right_back->setSpeed(speed);                  // Gradually decrease speed for right back motor
      _right_front->setSpeed(speed);                 // Gradually decrease speed for right front motor
      _left_front->setSpeed(speed + _motor_offset);  // Gradually decrease speed for left front motor with offset
      _left_back->setSpeed(speed + _motor_offset);   // Gradually decrease speed for left back motor with offset
      delay(20);                                     // Delay for smoother transition
    }
  }


  /**
   * Measures the distance to an object using ultrasonic sensor.
   * 
   * @return The distance to the nearest object in centimeters.
   */
  int get_distance() {
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
  /**
   * Checks the left and right directions and determines which way to turn.
   * 
   * @return The direction to turn: 0 for left, 1 for reverse, 2 for right.
   */
  uint8_t check_direction() {
    int distances[2] = { 0, 0 };  // Array to store the distances for both directions (right and left)
    uint8_t turn_dir = 1;         // Default turn direction (1 for reverse)

    // Move servo to the right to check the distance
    _servo_look->write(180);        // Look to the right
    delay(500);                     // Wait for the servo to reach the right position
    distances[0] = get_distance();  // Measure the distance to the right

    // Move servo to the left to check the distance
    _servo_look->write(0);          // Look to the left
    delay(1000);                    // Wait for the servo to reach the left position
    distances[1] = get_distance();  // Measure the distance to the left

    // Decision logic based on the measured distances
    if (distances[0] >= 200 && distances[1] >= 200)  // If both sides have large open space
      turn_dir = 0;                                  // Turn left (0)

    else if (distances[0] <= _stop_dist && distances[1] <= _stop_dist)  // If both sides are blocked
      turn_dir = 1;                                                     // Reverse (1)

    else if (distances[0] >= distances[1])  // If the right side has more space
      turn_dir = 0;                         // Turn left (0)

    else if (distances[0] < distances[1])  // If the left side has more space
      turn_dir = 2;                        // Turn right (2)

    return turn_dir;  // Return the calculated turn direction
  }
};

// Create an instance of the robot
static const obstacle_avoiding_robot robot;

void setup() {
  robot.begin();
}

void loop() {
  robot.loop();
}