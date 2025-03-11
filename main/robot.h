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


#ifndef _robot_main_h_
#define _robot_main_h_

// Robot Obstacle Avoidance with Low Power Management (Arduino Uno)
#include "config.h"

#ifdef USE_ADAFRUIT_V2
#include <Adafruit_MotorShield.h>
#else
#include <AFMotor.h>
#endif //!USE_ADAFRUIT_V1

#include <Servo.h>
#include "sensor.h"
#include "robot-position.h"


/**
 * Class to control the obstacle-avoiding robot with motor shield and ultrasonic sensor.
 */
class advanced_robot {
private:
  bool _is_first_time = true;
  bool _is_moving_forward = false;
  robot_position *_rp;
#ifdef USE_ADAFRUIT_V2
  Adafruit_MotorShield *_afms;     ///< Motor shield object
  Adafruit_DCMotor *_right_back;   ///< Right back motor
  Adafruit_DCMotor *_right_front;  ///< Right front motor
  Adafruit_DCMotor *_left_front;   ///< Left front motor
  Adafruit_DCMotor *_left_back;    ///< Left back motor
#else
  AF_DCMotor *_right_back;   ///< Right back motor
  AF_DCMotor *_right_front;  ///< Right front motor
  AF_DCMotor *_left_front;   ///< Left front motor
  AF_DCMotor *_left_back;    ///< Left back motor
#endif //!USE_ADAFRUIT_V2
  sensor_t *_sensor;              ///< Servo sensor to control the looking direction

public:
  /**
   * Constructor to initialize the motor shield and components.
   */
  advanced_robot();

  /**
  * @brief Destructor for the `advanced_robot` class.
  * 
  * This destructor is responsible for cleaning up and releasing resources when the 
  * `advanced_robot` object is destroyed. It ensures that all dynamically 
  * allocated memory associated with the object is properly freed to prevent memory 
  * leaks.
  * 
  * Specifically, it deletes the following objects:
  * - `_afms`: A pointer to the Adafruit Motor Shield instance, which controls the robot's motors.
  * - `_rp`: A pointer to the robot's position object, which tracks the robot's current position.
  * - `_servo_look`: A pointer to the servo motor used for scanning and obstacle detection.
  * 
  * These deletions ensure that all dynamically allocated memory is properly managed 
  * and cleaned up when the object goes out of scope or is explicitly destroyed.
  * 
  * @note This destructor does not free any memory associated with statically allocated 
  *       members or memory managed by other parts of the program. It only manages 
  *       memory specifically allocated within the `advanced_robot` class.
  */
  ~advanced_robot();

  /**
   * Setup the robot's components: Motor shield, servo, and ultrasonic sensor.
   */
  void begin();

  /**
  * @brief Checks if the battery is low and, if so, returns the robot to its start position.
  * 
  * This function monitors the robot's battery level. If the battery is low, the robot 
  * will attempt to return to its start position using the `return_to_start` function. 
  * Once the return process begins, the robot will enter a low-power sleep mode to 
  * conserve energy, allowing it to save power until it either runs out of battery or 
  * the sleep period expires.
  * 
  * The robot will enter sleep mode for 8 seconds (as controlled by the `LowPower` library) 
  * to reduce power consumption during the return process. The sleep mode helps prevent 
  * further battery depletion during the return journey.
  * 
  * @return bool Returns `true` if the robot successfully starts returning to the start 
  *         position (battery is low), or `false` if the battery is not low and no 
  *         return process is initiated.
  * 
  * @note This function is intended to be called when the robot detects that its battery 
  *       is running low, and it is not designed to monitor battery levels continuously.
  *       The sleep mode helps the robot conserve energy while returning to the start position.
  * 
  * @see return_to_start() for the procedure that brings the robot back to its start position.
  * @see battery_calculate() for the logic that checks the battery level.
  */
  bool go_to_start_position();

  /**
   * Main loop to control robot's movement.
   * The robot moves forward if no obstacle is detected. 
   * If an obstacle is detected, it stops and checks the direction to avoid the obstacle.
   */
  void loop();

private:

  /**
  * @brief Controls the robot's autonomous movement with obstacle avoidance (It could drain the battery life).
  * 
  * This function enables the robot to navigate independently by continuously monitoring 
  * its surroundings and making movement decisions in real-time. It prevents collisions 
  * by detecting obstacles and adjusting the robot's direction accordingly.
  * 
  * ## Function Behavior:
  * - Runs in a continuous loop, actively checking distances in **three directions**:
  *   - **Straight ahead**
  *   - **Right**
  *   - **Left**
  * - Calls `calculate_direction()` to determine the best movement strategy:
  *   - **Move forward** if there is sufficient space ahead.
  *   - **Turn left** or **turn right** based on available space.
  *   - **Move backward** if no viable path exists.
  * - Executes the appropriate movement command (`turn_left()`, `turn_right()`, or `move_backward()`).
  * - The function **terminates** when `go_to_start_position()` returns `true`, indicating 
  *   that the robot should return to its starting position (e.g., due to low battery).
  * 
  * ## Assumptions:
  * - The robot is equipped with **distance sensors** to detect obstacles.
  * - Motor control functions (`turn_left()`, `turn_right()`, `move_backward()`) are available.
  * - A predefined threshold `_stop_dist` is used to decide movement actions.
  * 
  * @note This function operates indefinitely unless the robot needs to return to its start position.
  */
  void autonomous_navigation();

  /**
  * @brief Reads battery voltage and checks for low battery condition.
  * 
  * This function:
  * - Reads the analog value from the battery pin.
  * - Converts it to actual voltage using the voltage divider formula.
  * - Prints a low battery warning if voltage is below the threshold.
  */
  bool battery_calculate();

  /**
  * @brief Drives the robot back to the start position, adjusting its position along 
  *        the X and Y axes.
  * 
  * This function guides the robot step-by-step back to its start position. It continuously 
  * adjusts the robot's movement based on its current position relative to the start 
  * position along the X and Y axes. The robot will move either left, right, forward, 
  * or backward depending on the distance between the robot's current position and the 
  * starting point.
  * 
  * The movement will continue until the robot reaches the start position within a defined 
  * threshold (1.0 unit) to avoid issues with floating-point precision. The function 
  * dynamically recalculates the distance to the start point after each movement to 
  * ensure accurate navigation. Once the robot is within the threshold distance, it will stop.
  * 
  * @note The robot adjusts its movement based on the position relative to the X and Y axes 
  *       (using `get_x_position()` and `get_y_position()`). The function ensures the robot 
  *       returns to its start position smoothly by executing small movements in either 
  *       direction.
  * 
  * @see calculate_distance_to_start() for how the distance is measured.
  * @see stop_move() for stopping the robot after reaching the start position.
  */
  void return_to_start();

  /**
  * @brief Moves the robot forward by running all motors in the forward direction.
  * 
  * This function initiates movement by commanding all four motors (right front, right back, 
  * left front, and left back) to run in the forward direction. The movement is accompanied 
  * by a smooth acceleration to avoid sudden starts and provide a more controlled movement.
  * 
  * The `accelerate()` function is called before the motors are activated to gradually 
  * increase the speed, ensuring smoother and more stable motion. This is especially important 
  * in robotic applications where abrupt movements may cause instability or discomfort.
  * 
  * @note The motors will continue running in the forward direction until another command 
  *       (such as stop or turn) is issued. This function only moves the robot forward 
  *       without any checks for obstacles or path planning.
  * 
  * @see accelerate() for how the acceleration is applied.
  */
  void move_forward();

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
  void move_backward(int duration);

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
  void stop_move();

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
  void turn_left(int duration);

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
  void turn_right(int duration);

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
  void set_turn_speed(uint8_t speed);

  /**
  * @brief Resets motor speed to the default speed.
  * 
  * This function resets the motor speeds to their default values, ensuring that 
  * the motors are running at the base speed (`_motor_speed`) for regular movement, 
  * without any turn-specific speed adjustments.
  */
  void reset_motor_speed();

  /**
  * @brief Gradually accelerates motors to smoothen the start of movement.
  * 
  * This function smoothly increases the motor speeds from 0 to the desired speed (`_motor_speed`) 
  * to provide a smoother start to the robot's movement. The gradual acceleration prevents sudden jerks 
  * and ensures that the robot starts moving more smoothly.
  */
  void accelerate();

  /**
  * @brief Gradually decelerates motors to smoothen the stop of movement.
  * 
  * This function gradually decreases the motor speeds from the current speed 
  * (`_motor_speed`) to 0, ensuring that the robot slows down smoothly and does 
  * not come to an abrupt stop. The deceleration helps prevent the robot from 
  * jerking or skidding when stopping.
  */
  void decelerate();

  /**
  * @brief Measures and describes distances in three directions: straight, right, and left.
  * 
  * This function reads the distance in front, then moves a servo to check the distances 
  * on the right and left before returning to the center position.
  * 
  * @param[out] straight Distance measured in the forward direction.
  * @param[out] right Distance measured to the right.
  * @param[out] left Distance measured to the left.
  */
  void describe_distance(int &straight, int &right, int &left);
  /**
  * @brief Checks the left and right directions and determines which way to turn.
  * 
  * This function uses a servo to measure the distance on both the left and right 
  * sides of the robot. Based on the distances, it decides which direction the robot 
  * should turn: left, right, or reverse. The decision-making process ensures the 
  * robot turns based on available space and obstacles.
  * 
  * The servo is moved to the right and left to measure distances, and based on 
  * the readings, a decision is made:
  * - If both sides have clear space, it turns left.
  * - If both sides are blocked, it reverses.
  * - If one side has more space, it chooses that direction.
  * 
  * @return uint8_t The direction to turn:
  *                  - ROBOT_TURN_LEFT for left,
  *                  - ROBOT_TURN_REVERSE for reverse,
  *                  - ROBOT_TURN_RIGHT for right.
  * 
  * @note The function relies on distance measurements to make the decision, 
  *       so it’s essential that the `_sensor->get_distance()` function provides accurate 
  *       readings. The servo’s position is adjusted to check the left and right 
  *       sides of the robot, and there are delays to ensure the servo has enough 
  *       time to move to the correct positions.
  */
  uint8_t check_direction(int straight_distance = 0);

  /**
  * @brief Determines the robot's movement direction based on detected distances.
  * 
  * This function evaluates the distances in front, to the right, and to the left 
  * and decides whether the robot should move forward, turn left, turn right, or reverse.
  * 
  * @param[in] straight Distance measured in the forward direction.
  * @param[in] right Distance measured to the right.
  * @param[in] left Distance measured to the left.
  * @return uint8_t Direction constant indicating the robot's next movement.
  *         - `ROBOT_MOVE` (0) – Move forward.
  *         - `ROBOT_TURN_LEFT` (1) – Turn left.
  *         - `ROBOT_TURN_RIGHT` (2) – Turn right.
  *         - `ROBOT_TURN_REVERSE` (3) – Reverse.
  */
  uint8_t calculate_direction(const int &straight, const int &right, const int &left) const;
};

#endif  //!_robot_main_h_