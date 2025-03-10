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
// 3:11 PM 3/8/2025

#ifndef _robot_position_h_
#define _robot_position_h_
#include <math.h>

/**
 * @class robot_position
 * @brief Represents a robot's position in a 2D coordinate system and provides functionality to move and calculate distance.
 * 
 * This class allows you to track and manipulate the robot's position in a 2D space, where the robot's starting 
 * position is at the origin (0, 0). It provides movement methods that allow the robot to move in four directions:
 * left, right, forward, and backward, by a defined distance. It also offers functionality to calculate the distance
 * to the starting point, which can be used for navigation or determining when the robot has returned to its origin.
 * 
 * The position of the robot is tracked by two variables, _x_position and _y_position, which represent the robot's 
 * current coordinates. The class assumes that the robot's initial position is at the origin (0, 0), and its movements 
 * adjust these coordinates accordingly. It can be used in robotics applications where the robot needs to move around 
 * a space and return to its starting position or calculate its distance from the start.
 */
class robot_position {
public:
  /**
   * @brief Default constructor for the robot_position class.
   * 
   * Initializes the robot's position at the origin (0, 0). The robot starts at a known position for ease of 
   * movement and distance calculation.
   */
  robot_position() {}

  /**
   * @brief Move the robot left by a defined distance.
   * 
   * This function decreases the robot's X-coordinate by a set distance, effectively moving it to the left on the
   * coordinate grid. The exact distance moved per call is defined by the _move_distance constant.
   */
  void move_left();

  /**
   * @brief Move the robot right by a defined distance.
   * 
   * This function increases the robot's X-coordinate by a set distance, effectively moving it to the right on the 
   * coordinate grid. The exact distance moved per call is defined by the _move_distance constant.
   */
  void move_right();

  /**
   * @brief Move the robot forward by a defined distance along the Y-axis.
   * 
   * This function increases the robot's Y-coordinate by a set distance, moving the robot in a positive direction 
   * along the Y-axis. The exact distance moved per call is defined by the _move_distance constant.
   */
  void move_forward();

  /**
   * @brief Move the robot backward by a defined distance along the Y-axis.
   * 
   * This function decreases the robot's Y-coordinate by a set distance, moving the robot in a negative direction 
   * along the Y-axis. The exact distance moved per call is defined by the _move_distance constant.
   */
  void move_backward();

  /**
   * @brief Calculate the Euclidean distance from the robot's current position to the origin (0, 0).
   * 
   * This function computes the distance using the Pythagorean theorem: sqrt(x^2 + y^2), where x and y are the 
   * robot's current coordinates. This value represents the straight-line distance from the current position to 
   * the starting point (0, 0), and can be used to determine how far the robot has traveled from its origin.
   * 
   * @return The calculated distance in centimeters.
   */
  float calculate_distance_to_start() const;

  /**
   * @brief Get the current X-coordinate of the robot.
   * 
   * This function returns the robot's current position along the X-axis.
   * 
   * @return The X-coordinate of the robot in centimeters.
   */
  float get_x_position() const;

  /**
   * @brief Get the current Y-coordinate of the robot.
   * 
   * This function returns the robot's current position along the Y-axis.
   * 
   * @return The Y-coordinate of the robot in centimeters.
   */
  float get_y_position() const;

private:
  float _x_position = 0.0; /**< The X-coordinate of the robot's position, starting at 0.0. */
  float _y_position = 0.0; /**< The Y-coordinate of the robot's position, starting at 0.0. */
};

#endif  //!_robot_position_h_