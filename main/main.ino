// Rajib chy
// 12:24 AM 3/8/2025
// Obstacle Avoiding Robot

#include <Adafruit_MotorShield.h>
#include <Servo.h>

constexpr uint8_t _trig = 2;                                             ///< Trigger pin for ultrasonic sensor
constexpr uint8_t _echo = 13;                                            ///< Echo pin for ultrasonic sensor
constexpr uint8_t _max_dist = 150;                                       ///< Maximum distance (cm) for obstacle detection
constexpr uint8_t _stop_dist = 50;                                       ///< Minimum distance (cm) to stop before an obstacle
constexpr float _time_out = 2 * (_max_dist + 10) / 100 / 340 * 1000000;  ///< Timeout for ultrasonic sensor measurement

constexpr uint8_t _motor_speed = 55;   ///< Motor speed (0 to 255)
constexpr uint8_t _motor_offset = 10;  ///< Motor offset to balance motor power
constexpr uint8_t _turn_speed = 50;    ///< Speed boost for turning
/**
 * Class to control the obstacle-avoiding robot with motor shield and ultrasonic sensor.
 */
class obstacle_avoiding_robot {
private:
  Adafruit_MotorShield _afms;      ///< Motor shield object
  Adafruit_DCMotor *_right_back;   ///< Right back motor
  Adafruit_DCMotor *_right_front;  ///< Right front motor
  Adafruit_DCMotor *_left_front;   ///< Left front motor
  Adafruit_DCMotor *_left_back;    ///< Left back motor
  Servo _servo_look;               ///< Servo to control the looking direction

public:
  /**
   * Constructor to initialize the motor shield and components.
   */
  obstacle_avoiding_robot() {
    _afms = Adafruit_MotorShield();
    _right_back = _afms.getMotor(1);
    _right_front = _afms.getMotor(2);
    _left_front = _afms.getMotor(3);
    _left_back = _afms.getMotor(4);
  }

  /**
   * Setup the robot's components: Motor shield, servo, and ultrasonic sensor.
   */
  void begin() {
    Serial.begin(9600);    // Start serial communication
    if (!_afms.begin()) {  // Check if the motor shield is detected
      while (1) {}         // Stop execution if motor shield is not found
    }

    // Set motor speeds
    _right_back->setSpeed(_motor_speed);
    _right_front->setSpeed(_motor_speed);
    _left_front->setSpeed(_motor_speed + _motor_offset);
    _left_back->setSpeed(_motor_speed + _motor_offset);

    // Ensure motors are stopped
    stop_move();

    _servo_look.attach(10);  // Attach servo to pin 10
    pinMode(_trig, OUTPUT);  // Set trigger pin as output
    pinMode(_echo, INPUT);   // Set echo pin as input
  }

  /**
   * Main loop to control robot's movement.
   * The robot moves forward if no obstacle is detected. 
   * If an obstacle is detected, it stops and checks the direction to avoid the obstacle.
   */
  void loop() {
    _servo_look.write(90);          // Look straight ahead
    delay(750);                     // Wait before measuring distance
    int distance = get_distance();  // Get the current distance from the obstacle

    // Move forward if no obstacle is too close
    if (distance >= _stop_dist) {
      move_forward();
    }

    // Keep checking until the distance is less than _stop_dist
    while (distance >= _stop_dist) {
      distance = get_distance();
      delay(250);  // Wait before re-checking distance
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
   * Moves the robot forward by running all motors in the forward direction.
   */
  void move_forward() {
    accelerate();  // Smooth acceleration when moving forward
    _right_back->run(FORWARD);
    _right_front->run(FORWARD);
    _left_front->run(FORWARD);
    _left_back->run(FORWARD);
  }

  /**
   * Stops the robot by releasing all motors.
   */
  void stop_move() {
    _right_back->run(RELEASE);
    _right_front->run(RELEASE);
    _left_front->run(RELEASE);
    _left_back->run(RELEASE);
  }

  /**
   * Turns the robot left for a specified duration.
   * 
   * @param duration The duration to turn left.
   */
  void turn_left(int duration) {
    set_turn_speed(_turn_speed);
    _right_back->run(FORWARD);
    _right_front->run(FORWARD);
    _left_front->run(BACKWARD);
    _left_back->run(BACKWARD);
    delay(duration);
    reset_motor_speed();
    stop_move();
  }

  /**
   * Turns the robot right for a specified duration.
   * 
   * @param duration The duration to turn right.
   */
  void turn_right(int duration) {
    set_turn_speed(_turn_speed);
    _right_back->run(BACKWARD);
    _right_front->run(BACKWARD);
    _left_front->run(FORWARD);
    _left_back->run(FORWARD);
    delay(duration);
    reset_motor_speed();
    stop_move();
  }

  /**
   * Sets turn speed for all motors.
   * 
   * @param speed The speed to be added to the motors during turns.
   */
  void set_turn_speed(uint8_t speed) {
    _right_back->setSpeed(_motor_speed + speed);
    _right_front->setSpeed(_motor_speed + speed);
    _left_front->setSpeed(_motor_speed + _motor_offset + speed);
    _left_back->setSpeed(_motor_speed + _motor_offset + speed);
  }

  /**
   * Resets motor speed to default speed.
   */
  void reset_motor_speed() {
    _right_back->setSpeed(_motor_speed);
    _right_front->setSpeed(_motor_speed);
    _left_front->setSpeed(_motor_speed + _motor_offset);
    _left_back->setSpeed(_motor_speed + _motor_offset);
  }

  /**
   * Gradually accelerates motors to smoothen the start of movement.
   */
  void accelerate() {
    for (uint8_t speed = 0; speed <= _motor_speed; speed++) {
      _right_back->setSpeed(speed);
      _right_front->setSpeed(speed);
      _left_front->setSpeed(speed + _motor_offset);
      _left_back->setSpeed(speed + _motor_offset);
      delay(20);  // Delay for smoother transition
    }
  }

  /**
   * Gradually decelerates motors to smoothen the stop of movement.
   */
  void decelerate() {
    for (uint8_t speed = _motor_speed; speed >= 0; speed--) {
      _right_back->setSpeed(speed);
      _right_front->setSpeed(speed);
      _left_front->setSpeed(speed + _motor_offset);
      _left_back->setSpeed(speed + _motor_offset);
      delay(20);  // Delay for smoother transition
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
    _servo_look.write(180);         // Look to the right
    delay(500);                     // Wait for the servo to reach the right position
    distances[0] = get_distance();  // Measure the distance to the right

    // Move servo to the left to check the distance
    _servo_look.write(0);           // Look to the left
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
obstacle_avoiding_robot robot;

void setup() {
  robot.begin();
}

void loop() {
  robot.loop();
}