## Robot Obstacle Avoidance with Low Power Management (Arduino Uno)

This project demonstrates a simple obstacle-avoiding robot built using an Arduino Uno. The robot utilizes the Adafruit Motor Shield and Adafruit BusIO libraries to control movement and detect obstacles via sensors. Additionally, it incorporates Low-Power-1.81 to efficiently manage power, putting the robot into low-power sleep modes when necessary, especially when the battery is low.

## Key Features:
- **Obstacle Avoidance**: Robot moves in all directions (left, right, forward, and backward) while avoiding obstacles.
- **Low Battery Return**: When the battery runs low, the robot automatically returns to its start position and enters sleep mode using the `LowPower.powerDown` function.

## Requirements

- Arduino Uno
- Adafruit Motor Shield
- Adafruit BusIO
- Low-Power-1.81

## Setup

1. Install the **Adafruit Motor Shield and Adafruit BusIO** libraries.
2. Connect the motor shield to the **Arduino Uno**.
3. Upload the code to the **Arduino Uno**.

## License

This project is licensed under the MIT License.
