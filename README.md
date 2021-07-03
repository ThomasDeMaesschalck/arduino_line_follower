# Arduino line follower robot
School project for Programming 2. 

Requires an Arduino Mega board, two DC motors, two ultrasonic sensors, a buzzer, RGB LEDs, two color sensors, a battery, a ton of cables, etc.

Arduino line follower robot with the following capabilities:
- Performs countdown procedure on startup.
- Automatically follows a white line.
- Can take gentle corners, 90° turns and very sharp turns.
- When the end of the white line is reached, the robot will turn and follow the line in the reverse direction.
- Has obstacle detection. Robot will slow down and activate the buzzer (like a parking sensor) when an obstacle is detected. 
- Comes to a full stop when the obstacle is very near and will blink all LEDs.
- When the obstacle is removed, the robot resumes operation.
- Detects a "bridge". In this case, an object located up to 50cm above the robot. Robot will blink all LEDs.
- The robot's operation LED changes color when a horizontal white line is detected
- Robot performs 360-degrees counterclockwise turn when a red circle is detected left of the robot
- Robot performs 360-degrees clockwise turn when a green  circle is detected right of the robot

![IMG-20190611-WA0000](https://user-images.githubusercontent.com/58608713/124362568-00851b00-dc36-11eb-88d1-8bb36f0a82a7.jpeg)

