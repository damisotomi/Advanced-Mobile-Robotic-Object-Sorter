# Advanced-Mobile-Robotic-Object-Sorter
An Arduino-powered mobile robot with PID control for precise path navigation and object sorting. Utilized a QTR sensor for line detection, a gripping mechanism, and color identification for automated object retrieval and sorting based on color criteria  

## WHY IT WAS DEVELOPED
  The advanced mobile robot sorter was developed as part of my semester project for the advanced mechatronics course during my first year second semester in the mechanical engineering master program at New York University
  
  ![image](https://github.com/damisotomi/Advanced-Mobile-Robotic-Object-Sorter-v1/assets/67606934/b3ebde0a-5c54-473c-946f-ea0ddf9fdaa8)
  
## HOW IT WORKS
  -This advanced mobile robot sorter has 3 modes.
### Mode 1. The robot moves on a black line using PID control with a QTR 8RC sensor and stops at the end of its path if no obstacle is seen within a certain distance. 
### Mode 2: Robot moves on the black line using PID control with a QTR 8RC sensor, tries to pick up an object it senses but then continues its journey if no object was picked up
### Mode 3, the robot moves on the black line, picks up an actual object, returns it to the base, and sorts based on color (Red to the left and others to the right)

### MICROCONTROLLER AND PROGRAMMING LANGUAGE USED
  - An Arduino Uno Microcontroller was used to build this project as required by the project guidelines
  - The programming language used was C programming

### Components used
  - Ultrasonic distance sensor - For obstacle detection
  - IR infrared Obstacle avoidance sensor- To detect the presence of an object nearby
  - TCS34725 Sensor Recognition Module RGB Sensor- To identify the color of the detected object
  - L298N Motor Driver- To control the DC motors used to control the wheels of the car and also to receive 5v volts from the Arduino for logic circuitry of the wheels 
  - Qtr 8RC Reflectance Sensor array - for line detection and following 
  - Connecting wires
  - Resistors
  - Gripper 3.0 - From parallax website- To grip objects
  - Standard Servo motors- To control the gripper opening and closing mechanism

### FUTURE IMPROVEMENTS
  - To replace the functionalities of the ultrasonic sensor, IR infared obstacle avoidance sensor, and TCS34725 sensor with a Pixy 2 camera for better obstacle detection
  - and more accurate color sorting as seen in Advanced-Mobile-Robotic-Object-Sorter v2

### HOW TO USE
  - Create a black line path using black tape on a white surface for the robot to follow
  - Turn on the power switch of the motor driver of the assembled robot
  - Place an obstacle along the path of the robot visible enough for the ultrasonic sensor to see and the gripper to grip
  - watch as the robot moves along the path and picks up obstacles

### Live demo
https://drive.google.com/file/d/1b-3PlHTEjwaR0bIDIo_oKqriqa6O3WmA/view?usp=sharing
