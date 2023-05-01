# Genetic-Algorithm-Robot
## Genetic Algorithm-based Obstacle Avoiding Robot Simulation

### Robot
This is a very simple differential robot. It follows the widely-used velocity model. It can be controlled with keyboard buttons as follows:
* W: positive increment of left wheel motor speed
* S: negative increment of left wheel motor speed
* O: positive increment of right wheel motor speed
* L: negative increment of right wheel motor speed
* X: both motor speeds are zero
* T: positive increment of both wheels’ motor speed
* G: negative increment of both wheels’ motor speed

Make the variable ```we_play = True in main.py``` to manually control the robot. When it is 'False', the robot goes into the learning phase by using Neural Network and Genetic Algorithm.

The robot has 12 omni-directional IR sensors to detect the obstacles. Obstacle avoidance is triggered by setting a threshold on the sensor value. The following rules are followed to avoid obstacles:
* If the robot strikes the wall at 90&deg; , it stops.
* If it strikes the wall at another angle, it slides along the wall until interrupted based on previous heading.

### Learning
The robot avoids obstacles while collecting dust from the map. The most important part of the learning process is the Neural Network and the Genetic Algorithm.
* The Neural Network takes the sensor distances and modifies the motor speed based on that.
* The control mechanism of the Neural Network is used in the Evolutionary Algorithm to create genomes. As generation evolves, the genomes learn the best starting position, motor speed, heading, and thus learns to avoid the obstacle while collecting dust. The fitness value evolves as the number of generation increases.
* The fitness value is calculated using the total number of dust collected and number of collisions. This helps the algorithm learn to avoid collisions and collect dust.

To execute the simulation run ```python main.py```
