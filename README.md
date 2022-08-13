# Self-driving vehicle simulation

Our goal is to train a self-driving robot (Turtlebot) in a 3D world simulator ([Gazebo](https://wiki.ros.org/turtlebot3_gazebo)). The vehicle will use its camera to detect objects including stop signs and target items. The vehicle will plan its motion path efficiently using motion curves, and update the motion curve dynamically as new information is gathered from the surroundings. 

_Turtlebot (lower-left) along with position and velocity tracking measurements_

<img width="1898" alt="Screen Shot 2020-10-19 at 8 39 37 PM" src="https://user-images.githubusercontent.com/3321825/184516551-71aba629-bf6e-4728-8d9d-5bea9517239f.png">

## Detection

Turtlebot is a two-wheeled robot with a front-facing camera and range-detectors around the perimeter of the robot. Using the range-finding scans, the vehicle forms a map of the explored regions. Positions of objects and walls are determined by their relatively stable positions, and the absolute `x, y` position of the vehicle is determined by a combination of distance tracking of the vehicle’s wheels and distance/angle to stable objects. 

_Map of explored regions based on range scans_
<img width="1282" alt="Screen Shot 2020-10-20 at 4 49 29 PM" src="https://user-images.githubusercontent.com/3321825/184516583-689d2ad2-28e9-45ae-a470-bda7407ed32a.png">

Using the front facing camera, the vehicle runs a MobileNet Tensorflow model (convolutional neural network) to detect images in the scene. The model returns properties including label identity, classification probability, left & right bounding rays, bounding box, and object name. Using these values, the vehicle will recognize and stop in front of stop signs, and it will move toward target items (boxes with food items). 

_The vehicle's camera view of food items_

<img width="389" alt="Screen Shot 2020-11-16 at 9 10 55 PM" src="https://user-images.githubusercontent.com/3321825/184516599-19a8c7c5-1b7d-4754-9110-7e2fa293718a.png">
<img width="396" alt="Screen Shot 2020-11-16 at 10 45 25 PM" src="https://user-images.githubusercontent.com/3321825/184516670-7a73893a-b6b2-45ac-9c02-80e5197c011c.png">
<img width="388" alt="Screen Shot 2020-11-16 at 10 24 14 PM" src="https://user-images.githubusercontent.com/3321825/184516657-0d7e889a-a2a5-4a10-b447-9a9500af7f14.png">

## Motion planning

Vehicle trajectory is planned through dynamically solving the differential equation that ensures pose stabilization at the destination position & angle (compared to the current position & angle). This outputs a smooth curve vehicle trajectory which is then turned into a motor output speed plan for the two wheels. The trajectory tracking involves solving a Hamiltonian matrix to efficiently calculate the route in realtime. Further research is needed to ensure robustness to the limitation of positional & speed inaccuracies - these minor inaccuracies can cause deviation from the planned path. 

_Motion planning with wall obstacles_

<img width="1433" alt="Screen Shot 2020-11-07 at 1 36 28 PM" src="https://user-images.githubusercontent.com/3321825/184516740-e9726b48-d1b1-4de7-a687-b2a1a66616ca.png">

_2D map of smooth motion trajectory (green line)_

<img width="516" alt="Screen Shot 2020-11-19 at 2 51 21 PM" src="https://user-images.githubusercontent.com/3321825/184516742-6ad30a56-5ab6-4fb3-8a97-9ba1e9d8454f.png">



## References

1. Son, Will. TurtleBot Gazebo. ros.org. (n.d.). https://wiki.ros.org/turtlebot3_gazebo 
2. Ikramov, Khakim D. (2001), "Hamiltonian square roots of skew-Hamiltonian matrices revisited", Linear Algebra and its Applications, 325: 101–107, https://doi.org/10.1016/S0024-3795(00)00304-9.
3. Meyer, K. R.; Hall, G. R. (1991), Introduction to Hamiltonian dynamical systems and the N-body problem, [Springer, ISBN 0-387-97637-X.](https://en.wikipedia.org/wiki/Special:BookSources/0-387-97637-X)
4. Dragt, Alex J. (2005), "The symplectic group and classical mechanics", Annals of the New York Academy of Sciences, 1045 (1): 291–307, https://pubmed.ncbi.nlm.nih.gov/15980319
5. Waterhouse, William C. (2005), "The structure of alternating-Hamiltonian matrices", Linear Algebra and its Applications, 396: 385–390, https://doi.org/10.1016/j.laa.2004.10.003
6. Paige, Chris; Van Loan, Charles (1981), "A Schur decomposition for Hamiltonian matrices", Linear Algebra and its Applications, 41: 11–32, https://doi.org/10.1016/0024-3795(81)90086-0



