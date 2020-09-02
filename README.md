# Grid-Localization-using-Bayes-Filter
Implementation of Grid Localization which is a variant of discrete Bayes Localization, using OS Bags and visualizing it in Rviz.

## Following repository contains:-
1. Grid.bag - The ROS bag file that contains the observation and motion data as messages
2. markers_example.py - An example python node that  contains helper functions to display markers in rviz. You may use the helper functions directly in your code and tweak them accordingly. 
Please read through the comments in the code  and test it out to understand how you can use the helper functions in your code.


## Explaination

Grid Localization is a variant ofdiscrete Bayes Localization. In this method, the map is an occupancy grid.
At each timestep, the algorithm finds out the probabilities of the robot’s presence at every grid cell.
The grid cells with maximum probabilities at each step, characterize the robot’s trajectory.
Grid Localization using Bayes filter runs in two iterative steps that utilize the movementdata (prediction step) and observation data (update step). 
If you download the repository and run it, it should do the following-:
1. pa3.launch: Running it should read the bag file, run the grid localization algorithm and display visualizations in rviz as follows:
1. A cube plotted for each of the tags (landmarks) at the appropriate locations as given in the description.
2. A marker of type LINE_STRIP which represents the path/trajectory that therobot has taken. For each step in the Bayes Filter (i.e after eachmeasurement update), it would add the point with the maximum probability to the line strip and publish to rviz.
3. estimation.txt: Each line in the text file contains the most probable state afterincorporating each movement and measurement data.For the most probable state after incorporating the movement data, output would be lineof the format (‘P’ stands for prediction):
P: (x, y, θ)For the most probable state after incorporating the measurement data, output aline of the format (‘U’ stands for update):
U: (x, y, θ)Here-: (x, y, θ) is the robot’s pose expressed in the continuous world.


## Concepts
### 1.Occupancy Grid 

Fig.1: Occupancy Grid mapGrid Localization discretizes the world that’s usually represented in a continuous form. 

Figure 1 isan example of a 2D occupancy grid. For this project we made a grid for 7m × 7m coverage. 
Your cell size should be 20cm × 20cm. You also should assume a dimension for the robot’s heading.  
So your grid is 3 dimensional. The third dimension covers the robot’s heading. You can discretize thatwith some specific value (10 degrees, 20 degrees or more). 
The initial position of your robot within the grid is (12, 28) and the initial heading is 200.52 degrees. The robot’s first pose in the 3rddimension depends on your selected discretization size. 
For example, if your discretization size is 90 degrees, the cell number in the 3rd dimension will be 200.52/90+1 = 3. So the robot’s initialpose is (12 , 28, 3). (Notice that it is starting from one).
Implement methods to convert between the continuous world representation and your gridsystem. Each cell in you grid stores the belief of how probable that state is. 
Hence, the sum of the probabilities of each grid cell should be equal to 1. For initial conditions, you may assume a pointmass probability distribution i.e. the initial cell to have a probability of 1 and 0 everywhere else. 
Another possible initialization would be to initialize a gaussian around the initial cell with the initialcell being the mean.

### 2.Landmarks for this project-:
There are six landmarks in the robot map, and they are at the following locations: 
Tag 0: x=1.25m, y=5.25m
Tag 1: x=1.25m, y=3.25m 
Tag 2: x=1.25m, y=1.25m
Tag 3: x=4.25m, y=1.25m
Tag 4: x=4.25m, y=3.25m
Tag 5: x=4.25m, y=5.25mThe robot moves in the area within these landmarks, observing some at any given time.

### 3.Sensor and Motion Model 
The observation data consists of range, bearing and tag_num. 
The robot observes a specific tagidentified by the tag_num (0-5) where range is the distance between the specific landmark and therobot, and bearing is the angle between the landmark and the robot heading direction.

Figure 2:- Sensor and Motion pf Robot

You should utilize the odometry motion model for this project as the movement data isgiven by the relative odometry information described by the motion parameters rotation1,translation and rotation2.

### 4. Noise


Figure 3-: Gausian Distribution

In this project we make use of Gaussian Distribution (Normal Distribution) to add noise into Motion and Observation:
Two parameters: mean μ and standard deviation σ Given a data point x, we can get how “probable” the value is in the gaussian distribution fora given mean and standard deviation. 
For every control input, we can consider a single gaussian distribution for each of the three motion parameters (rotation1, translation, rotation2).
Similarly, for every measurement data, you can consider a gaussian each for range and bearing.
You can implement this function with your own code or use a library function. (For Python you can use the numpy module).In Grid Localization, for the purpose of moving robot between cells, the motion and observation noise should be adjusted accordingly. You need a translation and rotation noise for movement,and range and bearing noise for your observation. A good choice for this purpose is half the cellsize. So for the example of 20×20 cells and 90 degree discretization, range and translation noises are 10cm, and bearing and rotation noises are 45 degrees. You will need to tweak the noise parameters to improve your state estimation. 
Usually measurement data is “less noisier” thanmovement data.

### 5.Bayes Filter 
Essentially, every iteration of the bayes filter has two steps:
1. Prediction Step - where you incorporate the control input(movement)
2. Update Step​ - where you incorporate the observationA pseudo algorithm for the Bayes filter is described in Algorithm 
a. Prediction Step (Lines 9-11)increases uncertainty in your belief while the update step (Lines 2-8) reduces uncertainty in yourbelief. 
Ideally, we would want the observation to be the last message at the end of our entire stateestimation process, as it aims to reduce uncertainty in our robot’s belief.

Figure 4-: Algorithm

