# Probabilistic-Reasoning-AI-enhanced-car 

## Introduction
This assignment concerns probabilistic reasoning to estimate a quantity of interest using noisy observations acquired over time. We consider the example of an intelligent road vehicle equipped with a (noisy) sensor that needs to locate other vehicles in order to plan a safe path to its goal. 

In this assignment, you will implement a probabilistic reasoning approach to estimate the locations of StdCars in the environment. You will also implement an intelligent driver that plans the path of the AutoCar to reach the goal(s) in the environment.

## Domain Description
Consider a grid world with an autonomous car (with your software in its computer) moving in the presence of other cars on the road. The autonomous car (now referred to as AutoCar) needs to know the location of other cars (referred to as StdCar) in order to plan its path safely without collisions with other cars. The domain is described as follows:

Figure: The simulated 2D environment used in this assignment. The AutoCar (leftmost car) estimates a belief (multiple colours) over the positions of other 3 StdCars. 

### Environment 
The environment is a 2D grid world with discrete grid cells. The grid can have cells which are occupied with obstacles. Along with the AutoCar, there can be $K$ StdCars present in the environment. The cars can move from one grid cell to another grid cell that is unoccupied. Some cars may simply be static the whole time. 

### Noisy Sensor
The AutoCar is equipped with a microphone-like sensor that measures the *loudness* of another car in the vicinity. The loudness recorded in the microphone can be interpreted as a  measurement of the relative distance between the AutoCar and a StdCar. 

The sensor measurements are noisy. Let  $z_t$ denote the measured distance. The distance is normally distributed as $z_t = \mathcal{N}(\vert \vert y_t - x_t \vert \vert_2, \sigma^{2})$, where $y_t$ and $x_t$ denote the true positions of the AutoCar and the StdCar in the grid respectively and $\sigma$ denotes the standard deviation. For example, if the relative distance is 5 grid cells then the sensor may measure the distance as 5.1, 4.0, 8.0 with decreasing probabilities.

### Transitions 
The motion of a StdCar(s) is also uncertain. When the car takes an action (i.e., movement from one grid cell to another cell) the car may not arrive at the intended grid cell. The transition probabilities for car movement are assumed to be known. 

Assume that the movement of the StdCar(s) is *independent* of each other. Note that in real road driving this assumption may *not* be true, two cars may be moving while taking the motion of each other into consideration, but such correlations can be ignored for now. The noise in the sensor measurement for each car is also assumed independent.

## Implementation 
### Estimation - Part I
Our goal is to implement a probabilistic tracker (running on the AutoCar) that estimates a probability distribution (a *belief*) over the possible location of StdCar(s) from sensor data collected over time. Formally, we want to estimate the belief as $p(x^k_t \vert z^k_0, z^k_1, \dots z^k_t)$, where  $x^k_t$ denotes the 2D location for the $k^{th}$ StdCar from the sequence of noisy observations $\{z^{k}_0, z^{k}_1, \dots z^{k}_t \}$ of the $k^{th}$ StdCar as received by the AutoCar’s sensor up to time $t$. Since, the estimated beliefs are probability distributions, they must be *normalised*. 

Please implement an algorithm (called *Estimator*) to estimate the belief over the position of each StdCar(s) from the collected measurement corresponding to that car. Analyse the state space, the transition and the observation models. 

- One approach is to use Exact Inference (standard Filtering in a time series model) to estimate the current position of the car. However, this approach may be too slow for a larger grids to be computed in a reasonable time.  An alternate approach is Particle Filtering. This approach will now only approximately represent the belief but has the advantage of better scalability.
- Please base your implementation on a Particle filter. Experiment with this approach on a few settings. You may notice that the particle filter may not perform well in some cases (e.g., when the StdCar is not moving). Based on your insights, improve your particle filter implementation for the given problem to arrive at your best solution.

Since the focus of this part is on tracking, assume that the code for driving the AutoCar is given. The code provided will drive the car collecting measurements of other cars. Your task will be to estimate the belief given the observations collected. Note that the driving code is not sophisticated and crashes (with StdCars or obstacles) may occur.

### Planning - Part II
We now turn our attention to driving the AutoCar making use of the tracking ability you developed in Part A. Assume that the AutoCar is provided a sequence of goal locations (grid cells) that must be visited (in order). Given the tracker developed in Part A, the AutoCar should have a good estimate of the positions of other cars. Our goal now is to use this information to plan a safe path without colliding with any of the other cars in the environment or the walls.  

- Please implement a planner (called *Intelligent Driver*) for safely driving the AutoCar. The planner will determine which grid cell the AutoCar should move into next. Please think about how the car can be kept safe while determining paths to the prescribed goal locations in the presence of uncertainty over the other cars. You may need to forecast the positions of other cars into the future as you decide the next action for the AutoCar. The AutoCar simply needs to decide the next grid cell it will move into. Once the car decides the grid cell it is aiming for, the car will turn and start moving towards the grid cell (motion will be simulated).
- A simple planner for driving the car is already provided as reference (See AutoDriver). The approach extracts a graph from the simulator and then determines a path that avoids collision by using the beliefs to detect the presence of other cars on its path. ****This driver is rather naive and does not necessarily attempt to visit all the goal locations. As you implement your Intelligent Driver you may refer to the provided code as a reference.

## Starter Code 
1. Clone the repository: 
```bash
git clone
cd Probabilistic-Reasoning-AI-enhanced-car
cd dir
```
2. Conda environment: Please configure a conda environment with the permitted assignment dependencies using the commands provided below.  
```bash
# Create the conda environment
conda env create -n assign --file environment.yml # For Linux 
conda env create -n assign --file environment_win.yml # For Windows
conda env create -n assign --file environment_mac.yml # For Mac

conda activate assign # Run the environment
```
3. Code Package: The files you need to modify are estimator.py and intelligentDriver.py. 
```yaml
A3
├── layouts # The layouts of different environments. Prefix 'm_' implies a 'multiple goals' version of the corresponding layout.
│   ├── small.json   
│   ├── lombard.json 
│   ├── m_small.json 
│   └── m_lombard.json 
├── learned # The transition probabilities for each layout. Layout 'x' and 'm_x' have the same transition probabilites.
│   ├── smallTransProb.p  
│   └── lombardTransProb.p  
├── engine # You may want to explore this directory. 
├── environment.yml  # Conda environment file for Linux.
├── environment_win.yml  # Conda environment file for Windows.
├── environment_mac.yml  # Conda environment file for Mac.
├── drive.py  # Driver code to run and visualize your implementations.
├── none.py # Code for no inference.
├── estimator.py: # The file where you place your estimation (or tracking) implementation.
├── intelligentDriver.py:  # The file where you have to implement your planning approach.
└── util.py  # Code for utilities needed in estimator.py
```
## Simulator: 
The simulator is provided in the engine directory. You can run the simulator by running the following command: 
```bash
python3 drive.py
```
| Key | Action |
| --- | --- |
| -a <autonomous or not> | Enabling autonomous driving (instead of manual). To drive manually, use the ‘w’, ‘a’, and ‘d’ keys. |
| -i <inference-method> | Use { “none”, “estimator” } to estimate the belief over the locations of StdCar(s). |
| -l <map> | The layout/map can be “small” or “lombard”.  |
| -k  | The number of StdCar(s) in the environment. |
| -m <multiple goals> | Multiple-goal version of the layout |
| -d <debug> | Debug mode where all cars are displayed on the map.  |
| -p <parked> | All StdCars remain parked (so that they don’t move).  |
| -j | To invoke your intelligent driver.  | 

Invoke the environment (without estimation) in the ‘small’ layout with 2 StdCars as follows:
```bash
    python3 drive.py -d -k 2 -l small -a -i none
```

The estimator implementation can be tested with the default driver ‘AutoDriver’ on the above environment as follows: 
```bash
    python3 drive.py -d -k 2 -l small -a -i estimator
```

The intelligent driver implementation can be tested on layouts with multiple goals as follows:
```bash
    python3 drive.py -d -k 2 -m -l small -a -i estimator -j
``` 
Note that the simulation automatically stops when the AutoCar crashes i.e., collides with a StdCar, hits the obstacles, or hits the boundary of the layout. After a crash, you can close the simulation window using the GUI or by pressing the ‘q’ key. 
