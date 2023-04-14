# Probabilistic-Reasoning-AI-enhanced-car 
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

conda activate assign3 # Run the environment
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
4. Simulator: The simulator is provided in the engine directory. You can run the simulator by running the following command: 
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