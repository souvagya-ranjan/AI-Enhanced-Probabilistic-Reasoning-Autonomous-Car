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
2. Code Package: The files you need to modify are estimator.py and intelligentDriver.py. 
```yaml
A3
├── layouts # The layouts of different environments. Prefix 'm_' implies a 'multiple goals' version of the corresponding layout.
│   ├── small.json   
│   ├── lombard.json 
│   ├── m_small.json 
│   └── m_lombard.json 
├── learned # The transition probabilities for each layout. Layout 'x' and 'm_x' have the same transition probabilites.
│   ├── smallTransProb.p  
│   └── lombardTransProb.p  
├── engine # You may want to explore this directory. 
├── environment.yml  # Conda environment file for Linux.
├── environment_win.yml  # Conda environment file for Windows.
├── environment_mac.yml  # Conda environment file for Mac.
├── drive.py  # Driver code to run and visualize your implementations.
├── none.py # Code for no inference.
├── autoDriver.py  # A baseline naive driver.
├── estimator.py  # The file where you place your estimation (or tracking) implementation.
    color : "#FF0000"
├── intelligentDriver.py  # The file where you have to implement your planning approach.
    color : "#FF0000"
└── util.py  # Code for utilities needed in estimator.py
```