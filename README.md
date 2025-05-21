# Advanced Dual Track Model

### Overview
Advanced Dual Track Model is a vehicle dynamics simulation. The vehicle is modeled as a multi-body-system with 40 states. TMeasy is used as the tire model. 
Double-Wishbone suspension kinematics are modeled. It is possible to run simulation scenarios on 3D road surfaces. 
Tire and suspension behaviour can be simplified to linear models. 

### People involved

##### Developer
[Simon Frank](mailto:simon.sf.frank@tum.de)

# Workflow
All files required to run simulations in MATLAB are contained in the "MATLAB" folder. 

### Start a simulation
* Run "AdvancedDualTrackModel.m"
* Select an input file containing control inputs for the simulated vehicle
* Select a 3D road file or skip this step to use a flat road instead (recommended for general use). 
* The results of the simulation are plotted automatically after it has finished
* Use "s_saveCSV.m" to save the results from the MATLAB workspace to a ".csv" file

### Input files
Input files must be ".csv" files containing at least the following signals (note capitalization, see "s_inputCreation.m"): 
* "Time" (in s)
* "delta_D" (in rad)
* "p_BFA" (in bar)
* "p_BRA" (in bar)
* "throttle" (normalized)
* "gear" (no unit)
* "mu_N" (normalized)

Other signals such as "a_x" or "a_y" can be included for validation. 

### 3D road files
Road files must be ".mat" files containing the following data (see "s_roadCreation.m"): 
* "x" (Vector of road X coordinates in 0.1m steps)
* "y" (Vector of road Z coordinates in 0.1m steps)
* "Z" (Matrix of road Z heights in 1m steps)
* "Mu" (Matrix of normalized road mu in -)
