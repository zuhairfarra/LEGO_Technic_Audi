# LEGO_Technic_Audi
 
This repository contains exploratory work performed using the LEGO Audi Technic kit. The repo includes the following:

Pybricks Scripts: Python scripts from Pybricks to ran on the Technic Hub
Python Logging Scripts: Python scripts to be ran locally for logging data from the Technic hub
Matlab Files: Post processing scripts to be ran in Matlab for data plotting and parameter estimation
Components: Model(s) of kit components in Simulink, not runnable models
Models: Model(s) of kit assembly or test harnesses of component models, runnable models

Workflow for this repo, from running test on hub to comparing model to test data is as follows:

1. Design test and run script on the Technic hub
2. Read data from the hub using a logging script from a local machine and store in a file
3. Import data into a Matlab instance
4. Plot and/or use physics based models to estimate parameters of the kit from the test
5. Use estimated parameters in the Simulink model and simulate the test

For example, using data (from the Data folder) and a process_.m script can be used to post process the data, the estimated parameters can be used to then run in a run_.m file which runs a simulink model with the required parametrization.

In the data folder, there are workspace files for motor parameters of different motor units used in data collections, labeled M1, M2, and M3.
