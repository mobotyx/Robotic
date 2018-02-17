# Search and Sample Return Project

This project is modeled after the [NASA sample return challenge](https://www.nasa.gov/directorates/spacetech/centennial_challenges/sample_return_robot/index.html) and is part of the first Project to submit for the Udacity Robot Engineer nanodegree.

This repository contains a jupyter notebook under code directory, as well the code used to drive the rover simulator.

## Running the simulation

### 1. Notebook Used To Implement and Test the Algorithms

To run the code, Python3, anaconda and Jupyter Notebooks needs to be installed. Refer to the official anaconda website for the required steps, or follow this: [Anaconda and Jupyter Notebooks](https://classroom.udacity.com/courses/ud1111). Once done, create a new anaconda environment called RoboND

Clone this repository into your local machine and navigate inside the code directory, then activate the environment (Note that this code is based on `RoboND-Python-Starterkit` ( https://github.com/ryan-keenan/RoboND-Python-Starterkit )
```sh
source activate RoboND
```
open jupyter notebook 

```sh
jupyter notebook
```
The notebook to look into is called `Rover_Project_Notebook.ipynb` , it contains all the developed functions and associated explanations. These function are used later on in the rover control code.

Run the full notebook and inspect the results at each step. 

### 2. Download The Simulator 

To be able to see the results in a simulation environment, you need to install Roversim simulator.

The simulator is based on Unity game engine and is developed by Udacity for the Robotics Engineer nanodegree. Go ahead and download the simulator: [Linux Roversim!](https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Linux_Roversim.zip)

### 3. Inspect and Run the Code 

Always under the code folder, you find the following python files:

`drive_rover.py` : contains the rover class that keeps track of its state at all times. It also contains some utility code to connect with the the simulator and exchange data as well as the main entry point.

`perception.py` : functions coded in jupyter notebook are found here. They are basically used by the _decision_ code to understand the environment around the rover through processed image data.

`decision.py`: contains commands to drive or stop the rover as well as a decision tree that implements a navigation logic and a sample collection logic.

`supporting_functions.py` : helper/untility functions

To run the code and see the results, first run the simulator from your `Linux Roversim` install directory and select _Autonomous mode_

From a terminal (make sure RoboND is still activated), run the following:

```sh
python drive_rover.py
```
Once the socket connection is established, you'll see the rover driving around.

Note: when you kill the python code and run it again, the simulator wont connect. One solution for this issue is to run the following command from a terminal

```sh
killall -s 9 python 
```
This will kill any python running process and make sure that the socket connection is freed. you can then run again the driver_rover.

## Obstacle avoidance and Decision Tree

In order to control the rover, we use a simple decision tree implemented in the `decision_step` function under `decision.py` 

### Decision Tree Overview

The `Rover` object is passed to this function in order to get access to the full state such orientation and throttle and brake settings.

The rover navigation logic implements multiple states 

* _forward_ : when in this state, the rover will throttle up to drive forward until it reaches a predetermined speed. This speed can either be its max velocity `Rover.max_vel` or a reduced speed `Rover.slow_vel`. Both variables are fixed.

* _stop_ : when in this sate, the rover will decelerate using its brake until it stops 

* _align_: when a rock is detected and we wish to collect it, we set the driving mode to this state. The rover will kick-in an align logic based on the polar coordinate of the rock with reference to the rover coordinates frame.

* _collect_: this state is always activated from the _align_ state. When the _align_ step determine that the rover is rhoughtly aligned with the rock, it will switch the mode to _collect_ in order to make the final approach to pick-up the rock.

Depending on the given state, the following two function can be called 

`drive_rover` : It will accelerate the rover until reaching the velocity set point, another parameter that is passed is the steering command. Note that if we wish to decelerate without hitting the brakes, we just set a slower speed and the throttle will be reversed until reaching the new setpoint.

`stop_rover`: straight forward function, we just zero the throttle and hit the brakes given a brake force given as parameter.

### Obstacle Avoidance 

Given the 
![GitHub Logo](/misc/safety_box.png)





## Navigating Autonomously
The file called `drive_rover.py` is what you will use to navigate the environment in autonomous mode.  This script calls functions from within `perception.py` and `decision.py`.  The functions defined in the IPython notebook are all included in`perception.py` and it's your job to fill in the function called `perception_step()` with the appropriate processing steps and update the rover map. `decision.py` includes another function called `decision_step()`, which includes an example of a conditional statement you could use to navigate autonomously.  Here you should implement other conditionals to make driving decisions based on the rover's state and the results of the `perception_step()` analysis.

`drive_rover.py` should work as is if you have all the required Python packages installed. Call it at the command line like this: 

```sh
python drive_rover.py
```  

Then launch the simulator and choose "Autonomous Mode".  The rover should drive itself now!  It doesn't drive that well yet, but it's your job to make it better!  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results!  Make a note of your simulator settings in your writeup when you submit the project.**

### Project Walkthrough
If you're struggling to get started on this project, or just want some help getting your code up to the minimum standards for a passing submission, we've recorded a walkthrough of the basic implementation for you but **spoiler alert: this [Project Walkthrough Video](https://www.youtube.com/watch?v=oJA6QHDPdQw) contains a basic solution to the project!**.


