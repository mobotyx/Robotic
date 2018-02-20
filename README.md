
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


## Perception Of the Environment 

In order to understand the navigable environment vs obstacle regions, the Rover is equipped with a camera mounted in its front and always looking forward. The logic in relation to turning the camera image into actionable data is implemented in `perception.py`, the following are roughly the steps as implemented in`process_image()`from the notebook and copied over to `perception_step()` in the simulation code.

### 1. Warping the Image / Perspective Transform

In order to relate the pixel data to Cartesian coordinates, the first step in the pipeline is to transform the image the the rover is seeing as if it was looking from the top downward into the FOV or the camera. This is accomplished by using two OpenCV functions : `cv2.getPerspectiveTransform `and `cv2.warpPerspective`. (click [here](https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_geometric_transformations/py_geometric_transformations.html) for more information about the perspective transform)
```python
	warped, mask = perspect_transform(Rover.img, source, destination)
```
as in the code snippet, we also return a mask which represent a binary representation of the FOV of the camera.

### 2. Navigable terrain vs Obstacle 

Given the warped image, a color threshold is applied. This is performed inside `color_thresh` which basically use an RGB threshold as a condition, applied to the original image and returning a binary array corresponding to navigable pixel or obstacle pixel. In our case, an RGB threshold of (160, 160, 160) seems to be doing well enougth in detecting sand colors and considering them as navigable terrain.

```python
    col_thres = color_thresh(warped)
    obs_map   = np.absolute(np.float32(col_thres) - 1) * mask 
```
The obstacle map is then determined by negating the color threshold data and applying the mask. 

### 3. Rock/Samples Identification  

Samples are observed to be yellowish, so in order to detect them, another function that rely on the same color threshold technique as described above is implemented, but this time to keep only the yellow sample data. 
 ```python
 def find_rocks(img, levels=(110,110,50)):
    rock_pix = (img[:,:,0] > levels[0]) & (img[:,:,1] > levels[1])  & (img[:,:,2] < levels[2])
    color_select = np.zeros_like(img[:,:,0])
    color_select[rock_pix] = 1
    return color_select
```
And called as follows (note that we use the warped image before color thresholding it)
 ```python
    rock_map = find_rocks(warped, levels=(110,110,50))

```
One final step as far as rocks are concerned is to precisely map them in the world map. To do this, we need to select only one pixel (closest pixel) from the rock_map array and map it. this is performed as follows: 

 ```python
	 rock_x, rock_y = rover_coords(rock_map)
	 rock_x_world, rock_y_world = pix_to_world(rock_x, rock_y, xpos, ypos, yaw, world_size, scale)
	 rock_dist, rock_ang = to_polar_coords(rock_x, rock_y)
     rock_idx = np.argmin(rock_dist)
     rock_xcen = rock_x_world[rock_idx]
     rock_ycen = rock_y_world[rock_idx]

```
### 4. Mapping to the World Map 

In the simulation code, the world map is displayed in the bottom right corner of the screen. We map the navigable terrain, the obstacles and the rocks/samples to this map in different colors. Red for obstacle, Blue for Navigable terrain and white for the detected rocks/samples.

For the obstacle and navigable terrain, this is performed as follows 

 ```python
    # Convert from rover to world coordinates
    x_world, y_world = pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale)
    x_obs_world, y_obs_world = pix_to_world(xobs, yobs, xpos, ypos, yaw, world_size, scale)
      
    # Map to World 
    Rover.worldmap[y_world, x_world, 2] = 255    
    Rover.worldmap[y_obs_world, x_obs_world, 0] = 255   
    nav_pix = Rover.worldmap[:,:,2] > 0
    Rover.worldmap[nav_pix,0] = 0

```
`pix_to_world` is a standard transformation for a given pixel point (array of points) from the rover coordinates to the world coordinates, given the rover X,Y position and Yaw angle as well as a scaling parameter to normalize to the world map.

`Rover.worldmap` : this is the output image that is generated and fed to the simulator. We use the Red (0) and Blue (2) for navigable terrain and obstacles, respectively 

For rock mapping, it is done in a similar fashion by using the centric coordinates as described above: 

 ```python
    Rover.worldmap[rock_ycen, rock_xcen, 1] = 255
```

## Obstacle avoidance and Decision Tree

In order to control the rover, we use a simple decision tree implemented in the `decision_step` function under `decision.py` 

### Decision Tree Overview

The `Rover` object is passed to this function in order to get access to the full state such orientation and throttle and brake settings.

The rover navigation logic implements multiple states 

* _forward_ : when in this state, the rover will throttle up to drive forward until it reaches a predetermined speed. This speed can either be its max velocity `Rover.max_vel` or a reduced speed `Rover.slow_vel`. Both variables are fixed.

* _stop_ : when in this sate, the rover will decelerate using its brake until it stops 

* _align_: when a rock is detected and we wish to collect it, we set the driving mode to this state. The rover will kick-in an align logic based on the polar coordinate of the rock with reference to the rover coordinates frame.

* _collect_: this state is always activated from the _align_ state. When the _align_ step determine that the rover is roughly aligned with the rock, it will switch the mode to _collect_ in order to make the final approach to pick-up the rock.

Depending on the given state, the following two function can be called 

`drive_rover` : It will accelerate the rover until reaching the velocity set point, another parameter that is passed is the steering command. Note that if we wish to decelerate without hitting the brakes, we just set a slower speed and the throttle will be reversed until reaching the new setpoint.

`stop_rover`: straight forward function, we just zero the throttle and hit the brakes given a brake force given as parameter.

### Obstacle Avoidance 

In the perception file, we determine the navigable terrain vs obstacles. This data is fed to the decision tree in the form of lists of x,y coordinates of all the navigable pixels in the rover coordinate frame.

To avoid obstacles, a simple concept have been implemented. As illustrated in the image below, we determine a simple box just in front of the rover. It represent a safety zone. We calculate the amount of navigable pixels as a ratio to the total number of pixels in the box. 


For instance, if the box is 1 meter large by 1 meter long, then we can have a total amount of 10x10 = 100 pixels inside the box (see notebook for explanation about pix to meter conversion)

Given a ratio parameter, let's say 70%, when the navigable pixels is more than 70% of the total number of possible pixels, we drive forward, otherwise we initiate a stop. 

One rule of thumb to determine the box size is to give enough space each side that the rover fits in and a distance forward that depend on the rover braking capability  

![GitHub Logo](/misc/safety_box.png)
  
### Fast and Low Speed 

As described earlier, the rover can drive two speeds, max velocity or reduced speed. The speed setting is determined in the following code snippet 

```python
if (abs(nav_angles_deg) < Rover.nav_ang_thres and np.mean(Rover.nav_dists) > Rover.nav_dis_thres):
	drive_rover(Rover, Rover.max_vel, nav_angles_deg)
    # Otherwise, reduce speed
else:
	drive_rover(Rover, Rover.slow_vel, nav_angles_deg)
```

Basically, as long as the rover is in the _forward_ mode, we monitor how wide or narrow the navigable terrain is and how much distance we have ahead. 

This logic compromise between speed and braking efficiency. when we are closing-in to an obstacle, we would decelerate first to a reduced speed then apply a smoother brake. 

This improvement has also an effect in the image stability. The more the rover brakes hard from a high speed, higher is the pitch transition amplitude during the brake. And if the rover is close enough to an obstacle, the pitch would induce errors in mapping and the navigable terrain at the specific position where the brake happened would overshoot (camera pitching with the rover)

Note : A further improvement of this logic is to add more speed levels or deceleration zones.

## Further Improvements

Some limitation are observed during the test runs and there is room for improvement

1. __Deceleration zones__ : Adding more deceleration set points given the navigable terrain forward would make the final braking smoother, thus smaller transitional pitch amplitude.

2. __Collection samples__ : The current logic for samples collection assume that when the mode switch to _align_ and _collect_ , no obstacle is between the rover and the sample based on the fact that if we see it, we can reach it. But this has one limitation: if there is no space in each side where the rover can fit in, the operation would result in the rover hitting an obstacle in its right or left side. This behavior is observed when a rock is placed in a region almost surrounded with obstacles, but still a free line of sight in relation to the rover.

3.  __Steering___ : The steering angle is hard-coded to -15 when the rover has to turn close to an obstacle. in some case, it is best to turn the other way. An improvement would be to keep a short history of the navigable terrain and when the rover initiate the brake, it will revisit this history to determine how the terrain was behind and decide either it will turn left or right.
