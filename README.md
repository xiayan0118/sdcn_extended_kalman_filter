# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. `mkdir build`
2. `cd build`
3. `cmake ..`
4. `make`
5. `./ExtendedKF`

Tips for setting up your environment can be found in the classroom lesson for this project.

**INPUT**: values provided by the simulator to the c++ program

```
["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)
```

**OUTPUT**: values provided by the c++ program to the simulator

```
["estimate_x"] <= kalman filter estimated position x

["estimate_y"] <= kalman filter estimated position y

["rmse_x"]

["rmse_y"]

["rmse_vx"]

["rmse_vy"]
```

---

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Project Rubric

[//]: # (Image References)

[image1]: ./images/dataset_1.png "Dataset 1"
[image2]: ./images/dataset_2.png "Dataset 1"

#### 1. Your code should compile.

Code compiles without errors with `cmake` and `make`. The project can be built using the sequence of commands in previous section.

The compiled executable is functional in the simulator, as shown in the two screenshots below.

##### Dataset 1
![alt text][image1]

##### Dataset 2
![alt text][image2]

#### 2. `px, py, vx, vy` output coordinates must have `RMSE <= [.11, .11, 0.52, 0.52]` when using the file: `obj_pose-laser-radar-synthetic-input.txt`

As shown in the screenshots above:

* Dataset 1: `RMSE = [0.0973, 0.0855, 0.4513, 0.4399]`
* Dataset 2: `RMSE = [0.0726, 0.0967, 0.4579, 0.4966]`



