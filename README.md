[//]: # (Image References)
[image1]: ./img/Setup.PNG "Setup"
[image2]: ./img/Dataset_1.PNG "Dataset 1"
[image3]: ./img/Dataset_2.PNG "Dataset 2"

# Extended Kalman Filter
Self-Driving Car Engineer Nanodegree Program

This project utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.

This project involves Udacity Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

**INPUT**: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)

**OUTPUT**: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x

["estimate_y"] <= kalman filter estimated position y

["rmse_x"]

["rmse_y"]

["rmse_vx"]

["rmse_vy"]

## Simulator

In the Simulator, Lidar measurements are red circles, radar measurements are blue circles with an arrow pointing in the direction of the observed angle, and estimation markers are green triangles. The simulator provides the script the measured data (either lidar or radar), and the script feeds back the measured estimation marker, and RMSE values from its Kalman filter.

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Running the Filter

1. Start the Simulator and select the first project
2. From the build directory, run the Extended Kalman Filter program: `./ExtendedKF `
3. Select the dataset in the Simulator and click "Start":

![alt text][image1]

4. If the Filter is working fine, it is expected to achieve a final RMSE (Root Mean Square Error) of [.11, .11, 0.52, 0.52] for px, py, vx and vy.
5. Here an example of the first Dataset, where the LIDAR data is sent first:

![alt text][image2]

6. And here an example of the second Dataset, where the RADAR data is sent first:

![alt text][image3]
