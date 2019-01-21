# Extended Kalman Filter Project Writeup

---

[//]: # (Image References)

[image1]: ./images/dataset1_rmse.png "Dataset 1"
[image2]: ./images/dataset2_rmse.png "Dataset 1"

---

## Starter code
* [Udacity repo] (https://github.com/udacity/CarND-Extended-Kalman-Filter-Project)

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

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Rubrics

### Accuracy
1. px, py, vx, vy output coordinates must have an RMSE <= [.11, .11, 0.52, 0.52] when using the file: "obj_pose-laser-radar-synthetic-input.txt" which is the same data file the simulator uses for Dataset 1.

* The RMSE of dataset #1 at tep 499 is in range [.11, .11, .52, .52]:

    | Variable | RMSE   |
    |----------|--------|
    | px       | 0.0973 |
    | py       | 0.0855 |
    | vx       | 0.4513 |
    | vy       | 0.4399 |

![Screenshot][image1]


* The RMSE of dataset #2 at step 498 is also in the range:

    | Variable | RMSE   |
    |----------|--------|
    | px       | 0.0726 |
    | py       | 0.0965 |
    | vx       | 0.4216 |
    | vy       | 0.4932 |

![Screenshot][image2]


### Follows the Correct Algorithm
1. Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.
Yes it follows the steps introduced from the course.

2. Your Kalman Filter algorithm handles the first measurements appropriately.
Yes in `FusionEKF.cpp`, if the filter is not initialized yet, it will only update the state and timestmap, according to the types of sensor.


3. Your Kalman Filter algorithm first predicts then updates.
Yes in `FusionEKF.cpp` function `ProcessMeasurement()`, which calls `Predict()` then `Update()` or `UpdateEKF()`.

4. Your Kalman Filter can handle radar and lidar measurements.
Yes the filter will apply different state-to-measurement (H) and measure covaraince matrix (R) for different types of sensor.


