# Self-Driving Car Technology - Unscented Kalman Filters

## Overview

While cameras are incredibly useful for autonomous vehicles (such as in [traffic sign detection and classification](https://github.com/dinoboy197/CarND-Traffic-Sign-Classifier-Project)), other sensors also provide invaluable data for creating a complete picture of the environment around a vehicle. Because different sensors have different capabilities, using multiple sensors simultaneously can yield a more accurate picture of the world than using one sensor alone. **[Sensor fusion](https://en.wikipedia.org/wiki/Sensor_fusion)** is this technique of combining data from multiple sensors. Two frequently used non-camera-based sensor technologies used in autonomous vehicles are [radar](https://en.wikipedia.org/wiki/Radar) and [lidar](https://en.wikipedia.org/wiki/Lidar), which have different strengths and weaknesses and produce very different output data.

Radars exist in many different forms in a vehicle: for use in adaptive cruise control, blind spot warning, collision warning, collision avoidance, to name a few. Radar uses the [Doppler effect](https://en.wikipedia.org/wiki/Doppler_effect) to measure speed and angle to a target. Radar waves reflect off hard surfaces, but can pass through fog and rain, have a wide field of view and a long range. However, their resolution is poor compared to lidar or cameras, and they can pick up radar clutter from small objects with high reflectivity. Lidars use [infrared](https://en.wikipedia.org/wiki/Infrared) laser beams to determine the exact location of a target. Lidar data resolution is higher than radar, but lidars cannot measure the velocity of objects, and they are more adversely affected by fog and rain.

![Lidar point cloud][lidar]

**Example of lidar point cloud**

![Radar location and velocity][radar]

**Example of radar location and velocity data**

A very popular mechanism for object tracking in autonomous vehicles is a [Kalman filter](https://en.wikipedia.org/wiki/Kalman_filter). Kalman filters uses multiple imprecise measurements over time to estimate locations of objects, which is more accurate than single measurements alone. This is possible by iterating between estimating the object state and uncertainty, and updating the state and uncertainty with a new weighted measurement, with more weight being given to measurements with higher precision. This works great for linear models, but for non-linear models (such as those which involve a turning car, a bicycle, or a radar measurement), the [extended Kalman filter](https://github.com/dinoboy197/CarND-Extended-Kalman-Filter-Project) must be used. Extended Kalman filters are considered the standard for many autonomous vehicle tracking systems.

Unfortunately, when the models are very non-linear, the extended Kalman filter can yield very poor performance. As a remedy, the [unscented Kalman filter](https://en.wikipedia.org/wiki/Unscented_Kalman_filter) can be used.

![Kalman probability][kalman-prob]

This repository contains an unscented Kalman filter implementation which estimates the location and velocity of a vehicle with noisy radar and lidar measurements.

### Technologies Used

* C++
* uWebSockets
* Eigen

[//]: # (Image References)

[tracking]: ./tracking.jpg "Tracking"
[lidar]: ./lidar.jpg "Lidar"
[radar]: ./radar.jpg "Radar"
[kalman-algo]: ./kalman-algorithm.jpg
[kalman-prob]: ./kalman-prob.jpg

### Repository Contents

* [README.md](README.md) - this readme
* [main.cpp](src/main.cpp) - program entry point; communicates with simulator, executes unscented Kalman filter and computes RMSE
* [ukf.cpp](src/ukf.cpp) - sensor fusion and unscented Kalman filter module
* [tools.cpp](src/tools.cpp) - RMSE mathematical utility method
* [video.mp4](video.mp4) - video of predicted location of vehicle in simulator based on unscented Kalman filtering

## Running code from this repository

Running the code in this repository requires the Udacity Term 2 Simulator contents to be properly installed. Click [here](https://github.com/udacity/self-driving-car-sim/releases) for details. This will include installing the simulator itself and the uWebSocketIO library.

Once that is complete,
```sh
mkdir build && cd build
cmake .. && make
./UnscentedKF
# in a separate terminal, start the simulator
```

## Kalman filters

Conceptually, Kalman filters work on the assumption that the actual or "true" state of the object (for instance, location and velocity) and the "true" uncertainty of the state may not be knowable. Rather, by using previous estimates about the object's state, knowledge of how the object's state changes (what direction it is headed and how fast), uncertainty about the movement of the vehicle, the measurement from sensors, and uncertainty about sensor measurements, an estimate of the object's state can be computed which is more accurate than just assuming the values from raw sensor measurements.

![Kalman algorithm][kalman-algo]

With each new measurement, the "true state" of the object is "predicted". This happens by applying knowledge about the velocity of the object as of the previous time step to the state. Because there is some uncertainty about the speed and direction that the object traveled since the last timestep (maybe the vehicle turned a corner or slowed down), we add some "noise" to the "true state".

Next, the "true state" just predicted is modified based on the sensor measurement with an "update". First, the sensor data is compared against belief about the true state. Next, the "Kalman filter gain", which is the combination of the uncertainty about the predicted state and the uncertainty of the sensor measurement, is applied to the result, which updates the "true state" to the final belief of the state of the object.

### Unscented Kalman filters

Because the motion of the object may not be linear; for example, if it accelerates around a curve, or slows down as it exits a highway, or swerves to avoid a bicycle, the model of the object motion may be non-linear. Similarly, the mapping of the sensor observation from the true state space into the observation space may be non-linear, which is the case for radar measurements, which must map distance and angle into location and velocity. The extended Kalman filter can often be used here when the non-linear motion or observation models are easily differentiable and can be approximated by a linear functions.

However, in some cases the motion or observation models are either not easily differentiable or cannot be easily approximated by linear functions. For example, many motion models use non-constant velocity; often they include both straight lines *and* turns. Even if these functions could be differentiated properly, the probability distributions of the uncertainty in the models might **not** be normally distributed after a "predict" or "update" step, which breaks the Kalman filter assumptions. To avoid this problem, using an unscented Kalman filter can be used, which involves transforming point samples from a Gaussian distribution through the non-linear models to approximate the final probability distribution. This has two main advantages:

* the sample points often approximate the probability distribution of the non-linear functions better than a linear model does
* a Jacobian matrix is not necessary to compute

In order to preserve the "normal" nature of the motion and observation probability distributions, the unscented Kalman filter uses the "unscented transformation" to map a set of points through the motion and observation models. To do this, a group of "sigma points" are selected which represent the original state probability distribution. Once transformed by the non-linear model, the mean and per-dimensional standard deviations of the transformed points are computed, which yields a useful approximation of the real mean and standard deviation of the non-linear model.

Other than using sigma points to represent the motion and observation model probability functions, the unscented Kalman filter flow is similar to that of the extended Kalman filter, including a measurement-model independent "predict" step, followed by a measurement-model dependent "update" step. 

## Implementation

Position estimation occurs in a [main loop](https://github.com/dinoboy197/CarND-Unscented-Kalman-Filter-Project/blob/master/src/main.cpp#L41-L151) inside the program. First, the program waits for the next sensor measurement, either from the lidar or radar. [Lidar sensor measurements contain the absolute position of the vehicle](https://github.com/dinoboy197/CarND-Unscented-Kalman-Filter-Project/blob/master/src/main.cpp#L70-L78), while [radar measurements contain the distance to the vehicle, angle to the vehicle, and speed of the vehicle along the angle](https://github.com/dinoboy197/CarND-Unscented-Kalman-Filter-Project/blob/master/src/main.cpp#L81-L91). The measurements are [passed through the sensor fusion module](https://github.com/dinoboy197/CarND-Unscented-Kalman-Filter-Project/blob/master/src/main.cpp#L109) (which uses a Kalman filter for position estimation), then the [new position estimation is compared to ground truth](https://github.com/dinoboy197/CarND-Unscented-Kalman-Filter-Project/blob/master/src/main.cpp#L130) for performance evaluation.

The unscented Kalman filter module encapsulates the core logic for processing a new sensor measurement and updating the estimated state (location and velocity) of the vehicle. Upon initialization, [the lidar and radar measurement uncertainties are initialized](https://github.com/dinoboy197/CarND-Unscented-Kalman-Filter-Project/blob/master/src/ukf.cpp#L43-L56) (which would be provided by the sensor manufacturers), as well as [the motion model uncertainty](https://github.com/dinoboy197/CarND-Unscented-Kalman-Filter-Project/blob/master/src/ukf.cpp#L37-L41). Upon receiving a first measurement, the [initial state is set to the raw measurement from the sensor](https://github.com/dinoboy197/CarND-Unscented-Kalman-Filter-Project/blob/master/src/ukf.cpp#L90-L112) (using a conversion in the case of the first measurement being from a radar). Next, [the "predict" step is run](https://github.com/dinoboy197/CarND-Unscented-Kalman-Filter-Project/blob/master/src/ukf.cpp#L134). Finally, [the "update" step is run](https://github.com/dinoboy197/CarND-Unscented-Kalman-Filter-Project/blob/master/src/ukf.cpp#L137-L141).

The "predict" step has three main functions:
* [generating sigma points](https://github.com/dinoboy197/CarND-Unscented-Kalman-Filter-Project/blob/master/src/ukf.cpp#L137-L141)
* [predicting the sigma points through the motion model transform](https://github.com/dinoboy197/CarND-Unscented-Kalman-Filter-Project/blob/master/src/ukf.cpp#L183-L227)
* [predicting the new state mean and covariances](https://github.com/dinoboy197/CarND-Unscented-Kalman-Filter-Project/blob/master/src/ukf.cpp#L232-L249)

There are two different "update" implementations; one for [radar](https://github.com/dinoboy197/CarND-Unscented-Kalman-Filter-Project/blob/master/src/ukf.cpp#L279-L297) and one for [lidar](https://github.com/dinoboy197/CarND-Unscented-Kalman-Filter-Project/blob/master/src/ukf.cpp#L254-L271). Both implementations must first transform the predicted sigma points into the measurement space ([lidar](https://github.com/dinoboy197/CarND-Unscented-Kalman-Filter-Project/blob/master/src/ukf.cpp#L347-L374), [radar](https://github.com/dinoboy197/CarND-Unscented-Kalman-Filter-Project/blob/master/src/ukf.cpp#L347-L374)), then [update the predicted state based on the transformed measurements](https://github.com/dinoboy197/CarND-Unscented-Kalman-Filter-Project/blob/master/src/ukf.cpp#L381-L419).

## Performance

To test the sensor fusion using the unscented Kalman filter, the simulator provides raw lidar and radar measurements of the vehicle through time. Raw lidar measurements are drawn as red circles, raw radar measurements (transformed into absolute world coordinates) are drawn as blue circles with an arrow pointing in the direction of the observed angle, and sensor fusion vehicle location markers are green triangles.

![Kalman filter tracking][tracking]

[Click here to view a video of the tracking](video.mp4).

Clearly, the green position estimation based on the sensor fusion tracks very closely to the vehicle location, even with imprecise measurements from the lidar and radar inputs (both by visual inspection and by the low RMSE.)

Additionally, by comparison with the extended Kalman filter on the same input dataset, it is clear that the error in the estimated positions from the unscented Kalman filter are lower than that of the extended Kalman filter:


|           | Extended        | Unscented     | Improvement in Error  |
|:---------:|:---------------:|:-------------:|:------------:|
|   PX      | 0.0974          | 0.0756        |   22%        |
|   PY      | 0.0855          | 0.0849        |    1%        |
|   VX      | 0.4517          | 0.3530        |   22%        |
|   VY      | 0.4404          | 0.2428        |   45%        |
