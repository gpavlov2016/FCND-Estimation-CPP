# Estimation Project #

In this project, the estimation portion of the controller used in the CPP simulator is implemented.  The simulated quad will be flying with estimator from this project and controller from the previous project.

## Setup ##

This project uses the C++ development environment.

 1. Clone the repository
 ```
 git clone https://github.com/gpavlov2016/FCND-Estimation-CPP
 ```

 2. Import the code into your IDE like done in the [Controls C++ project](https://github.com/udacity/FCND-Controls-CPP#development-environment-setup)
 
 3. You should now be able to compile and run the estimation simulator


### Project Structure ###

This project adds the Extended Kalman Filter implementation to the control part.

 - The EKF is implemented in `QuadEstimatorEKF.cpp`

 - Parameters for tuning the EKF are in the parameter file `QuadEstimatorEKF.txt`

 - When you turn on various sensors (the scenarios configure them, e.g. `Quad.Sensors += SimIMU, SimMag, SimGPS`), additional sensor plots will become available to see what the simulated sensors measure.

 - The EKF implementation exposes both the estimated state and a number of additional variables. In particular:

   - `Quad.Est.E.X` is the error in estimated X position from true value.  More generally, the variables in `<vehicle>.Est.E.*` are relative errors, though some are combined errors (e.g. MaxEuler).

   - `Quad.Est.S.X` is the estimated standard deviation of the X state (that is, the square root of the appropriate diagonal variable in the covariance matrix). More generally, the variables in `<vehicle>.Est.S.*` are standard deviations calculated from the estimator state covariance matrix.

   - `Quad.Est.D` contains miscellaneous additional debug variables useful in diagnosing the filter. You may or might not find these useful but they were helpful to us in verifying the filter and may give you some ideas if you hit a block.


#### `config` Directory ####

In the `config` directory, in addition to finding the configuration files for your controller and your estimator, you will also see configuration files for each of the simulations.  For this project, you will be working with simulations 06 through 11 and you may find it insightful to take a look at the configuration for the simulation.

As an example, if we look through the configuration file for scenario 07, we see the following parameters controlling the sensor:

```
# Sensors
Quad.Sensors = SimIMU
# use a perfect IMU
SimIMU.AccelStd = 0,0,0
SimIMU.GyroStd = 0,0,0
```

This configuration tells us that the simulator is only using an IMU and the sensor data will have no noise.  You will notice that for each simulator these parameters will change slightly as additional sensors are being used and the noise behavior of the sensors change.


## Implementation ##

Project outline:

 - [Sensor Noise Estimation](#step-1-sensor-noise)
 - [Attitude Estimation](#step-2-attitude-estimation)
 - [Prediction Step](#step-3-prediction-step)
 - [Magnetometer Update](#step-4-magnetometer-update)
 - [GPS Update](#step-5-closed-loop--gps-update)



### Sensor Noise Estimation ###

In this step, simulated noisy sensor data is collected and to estimate the standard deviation of the quad's sensors as follows:

Choose scenario `06_NoisySensors`.  The sensor data is collected on a static quad. You will see two plots at the bottom, one for GPS X position and one for The accelerometer's x measurement.  The dashed lines are a visualization of a single standard deviation from 0 for each signal. The standard deviations are initially set to arbitrary values (after processing the data in the next step, you will be adjusting these values).  If they were set correctly, we should see ~68% of the measurement points fall into the +/- 1 sigma bound.  When you run this scenario, the graphs you see will be recorded to the following csv files with headers: `config/log/Graph1.txt` (GPS X data) and `config/log/Graph2.txt` (Accelerometer X data).
The standard deviation of the sensor noise was calculated by openning the sensor logs in excel and applying the STDDEV function over all the sequence.
The result is plugged in into the top of `config/6_Sensornoise.txt`.  Specifically, the values for `MeasuredStdDev_GPSPosXY` and `MeasuredStdDev_AccelXY`.


### Attitude Estimation ###

In this step, the complementary filter-type attitude filter with a rate gyro attitude integration is implemented.

Run scenario `07_AttitudeEstimation`.  For this simulation, the only sensor used is the IMU and noise levels are set to 0 (see `config/07_AttitudeEstimation.txt` for all the settings for this simulation).  There are two plots visible in this simulation.
   - The top graph is showing errors in each of the estimated Euler angles.
   - The bottom shows the true Euler angles and the estimates.
In `QuadEstimatorEKF.cpp`, the function `UpdateFromIMU()` contains a complementary filter-type attitude filter. This is done by converting the current Euler angles of pitch, roll and yaw to quaternion representation and integrating the body fram gyro measurement with respect to the quaternion representation of the current attitude.

![attitude example](images/attitude-screenshot.png)

In the screenshot above the attitude estimation using linear scheme (left) and using the improved nonlinear scheme (right). Note that Y axis on error is much greater on left.

The math for the integration can be seen in section 7.1.2 of [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj)


### Prediction Step ###

In this step the prediction step of the filter is implemented.

Run scenario `08_PredictState`.  This scenario is configured to use a perfect IMU (only an IMU). Due to the sensitivity of double-integration to attitude errors, the accelerometer update was made very insignificant (`QuadEstimatorEKF.attitudeTau = 100`).  The plots on this simulation show element of your estimated state and that of the true state. 
The state prediction step is implemented in `QuadEstimatorEKF.cpp`, `PredictState()` functon. The prediction is done by converting the accelerometer measurement to inertial frame and integrating twice to calculate the position (and once for velocity). When you run scenario `08_PredictState` you should see the estimator state track the actual state, with only reasonably slow drift, as shown in the figure below:

![predict drift](images/predict-slow-drift.png)

With a realistic IMU (one with noise) in scenario `09_PredictionCov`, you will see a small fleet of quadcopter all using the prediction code to integrate forward. You will see two plots:
   - The top graph shows 10 (prediction-only) position X estimates
   - The bottom graph shows 10 (prediction-only) velocity estimates
In `QuadEstimatorEKF.cpp`, we calculate the partial derivative of the body-to-global rotation matrix in the function `GetRbgPrime()`. The rest of the prediction step (predict the state covariance forward) is implemented in `Predict()`. The implementation follows the math outlined in section 7.2 of [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj).
The process parameters reflect the magnitude of the error and are specified in `QuadEstimatorEKF.txt`. Note that as error grows our simplified model will not capture the real error dynamics (for example, specifically, coming from attitude errors), therefore  try to make it look reasonable only for a relatively short prediction period (the scenario is set for one second).

![good covariance](images/predict-good-cov.png)


### Magnetometer Update ###

Up until now we've only used the accelerometer and gyro for our state estimation.  In this step, the information from the magnetometer is added to improve filter's performance in estimating the vehicle's heading.
Scenario `10_MagUpdate` uses a realistic IMU and the plot is showing the estimated yaw error (`quad.est.e.yaw`). You should also see the estimated standard deviation of that state (white boundary).
The parameter `QYawStd` (`QuadEstimatorEKF.txt`) approximately captures the magnitude of the drift, as demonstrated here:

![mag drift](images/mag-drift.png)

The magnetometer update is implemented in the function `UpdateFromMag()`. It is fairly simple update since the yaw is measured directly by the magnetometer. The only neuance is normalizing the state in such a way that the error between the state and the measurement is within [-PI, +PI]. This results in the plot similar to this one:

![mag good](images/mag-good-solution.png)

The implementation follows the math outlined in section 7.3.2 of [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj).


### GPS Update ###

The EKF GPS Update is implemented in the function `UpdateFromGPS()`. The measurement model is fairly simple here as well since we directly measure 6 out of 7 variables in the state. 
In scenario `11_GPSUpdate` it is possible to configure the system to simulate ideal IMU and ideal GPS by modifying `Quad.UseIdealEstimator` in `config/11_GPSUpdate.txt`. For realistic IMU commenting out these lines in `config/11_GPSUpdate.txt`:
```
#SimIMU.AccelStd = 0,0,0
#SimIMU.GyroStd = 0,0,0
```
The implementation follows the math in section 7.3.1 of [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj).


## Authors ##

Thanks to Fotokite for the initial development of the project code and simulator.
