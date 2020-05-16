# End to End Vehicle Localizer

This project highlights the implementation of an End to End sparse localization of a vehicle in C++.

![Image cropped to region of interest](https://github.com/ashsiv/End-to-End-Vehicle-Localizer/blob/master/images/Main.JPG)

As seen in the above image, the blue circle (denoting the vehicle location estimate) falls right on top of the ground truth location (car symbol).
## Project Introduction

The inputs to the sparse localizer are 
1. Initial noisy GPS estimate of the initial location of the vehicle.
2. Map of the initial GPS estimate location
3. Noisy sensor/control data (velocity and yaw rate), observations (LIDAR sensor) at each time step.

Using the above information, a 2 dimensional particle filter is implemented in C++ to estimate the location of the vehicle.


## Implementing the Particle Filter
The particle filter implementation can be found in 'particle_filter.cpp' under the src directory.

### Particle Filter Initialization
Firstly the filter is initialized by specifying the no. of particles (I have chosen 100 samples). Next a normal distribution of particles is generated around the given initial estimate of GPS postion. The standard deviations of the (x,y) positions - GPS sensor noise, yaw rate and yaw sensor noise are passed as inputs to the filter. The weights of all the particles are initialized to 1.0 .

```
void ParticleFilter::init(double x, double y, double theta, double std[])
```
### Prediction
Next step is prediction. For each time step, the control data - velocity and yaw rate are used to predicted the future positions of each particle. Please note that (x,y) position noise and the yaw rate noise (Yaw sensor) are also added to the predicted particle postions.

```
void ParticleFilter::prediction(double delta_t, double std_pos[],double velocity, double yaw_rate)
```
```
particles[i].x = particles[i].x + ( (velocity/yaw_rate) * (   sin( particles[i].theta + (yaw_rate*delta_t) ) - sin(particles[i].theta) ) );
particles[i].y = particles[i].y + ( (velocity/yaw_rate) * ( - cos( particles[i].theta + (yaw_rate*delta_t) ) + cos(particles[i].theta) ) );
```
### Transformation, Data Association, Weight updates

Following prediction, the observations from LIDAR are used to update the weights of the particles. This is done in the following steps.

* First for computational efficiency, the given map landmarks is screened in such a way that only landmarks that fall under the LIDAR sensor range are considered (This helps to save the cycle time) for particle weight update.

* It is to be noted that the observations are with respect to the Car's frame of view (LIDAR sensor). Hence observations are transmformed to map coordinates (with respect to each particle).
```
transformed_obs.x  =  particles[i].x + (observations[j].x * cos(particles[i].theta)) - (sin(particles[i].theta)*observations[j].y);
transformed_obs.y  =  particles[i].y + (observations[j].x * sin(particles[i].theta)) + (cos(particles[i].theta)*observations[j].y);

```
* Next the transformed observations are associated with the nearest landmark and the corresponding landmark's id is associated for each particle.
```
dataAssociation(f_landmarks,T_obs);
```
* Finally a multivariate gaussian probability is esitmated for each particle and its associated landmark. The product of all the probabilities is assigned as the weight of the particle.

### Best particle identification
Following weight updates, the weights of the particles are normalized and resampled with a probability proportional to their weight.
A discrete distribution function in C++ is used.
```
discrete_distribution<int> distribution(wts.begin(),wts.end());
```
## Results

The particle with the heighest weight is assigned as the best particle that matches the vehicle location.

[Output Video.mp4](https://github.com/ashsiv/End-to-End-Vehicle-Localizer/blob/master/images/Output%20video.mp4)

![Image cropped to region of interest](https://github.com/ashsiv/End-to-End-Vehicle-Localizer/blob/master/images/Output.JPG)

## Running the Code

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh





