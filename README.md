# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Introduction

This project uses MPC algorithm to control the steering and acceleration of the vehicle so the vehicle can stay in the track. The source code can be found at [here](https://github.com/fwa785/CarND-MPC-Project).

## Algorithm

### Introduction

The MPC algorithm defines 6 states:

* X position: px
* Y position: py
* Driving Velocity: v
* Driving Orientation: psi
* Cross Track Error(CTE): cte. 
* Orientation Error: epsi

2 actuators:

* Orientation  Change: delta
* Acceleration: a

And it defines the time duration of the model(T) and the timesteps(dt) used in the model.  The algorithm pass the initial states to the model, and model uses the equations to calculate the next state for each timesteps in the time duration. It finds the best choices of actuators to minimize the overall cost during the time duration T.

### Polynomial Fitting

The waypoints are provided by the simulator. It is the reference trajectory of the vehicle. The trajectory is curved, so I use 3rd order polynomial function to fit the waypoint. The coefficients of the polynomial function are found by the fitting so that:
$$
polyeval(coeff, x) = coeff_{0} + coeff_{1}\times x + coeff_{2}\times x^2 + coeff_{3}\times x^3
$$
 The angle of the reference trajectory at each point is:
$$
polyderivative(coeff, x) = coeff_{1} + 2\times coeff_{2}x + 3\times coeff_{3}x^2
$$

$$
psi = atan(polyderivative(x))
$$

The waypoints are transformed into the vehicle's coordinate system before it's been fitted. 

### Preprocess

The state information provided by the simulator is for the map coordinate system. However, it is easy to process the information in the vehicle's coordinate system. For example, CTE is defined as the difference between the vehicle's current position and the reference trajectory. If we transform the states into the vehicle's coordinate system, then the initial orientation of the vehicle is always point to x axis, and CTE can be defined as simple as the difference of the desired y position and the actual vehicle's y position.

Therefore, before we start processing the information, the waypoints, the vehicle's states: px, py, v and psi are all converted into the vehicle's coordinates. Below is the transformation equation:
$$
{x_{c}=(x_{m} - px)*cos(-psi) - (y_{m} - py)*sin(-psi)}
$$
$$
{y_{c}=(y_{m} - py)*cos(-psi) + (x_{m} - px)*sin(-psi)}
$$

After the transformation, the vehicle's state px, py and psi all become 0.

The CTE is defined as:
$$
cte = polyeval(coeff, px) - py
$$
And the epsi is defined as:
$$
epsi = psi - atan(polyderivative(coeff, px))
$$

### Model

The model takes the initial state of px, py, v, psi, cte, and epsi and coefficients as the input. It defines the equations to calculate the states on the next timestep as following:
$$
x_{t+1} = x_{t} + v_{t} \times cos(psi_{t}) \times dt
$$

$$
y_{t+1} = y_{t} + v_{t} \times sin(psi_{t}) \times dt
$$

$$
psi_{t+1} = psi_{t} + v_{t} \times \frac{delta_{t}}{L_{f}}\times dt
$$

$$
v_{t+1} = v_{t} + a_{t}\times dt
$$

$$
cte_{t+1} = polyeval(coeff,x_{t}) - y_{t} + v_{t}\times sin(epsi_{t})\times dt
$$

$$
epsi_{t+1} = psi_{t} - atan(polyderivative(coeff, x_{t})) + \frac{v_{t}}{L_{f}}\times delta_{t}\times dt
$$
#### Constraints

There is no constraints for the states, although in the code, they're defined as constrained between the minimum double value and the maximum double value. It's mainly for the coding purpose.

The equations of the next states are defined as putting constrains to the difference between next state's value and the next state's equation. The constrain for this difference for every state is set to 0, so the equation has to be strictly followed.

The orientation change(delta) is constrained between -25 degree and 25 degree, so the vehicle doesn't make too big turns. The acceleration(a) is constrained between -1 and 1, so the vehicle doesn't accelerate or decelerate too quick. In the real life, the vehicles have limitation on the steering and acceleration they can make, so it is also to match the real life vehicle setting.

#### Cost

The cost of the model is defined in the following components:

* The deviation of cte from 0 with a weight of 15. The weight for cte is not very high because it's OK to derivate from the reference trajectory as long as the vehicle still stay within the track. I found out that if I set the cte's weight too high, the model tried hard to stay on the reference trajectory by steering the vehicle very frequently
  $$
  15 \times cte^2
  $$

* The deviation of epsi from 0 with a weight of 15.  Same reason for the cte, it doesn't need to have high weight.
  $$
  15 \times epsi^2
  $$

* The deviation of velocity from the reference velocity 100 mph.
  $$
  (v-v_{ref})^2
  $$

* The deviation of delta from 0 with a weight of 16000. Because the delta is small after converted to radians, and we'd like to minimize unnecessary orientation changes, I gave delta very high weight.
  $$
  16000 \times delta^2
  $$

* The deviation of a from 0 with a weight of 10.
  $$
  10 \times a^2
  $$

* The change of delta in each timestep with a weight of 8000. It's critical not to let the vehicle's steering change too quick.
  $$
  8000 \times (delta_{t+1} - delta_{t})^2
  $$

* The change of a in each timestep.
  $$
  (a_{t-1} - a_{t})^2
  $$








### Duration

The MPC model defines time duration(T) for the prediction and the timestep(dt). The number of timesteps is defined as N = T/dt. When we choose the N and dt, we want dt to be as small as possible so the calculation of the states in the next timestep is more accurate. However, the duration T can't be too small, or the model can't predict the change far enough to make the correct turn. But if dt is small, and T is big, then N is big. When N is big, the computation of the model is complicated and takes too long. If the computation takes too long, it adds the delay to the action. The delay causes inaccuracy in the prediction, so I need to limit N to a not big number to control the computation complexity. After trying different N and dt, I eventually settled N=8, and dt=0.05.

### Latency

There is a delay from the time the vehicle sends the information to the time the vehicle receives the control information. In this project, 100ms latency is simulated. So we need to adjust the initial states with the latency, or the control values determined by the model won't work well with the state 100ms later, especially if the vehicle drives with high speed, steering or acceleration. The initial state of the vehicle is adjusted with the follow equation so it uses the state 100ms later as the initial state.
$$
px = px + v \times cos(psi) \times delay
$$
$$
py = py + v \times sin(psi) \times delay
$$
$$
psi = psi - \frac{v}{L_{f}} \times delta \times delay
$$
$$
v = v + a \times delay
$$

## Result

With the algorithm described in this document implemented, the vehicle can travel on the track for many loops with a speed around 80mph without issue. 

[![IMAGE Final Result](https://img.youtube.com/vi/gUA74OtXAMk/0.jpg)](https://www.youtube.com/watch?v=gUA74OtXAMk)




## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.


