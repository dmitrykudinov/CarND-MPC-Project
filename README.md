# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## Model Predictive Control with Latency


---

## The Model
The state which successfully got utilized, and without spending much time on tuning of cost function coefficients, was taken from the class materials and consists of the following members:
- x
- y
- psi
- v
- CTE
- E_psi

With `steering` and `acceleration` as actuators.

The cost function, too, pretty much remains intact with the exception of heavier weights assigned to CTE, Trajectory Error, and smoothness of steering in particular:

```
    for (unsigned int t = 0; t < N; t++){
      //CTE
      fg[0] += 2.0*CppAD::pow(vars[cte_start + t], 2);
      //Trajectory error
      fg[0] += 850.0*CppAD::pow(vars[epsi_start + t], 2);
      //Speed Error
      fg[0] += 0.25*CppAD::pow(vars[v_start + t] - ref_v, 2);    
      
      //suppressing too much active use of actuators
      if (t < N - 1) {
        //steering
        fg[0] += CppAD::pow(vars[delta_start + t], 2);
        //acceleration
        fg[0] += CppAD::pow(vars[a_start + t], 2);
      }
      
      //smoothing actuator curves
      if (t < N - 2) {
        //steering
        fg[0] += 5000.0*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
        //acceleration
        fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);    
      }
    }
```

The reason of choosing such heavy weights for steering accuracy is because of the `const double ref_v = 80.0;` - as in real life the faster you go, the more precise and smooth steering manipulations should be.

In general, I am impressed how much less effort, compared to PID controller, it took to gain a smooth completion of a lap, and at a significantly higher speeds.

## Timestep Length and Elapsed Duration (N & dt)

For this model, the working solution uses 10 ticks every 0.1sec:
```
size_t N = 10;
double dt = 0.1;
```

## Polynomial Fitting and MPC Preprocessing

Since this is a curved track with, in some places, sequences of fast coming turns, a 3rd order polynomial was chosen to work for both waypoint approximation, 

```
auto coeffs = polyfit(xvals, yvals, 3);
```
and MPC function:
```
      AD<double> f0 = coeffs[0] + coeffs[1]*x0 + coeffs[2]*pow(x0,2) + coeffs[3]*pow(x0,3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*pow(x0,2));
```


---

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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * If challenges to installation are encountered (install script fails).  Please review this thread for tips on installing Ipopt.
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
