# Introduction 
This introduction aims to unravel the complexities of quadrotor dynamics and showcase the application of the proposed control approaches for precise tracking. Please add more but brief intro, including the importance of robust flight control, the design is part of a masters degree project done under the guidance of professor Wu Dawei, Hohai University Dept. of Artificial intelligent and Automation    
Focus:
*	Quadrotor Dynamics
*	State-Space Equation
*	Super-twisting adaptive gain sliding Mode Controller: Development through the backstepping approach.
*	Adaptive control (RBF)
# Quadrotor Dynamics
The Newton-Euler approach, which is a widely used dynamic modeling method for UAVs, and other rigid body systems. It describes the forces and torques acting on a rigid body using Newton's second law of motion for translation and Euler's equations for rotation. This approach is essential for control system design, ensuring accurate representation of UAV dynamics for trajectory tracking, disturbance rejection, and stability analysis.

<img src="https://github.com/Amlakyalew007/Quadrotor-UAV-fast-super-twisting-adaptive-backstepping-SMC/blob/main/images/quadrotor%20.png" width="400" height="350" />

## The following assumptions are made to find quadrotor dynamic equations
*	The quadrotor frame is both rigid and symmetrical.
* The center of mass aligns with the origin.
* The propellers maintain rigidity.
* The thrust and drag exhibit a proportional relationship to the square of the propeller speed.
                     ![alt text](https://github.com/Amlakyalew007/Quadrotor-UAV-fast-super-twisting-adaptive-backstepping-SMC/blob/main/images/equation.png)

![alt text](https://github.com/Amlakyalew007/Quadrotor-UAV-fast-super-twisting-adaptive-backstepping-SMC/blob/main/images/state%20space.png)


 # Fast Super-Twisting Adaptive Backstepping Sliding Mode control
Fast Super-Twisting Adaptive Backstepping Sliding Mode Control (FST-ABSMC) is an advanced control strategy designed for nonlinear systems facing uncertainties, disturbances, and model mismatches. It integrates the robustness of Sliding Mode Control (SMC), the smoothness and finite-time convergence of the Super-Twisting Algorithm (STA), and the adaptability of real-time parameter tuning. By incorporating a fast convergence term and a hyperbolic tangent function.

**Characteristics of FSTABSMC**
  *	Super-Twisting Control Law
  *	Adaptive Gain Tuning
  *	Fast Convergence
  *	Disturbance Rejection
  *	Energy Efficiency
  *	Smooth Control Signals
## Problem Formulation
Since the dynamics of the quadrotor UAV can be subdivided into second order subsystems, we can design the proposed control algorithm on a common second order system as shown below.

![alt text](https://github.com/Amlakyalew007/Quadrotor-UAV-fast-super-twisting-adaptive-backstepping-SMC/blob/main/images/equation%201%20(ODE).png)

## Controller Design
The control law is designed using the above second order system and will be applied to the UAVs subsystems. The control objectives to be achieved in here are the following aspects:

  *	All signals in the closed-loop system are asymptotically stable.
  *	The system output can track the specified desired signal.
  *	The adaptive gains adjust automatically.   
  * A backstepping control combined with fast super-twisting sliding mode control with adaptive gain will be designed for the system control and an adaptive RBF control law will be developed for the uncertainty/disturbance control.

The desired trajectory feeds into both attitude and translation controllers, which generate control signals for the quadrotor UAV model. The UAV is also exposed to disturbances. Its output states are measured and fed back to the controllers for continuous adjustment, forming a closed-loop control system.

![alt text](https://github.com/Amlakyalew007/Quadrotor-UAV-fast-super-twisting-adaptive-backstepping-SMC/blob/main/images/control%20D.png)

  ### The tracking error variable and the sliding surface respectively are defined as

![alt text](https://github.com/Amlakyalew007/Quadrotor-UAV-fast-super-twisting-adaptive-backstepping-SMC/blob/main/images/equation%202.png)

To achieve the control objectives of this approach the following Lyapunov function must be realized

![alt text](https://github.com/Amlakyalew007/Quadrotor-UAV-fast-super-twisting-adaptive-backstepping-SMC/blob/main/images/equation%203.png)

The tracking error and it’s derivative

![alt text](https://github.com/Amlakyalew007/Quadrotor-UAV-fast-super-twisting-adaptive-backstepping-SMC/blob/main/images/equation%204.png)

The derivative of the its candidate Lyapunov function

![alt text](https://github.com/Amlakyalew007/Quadrotor-UAV-fast-super-twisting-adaptive-backstepping-SMC/blob/main/images/equation%205.png)

RBF Neural Network: It is used approximate the system uncertainty
The system uncertainty (unknown function) and the RBF activation function (Gaussian)


![alt text](https://github.com/Amlakyalew007/Quadrotor-UAV-fast-super-twisting-adaptive-backstepping-SMC/blob/main/images/equation%206.png)

The sliding surface and it’s first derivative 

![alt text](https://github.com/Amlakyalew007/Quadrotor-UAV-fast-super-twisting-adaptive-backstepping-SMC/blob/main/images/equation%207.png)

Retaining some of the basic characteristics of classical super-twisting algorithm a few improvements are made as shown below

![alt text](https://github.com/Amlakyalew007/Quadrotor-UAV-fast-super-twisting-adaptive-backstepping-SMC/blob/main/images/equation%208.png)

The adaptive gain parameters b and c for the super-twisting sliding mode control are designed as

![alt text](https://github.com/Amlakyalew007/Quadrotor-UAV-fast-super-twisting-adaptive-backstepping-SMC/blob/main/images/equation%209.png)

![alt text](https://github.com/Amlakyalew007/Quadrotor-UAV-fast-super-twisting-adaptive-backstepping-SMC/blob/main/images/equation%2010.png)

# Quadrotor UAV Tracking and Disturbance Rejection Simulation

This project presents the simulation of a quadrotor UAV model focusing on its tracking performance and disturbance rejection capability under external disturbances such as wind. The UAV dynamics are modeled using six coupled nonlinear differential equations that include both rotational and translational motion, with disturbances introduced into each subsystem.

The external disturbances are defined as:
![alt text](https://github.com/Amlakyalew007/Quadrotor-UAV-fast-super-twisting-adaptive-backstepping-SMC/blob/main/images/equation%2011.png)

These sinusoidal disturbances simulate wind and environmental effects acting on the UAV during flight.

The control design and simulation aim to:
* Accurately track desired trajectories
*  Maintain stability under nonlinear coupled dynamics
* Reject disturbances affecting both attitude and position

Simulation is conducted in MATLAB/Simulink, and all models are available in this repository.

For running the simulation model:

![alt text](https://github.com/Amlakyalew007/Quadrotor-UAV-fast-super-twisting-adaptive-backstepping-SMC/blob/main/images/mat%201.png)

* Download the "MATLAB & Simulink" Folder onto your computer. 
* open the FSTABSMC.slx file and run it. 
* After this, run xyzplot.m file. 

The complete simulation model was developed using MATLAB & Simulink as shown in the image given below: 

![alt text](https://github.com/Amlakyalew007/Quadrotor-UAV-fast-super-twisting-adaptive-backstepping-SMC/blob/main/images/mat%202.png)


The following image presents the simulation results. The UAV was successfully able to complete the maneuver with minimal errors, even in the presence of sinusoidal disturbance which proved the high performance and robustness of the controllers.

### Translational Motion 

![alt text](https://github.com/Amlakyalew007/Quadrotor-UAV-fast-super-twisting-adaptive-backstepping-SMC/blob/main/images/x%2Cy%2Cz%20tracking.jpg)

### Rotational Motion

![alt text](https://github.com/Amlakyalew007/Quadrotor-UAV-fast-super-twisting-adaptive-backstepping-SMC/blob/main/images/phi%2C%20theta%2C%20psi.jpg)

**note:** To generate visually rich and dynamically challenging 3D trajectories, a torus knot is used for the position (x, y, z) desired trajectory, while a helical spiral is applied for the rotational (roll, pitch, yaw) desired trajectory.
