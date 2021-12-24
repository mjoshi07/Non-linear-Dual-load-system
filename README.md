# Non-linear-Dual-load-system
Design LQR and LQG controller for a non-linear dual suspended load system

## Problem statement
There are two loads suspended from cables attached to a crane

![image](https://github.com/mjoshi07/Non-linear-Dual-load-system/blob/main/data/problem_statement/problem_statement.png)

Obtain the following:
* Dyanmics of the system and nonlinear state space representation
* Linearized system around equilibrium point
* conditions for controllability
* LQR controller for the linearized system and reponse to initial conditions when applied to original non linear system
* Observable system from a list of output vectors
* Best Luenberger observer
* Output feedback controller 
* Initial state response when applied to original non linear system

Read the problem statement in detail [here](https://github.com/mjoshi07/Non-linear-Dual-load-system/blob/main/data/problem_statement/problem_statement.pdf)

* Using Euler-lagrange equations we obtain the dynamics of the system

* The non linear state space representation is given by: 
* <img src="https://github.com/mjoshi07/Non-linear-Dual-load-system/blob/main/data/output/non_linear_state.png" width="500"/>

* The linearized system state space representation is give by: 
* <img src="https://github.com/mjoshi07/Non-linear-Dual-load-system/blob/main/data/output/linearized_state_space.png" width="500"/>

* Initial condition response for open loop and closed loop system is shown below:
* <img src="https://github.com/mjoshi07/Non-linear-Dual-load-system/blob/main/data/output/open_loop.png" width="400" height="200"/> <img src="https://github.com/mjoshi07/Non-linear-Dual-load-system/blob/main/data/output/closed_loop.png" width="400" height="200"/>

* For the original non linear system, initial state response is shown below:
* <img src="https://github.com/mjoshi07/Non-linear-Dual-load-system/blob/main/data/output/state_response_non_linear.png" width="500"/>

* Initial and Step response for the output vector - x(t) using Luenberger observer is shown below:
<img src="https://github.com/mjoshi07/Non-linear-Dual-load-system/blob/main/data/output/initial_response_case1.png" width="400" height="200"/> <img src="https://github.com/mjoshi07/Non-linear-Dual-load-system/blob/main/data/output/step_response_case1.png" width="400" height="200"/>

* Initial and Step response for the output vector - [x(t),𝜃2(t)]  using Luenberger observer is shown below:
<img src="https://github.com/mjoshi07/Non-linear-Dual-load-system/blob/main/data/output/initial_response_case2.png" width="400" height="200"/> <img src="https://github.com/mjoshi07/Non-linear-Dual-load-system/blob/main/data/output/step_response_case2.png" width="400" height="200"/>

* Initial and Step response for the output vector - [x(t),𝜃1(t),𝜃2(t)]  using Luenberger observer is shown below:
<img src="https://github.com/mjoshi07/Non-linear-Dual-load-system/blob/main/data/output/initial_response_case3.png" width="400" height="200"/> <img src="https://github.com/mjoshi07/Non-linear-Dual-load-system/blob/main/data/output/step_response_case3.png" width="400" height="200"/>

* Intial and Step reponse for the smallest output vector - x(t) using output feedback controller and LQG method
<img src="https://github.com/mjoshi07/Non-linear-Dual-load-system/blob/main/data/output/LQG_initial_linear.png" width="400" height="200"/> <img src="https://github.com/mjoshi07/Non-linear-Dual-load-system/blob/main/data/output/LQG_step_linear.png" width="400" height="200"/>

* Initial state response for the closed loop feedback controller is shown below:
* <img src="https://github.com/mjoshi07/Non-linear-Dual-load-system/blob/main/data/output/state_response_LQG_non_linear.png" width="500"/>

* Initial state response for the Kalman Bucy state estimator is shown below:
* <img src="https://github.com/mjoshi07/Non-linear-Dual-load-system/blob/main/data/output/kalman_bucy_state_response.png" width="500"/>
