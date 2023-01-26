# LIFT EXAMPLE


You need to download/clone the following repositories:

- [DYNAMICAL MODEL](https://github.com/industrial-control-group-unibs/IMOCO_LIFT_MODEL)


- [CONTROL AND MOTION PLANNING](https://github.com/industrial-control-group-unibs/IMOCO-Lift-Motion-Law)


There are two examples: 

- test_no_ctrl.slx runs the model with only the motion planning and the kinematic model

- test.slx  runs the complete simulator (motion planning, control, dynamic model)

You have to load the data running load_data.m

You have to use the library:

[DYNAMICAL MODEL] "LumpedParameterLiftModel", and 
[CONTROL AND MOTION PLANNING] "LiftMotionControlLibrary".


The simulink scheme consists of: (from left to right)

![complete scheme](https://github.com/industrial-control-group-unibs/LIFT-Examples/blob/main/docs/complete_model.png)

(1) The user simulator. To eliminate it, simply replace the output "destination_request" with a step with the desired floor number (the elevator starts at floor zero) 

(2) The lift control. The main part is the "lift logic control block" presented in the figure below, with its mask.

![lift logic](https://github.com/industrial-control-group-unibs/LIFT-Examples/blob/main/docs/lift_logic.png)

The inputs are:
- floor switch (it sends the floor number when active, -1 if no switch is active)
- below floor proximity switch (used in the two-level speed profile, it sends the floor number when active, -1 if no switch is active)
- above floor proximity switch (used in the two-level speed profile, it sends the floor number when active, -1 if no switch is active)
- destination floor 

The outputs are:
- elevator speed profile (m/s) and its acceleration
- idle signal
- door and brake management

In this block, there is also the PID implementation.

(3) Simulated Lift. This block contains the lift Dynamic model, the sensors simulation as shown below.

![sensors](https://github.com/industrial-control-group-unibs/LIFT-Examples/blob/main/docs/sensor.png)

(4) Assert if the lift crashes.


## Ack

This work was partially supported by IMOCO4.E project (H2020-ECSEL, European Commission – G.A.101007311).
