# project goals
the goal of this project is to create two simple applications that simulate a vehicle body controller and a brake controller.The basic idea is to create two separate software application that will talk to one another via a CAN bus. The body controller application will be in python and will be running on this PC. the brake controller application will be in C and will run on a TI microcontroller. The two applications will communicate with one another via a CAN bus. 

 ## Body controller

 The body controller shall issue brake commands to the brake controller based on user input. The body controller will have a simple user interface that allows the user to input commands such as "apply brakes" or "release brakes". The body controller will then send the appropriate command to the brake controller via the CAN bus. the brake commands shall be in the form of a brake % command, where 0% means no braking and 100% means full braking. The body controller will also receive feedback from the brake controller regarding the current brake status, which it will display to the user.


The body controller shall implement a vehicle state: off, startup, ready, shutdown.

 fault:
 If the brake controller sends back a pressure value that is below a requested threshold, the body controller shall display a warning message to the user indicating that there is a fault in the brake system. The body controller shall also log this fault for later analysis. this fault message will be part of a separate message

 ## Brake controller
the brake controller shall receive brake commands from the body controller and apply the appropriate braking force to the vehicle. The brake controller will also send feedback to the body controller regarding the current brake status, including the current brake pressure and any faults that may have occurred. The brake controller will be implemented in C and will run on a TI microcontroller.

The brake controller shall have a state machine: wakeup, checks, ready, braking, fault

Brake commands 0-100% shall correspond to a brake pressure value of 0-100 psi.

 ## ICD

-brake command %
-vehicle state (off, startup, ready, shutdown)
-brake machine state (wakeup, checks, ready, braking, fault)
-brake status feedback (pressure value, fault status, warining)
-machine state (idle, braking, fault)


### DBC vehicle brake

### DBC vehicle state