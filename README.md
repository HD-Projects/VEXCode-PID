# VEXCode Object Oriented PID

This is a object orinted PID loop that can be applied to drivetrains.

# **THIS IS STILL UNTESTED CODE**

## How to use

**All classes are in the PID namespace**

**You Need to call loop() like vex::task pidLoop(PODOBJECT.loop);**

<hr>

* ``PID(double P, double I, double D, motor_group PID_Motor);``

>Initialize a PID instance with the kP, kI, and kD you want and it takes either a motor_group of motor object

* ``DrivePID(double P, double I, double D, double turnP, double turnI, double turnD, motor_group left, motor_group right, inertial turnIntertial, double wheelDiameter, int sleep(Optional));``

>Drive PID is the lateral motion PID set up for turning and forward and backward movement

* ``enable();`` or ``OBJECT.enabled = true;``

>Enable the PID(Loop has to be running)

* ``disable()`` or ``OBJECT.enabled = false;``

>Disable the PID(Will stop PID loop)

* ``reset()`` or ``OBJECT.resetEncoder = true;``(``PID::PID``) or ``OBJECT.resetSensors = true;``(``PID::DrivePID``)

>Disable the PID(Will stop PID loop)

* ``move(double distance)`` or ``OBJECT.desiredValue = true``

>Set the desired value for the PID loop
