## FTC 14365 TeamCode Tutorial Branch

Welcome!

This branch of the project contains several programs meant to help incoming members with  learning
to program, with fully commented initialization files and tutorial programs meant to demonstrate
various concepts of the robot.

## Initialization Programs

There are several program files which are required for basic robot control that are then  built off
of in standard programs. These are arbitrarily called "initialization programs" and will vary 
slightly from season to season based off motor/servo/sensors used, but the  basic format will always
be the same.

These programs are:

* Robot:      Robot is the largest and most comprehensive object that contains all other subsytems
              and does most of the heavy software mapping of individual parts. Individual programs
              should access all key functions through a Robot object.
* Delivery:   Delivery contains any miscellaneous motors and servos and their corresponding logic,
              typically involving the specific systems used in that competition.
* Vision:     Visions contains the webcam and all logic needed for both Tensorflow and Vuforia
              CV detection and tracking.
* Sensors:    Sensors contain any robot feedback mechanisms and logic not covered by any other
              category (traditionally, this has just been distance sensors)
* Subsystems: Subsystems is the "root" for Delivery, Vision, and Sensors, so therefore contains all
              basic objects for this. (Note: This type of program, the Abstract Class, is very
              complicated and as such you don't need to fully understand why it exists- just that
              it's important.
* AutoPrime:  AutoPrime contains all information important ONLY to autonomous, not Tele-Op as well;
              otherwise, that information would simply be stored in the Robot file. Also provides a
              "foundation" necessary for autonomous programs.
* Roadrunner: Not technically a single program, but rather a collection of many programs all focused 
              around the autonomous driving of the robot through custom PIDF settings and an
              incredibly thorough coordinate system of travel that further syncs with the IMU
              gyroscope and odometry dead reckoning wheels. All roadrunner related programs can be
              found in the drive, trajectorysequence, and util folders. Further information on how
              to utilize roadrunner can be found [here](https://learnroadrunner.com/). This also
              contains all drive motors and the IMU gyroscope.

These are also all on the master branch in order to make sure it has all basic initialization files.
As always, these systems are always constantly being updated from year to year based on the needs of
that year- but these still contain the same syntax and general context across all seasons.

## Tutorial Files

Unique to this branch, however, are the Tutorial files, focused around teaching new programmers how
to actually implement each subsystem's mechanisms.

* TutorialDeliveryAuto: Demonstrates basic delivery usage in autonomous
* TutorialDriveAuto: Demonstrates roadrunner usage in autonomous, with some examples of how to use
                     other mechanisms within roadrunner trajectory sequences as well.
* TutorialSensorAuto: Demonstrates basic sensor usage in autonomous (within a refresh loop)
* TutorialVisionAuto: Demonstrates custom Tensorflow model tracking and Vuforia navigation target
                      tracking as well as providing relevant telemetry for those.
* TutorialThreadAuto: Demonstrates how threads work in a simple autonomous program through updating
                      a distance sensor and manipulating a delivery aspect of the robot, as well as
                      providing telemetry feedback throughout (also covers the purpose of
                      TutorialDeliveryThread.java and TutorialSensorThread.java).
* TutorialTeleOp: Demonstrates a simple TeleOp program and how it's structured (NOTE: TODO)

Obviously, there is much more to programming than this. However, this should give new members a
good sense of how to manipulate elements of the robot. From here, a general further tutorial on java
and programming logic would be most helpful to start programming quickly.

That's all for this document! As always, ask any programming questions to Logan and let him know
when you feel confident in your ability to interpret and understand this tutorial branch!