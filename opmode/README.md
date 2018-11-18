# OpModes
All autonomous and teleOp programs will be in this directory<br>


## Naming Conventions

### TeleOp
The main teleOp file should be named `TeleopMain.java`.


### Autonomous
The main autonomous file should be  `AutonDepot.java` or `AutonCrater.java` depending on starting side. All autonomous OpMode Java files should be named as such
```
Auton<Alliance><Target1><Target2><Target...>.java
```

#### Alliance
- one of depot, crater, or Both
- each alliance should specify in which side of the cargo hold the robot starts in

#### Target
- any of Sampling, Pattern, Lift (winch), Marker, Park (parking)
- file should be named in the order the targets are reached

### Teleop
The main teleop file should be named `TeleopMain.java`. Any other variations of the main teleop should be named `Teleop<Description><Driving style>.java`.
Descriptions of the opmodes should specify what drivetrain the driver will drive.
Driving style refers to tank, arcade, other ways of driving the robot using the analog sticks.