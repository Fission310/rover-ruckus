# rover-ruckus
Stuy Fission 310<br>
First Tech Challenge Rover Ruckus '18-'19 Robot Code<br>
Written and maintained by Joe Suzuki<br>

## Instructions
Clone this repository into the TeamCode directory of the FTC app, which can be found [here](https://github.com/ftctechnh/ftc_app).

## Documentation
Documentation can be found in the `docs` directory.

## Naming Conventions

### Autonomous
The main autonomous file should be named `AutonMain.java`. All autonomous OpMode Java files should be named as such
```
Auton<Alliance><Target1><Target2><Target...>.java
```

#### Alliance
- one of Blue, Red, or Both
- each alliance should specify in which side of the cargo hold the robot starts in

#### Target
- any of Sampling, Pattern, Lift (winch), Marker, Park (parking)
- file should be named in the order the targets are reached

### Teleop
The main teleop file should be named `TeleopMain.java`. Any other variations of the main teleop should be named `Teleop<Description>.java`.

### Sensors/Concepts
Sensor or concept OpModes should be named `<Concept/Sensor><Description>.java` and placed in the `prototype` directory.

### Testing/Prototyping
ALL test or prototyping OpMode files should be named `<Description>Test.java` and placed in the `prototype` directory.

### Hardware
Hardware files should be placed in the `hardware` directory. The main hardware map should be named `HardwareMain.java`. Any constant values for configuring hardware should be placed in the file `RCConfig.java` and any other constant values should be in `Constants.java`

