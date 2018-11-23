package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.tankdrive.Drivetrain;
import org.firstinspires.ftc.teamcode.util.vision.VisionManager;

/**
 * HardwareTank is the class that is used to define all of the hardware for a single robot. In this
 * case, the robot is: Rover Ruckus.
 *
 * This class contains autonomous actions involving multiple mechanisms. These actions
 * may be common to more than one routine.
 */
public class HardwareTank extends Mechanism {

    /* Mechanisms */
    /**
     * Instance variable containing robot's drivetrain.
     */
    public Drivetrain drivetrain;
    /**
     * Instance variable containing robot's Hanger.
     */
    public Hanger hanger;
    /**
     * Instance variable containing robot's markers arm.
     */
    public Marker marker;

    /* Miscellaneous mechanisms */

    /**
     * Default constructor for HardwareMain. Instantiates public mechanism instance variables.
     */
    public HardwareTank(){
        drivetrain = new Drivetrain();
        hanger = new Hanger();
        marker = new Marker();
    }
    /**
     * Overloaded constructor for HardwareMain. Calls the default constructor and sets the OpMode
     * context for the robot.
     *
     * @param opMode    the LinearOpMode that is currently running
     */
    public HardwareTank(LinearOpMode opMode){
        this.opMode = opMode;
        drivetrain = new Drivetrain(opMode);
        hanger = new Hanger(opMode);
        marker = new Marker(opMode);
    }

    /**
     * Initializes all mechanisms on the robot.
     * @param hwMap     robot's hardware map
     */
    public void init(HardwareMap hwMap) {
        drivetrain.init(hwMap);
        hanger.init(hwMap);
        marker.init(hwMap);
    }

    /**
     * Waits for opMode's to start. Can perform actions while waiting.
     */
    public void waitForStart() {
        while (!opMode.isStarted()) {
            opMode.telemetry.addData("Heading:", drivetrain.getHeading());
            opMode.telemetry.update();
        }
    }

    public void land(){
        hanger.hangeToPos(FieldConstants.HANG_HEIGHT);
    }

    public void sample(VisionManager visionManager){
        if (opMode.opModeIsActive()) {
            while (!visionManager.isGoldAligned()) {
                drivetrain.driveArcade(0.3, 0.0);
            }
            if (visionManager.isGoldAligned()) {
                drivetrain.turn(90.0, 3.0);
                drivetrain.driveToPos(0.3, 13,13,3);
                drivetrain.driveToPos(0.3, -13,-13,3);
                drivetrain.turn(-90.0, 3.0);
            }
        }
    }
    public void samplePID(VisionManager visionManager){
        if (opMode.opModeIsActive()) {
            while (!visionManager.isGoldAligned()) {
                drivetrain.driveArcade(0.25, 0.0);
            }
            if (visionManager.isGoldAligned()) {
                drivetrain.driveToPos(0.3, -FieldConstants.TILE_HYPOTENUSE / 6,-FieldConstants.TILE_HYPOTENUSE / 6,2);
                drivetrain.turnPID(-90);
                drivetrain.driveToPos(0.3, FieldConstants.FLOOR_TILE * 6,FieldConstants.FLOOR_TILE * 6,3);
                drivetrain.driveToPos(0.3, -FieldConstants.FLOOR_TILE * 6,-FieldConstants.FLOOR_TILE * 6,3);
                drivetrain.turnPID(90);
            }
        }
    }

    public void markerDrop(){
        marker.markerLeft();
    }


    /**
     * Autonomous action for sampling the gold cube. Uses the robot's servo arm mechanism to detect gold cube
     * and in a linear slide fashion.
     * This assumes the color sensor faces the back of the robot.
     *
     *  @param visionManager    VisionManager containing the GoldDetector
     *  @param isAllianceRed    whether or not the robot is on the Red Alliance
     */

    // gold detector

    //

    /**
     * Autonomous action for dropping the marker. Uses the robot's distance sensor to detect the robot's
     * position using the vuforia pictograph. Moves parallel to wall until the edge is
     * reached.
     *
     *  @param targetCol      the cryptobox column that is being targeted (left is 0, center is 1, right is 2)
     *  @param isAllianceRed    whether or not the robot is on the Red Alliance
     */

    // Marker scorer

    //
}
