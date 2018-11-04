package org.firstinspires.ftc.teamcode.util;


import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.disnodeteam.dogecv.scoring.MaxAreaScorer;
import com.disnodeteam.dogecv.scoring.RatioScorer;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

/**
 * This class handles the robot CV functions and makes combining Vuforia and DogeCV easy in a single
 * OpMode.
 */
public class VisionManager {
    private SamplingOrderDetector sampleDetector;
    private GoldAlignDetector goldDetector;

    private Dogeforia vuforia;
    private List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    boolean targetVisible;
    private OpenGLMatrix lastLocation = null;

    public void samplingInit(HardwareMap hwMap) {
        sampleDetector = new SamplingOrderDetector();
        sampleDetector.init(hwMap.appContext, CameraViewDisplay.getInstance());
        sampleDetector.useDefaults();

        sampleDetector.downscale = 0.4; // How much to downscale the input frames

        // Optional Tuning
        sampleDetector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //sampleDetector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        sampleDetector.maxAreaScorer.weight = 0.001;

        sampleDetector.ratioScorer.weight = 15;
        sampleDetector.ratioScorer.perfectRatio = 1.0;

        sampleDetector.enable();
    }

    /**
     * Get the current order of Gold cube using DogeCV.
     * @return UNKNOWN, LEFT, CENTER, RIGHT based on cube order
     */
    public SamplingOrderDetector.GoldLocation getGoldLocation() {
        return sampleDetector.getCurrentOrder();
    }

    /**
     * Get the last known order of Gold cube using DogeCV.
     * @return UNKNOWN, LEFT, CENTER, RIGHT based on cube order
     */
    public SamplingOrderDetector.GoldLocation getLastGoldOrder() {
        return sampleDetector.getLastOrder();
    }

    public void samplingStop() {
        sampleDetector.disable();
    }

    public void goldAlignInit(HardwareMap hwMap) {
        goldDetector = new GoldAlignDetector();
        goldDetector.init(hwMap.appContext, CameraViewDisplay.getInstance());
        goldDetector.useDefaults();

        // Optional Tuning
        goldDetector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        goldDetector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        goldDetector.downscale = 0.4; // How much to downscale the input frames

        goldDetector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //goldDetector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        goldDetector.maxAreaScorer.weight = 0.005;

        goldDetector.ratioScorer.weight = 5;
        goldDetector.ratioScorer.perfectRatio = 1.0;

        goldDetector.enable();
    }

    public boolean isGoldAligned() {
        return goldDetector.getAligned();
    }

    public double getGoldPosX() {
        return goldDetector.getXPosition();
    }

    public void goldAlignStop() {
        goldDetector.disable();
    }

    public void vuforiaInit(HardwareMap hwMap) {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AZg2Pg7/////AAABmUJNdV5U3UdZuILyo65XAwgrRZJyRDvOUmdlSJISRf2lUECnalP9t4Vq+TPV2IXxriMQEqJFLiWzA30Qs7Kyx9qYi3HYgACR8ifSGJWZRgCDFa46j2xEo1xBP5S5z3bDl/7evo6bhaf3z5y9pdc1LKUVWx4woR0VN36y/IvcNI8FU4rUA5Big4AB4XKIlYpESWAYTj8hnAQSlXkKrG3CDKv7F2IDbZmtCv2SpZgfZEj+OPNlPRO41izEPeMX/svz6UoaugAfOt3M7ZwyX/+ZBZRJfgDRA7OiEPFi4aoACzWUBY2mSODV1n+BJN3+GRR2arcgSDUIJ7mTJ2nwNVdNblXSj18nOVLlS1WpLxCWEDzo\n";
        parameters.fillCameraMonitorViewParent = true;

        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();

        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */

        allTrackables.addAll(targetsRoverRuckus);

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);


        final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        targetsRoverRuckus.activate();

        goldDetector = new GoldAlignDetector();
        goldDetector.init(hwMap.appContext,CameraViewDisplay.getInstance(), 0, true);

        goldDetector.yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, 100);
        goldDetector.useDefaults();
        goldDetector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        vuforia.setDogeCVDetector(goldDetector);
        vuforia.enableDogeCV();
        vuforia.showDebug();
        vuforia.start();
    }

    public void getCubeDetails() {
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
//                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
//            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
//                translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
//            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
//            telemetry.addData("Gold Cube X Pos:", detector.getXPosition());
        }
//        else {
//            telemetry.addData("Visible Target", "none");
//            telemetry.addData("Gold Cube X Pos:", detector.getXPosition());
//        }


//        telemetry.update();
    }


}