package org.firstinspires.ftc.teamcode.util.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.hardware.RCConfig;

import java.util.List;

/**
 * This class handles the robot CV functions and makes TensorFlow easy in a single
 * OpMode.
 */
public class TensorFlowManager {
    /**
     * Constants for TensorFlow detectors + Vuforia Key
     */
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AZg2Pg7/////AAABmUJNdV5U3UdZuILyo65XAwgrRZJyRDvOUmdlSJISRf2lUECnalP9t4Vq+TPV2IXxriMQEqJFLiWzA30Qs7Kyx9qYi3HYgACR8ifSGJWZRgCDFa46j2xEo1xBP5S5z3bDl/7evo6bhaf3z5y9pdc1LKUVWx4woR0VN36y/IvcNI8FU4rUA5Big4AB4XKIlYpESWAYTj8hnAQSlXkKrG3CDKv7F2IDbZmtCv2SpZgfZEj+OPNlPRO41izEPeMX/svz6UoaugAfOt3M7ZwyX/+ZBZRJfgDRA7OiEPFi4aoACzWUBY2mSODV1n+BJN3+GRR2arcgSDUIJ7mTJ2nwNVdNblXSj18nOVLlS1WpLxCWEDzo\n";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    /**
     * Instance variable containing TensorFlow's locator
     */
    private TFLocation location;
    /**
     * Instance variable containing TensorFlow's detector
     */
    private TFDetector detector;

    /**
     * Default constructor for TensorFlowManager.
     */
    public TensorFlowManager() {}

    /**
     * Initializes Vuforia and starts TensorFlow detector if possible.
     * @param hwMap     robot's hardware map
     */
    public void init(HardwareMap hwMap, boolean webcam) {
        if (webcam) { this.initWebcamVuforia(hwMap); }
        else { this.initVuforia(); }
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) this.initTfod(hwMap);
    }

    /**
     * Activates TensorFlow interface.
     */
    public void start() { if (tfod != null) tfod.activate(); }
    /**
     * Deactivate TensorFlow interface.
     */
    public void stop() { if (tfod != null) tfod.shutdown(); }

    /**
     * Enum containing all possible detections of the minerals .
     */
    public enum TFDetector {
        GOLD("Gold"),
        SILVER("Silver"),
        NONE("None");

        private String detector;

        TFDetector(String detector) { this.detector = detector; }

        public String getDetectorValue() { return detector; }
    }

    /**
     * Enum containing all possible locations of the minerals .
     */
    public enum TFLocation {
        LEFT("Left", 0, 0, 0, 0),
        CENTER("Center", 0, 0, 0, 0),
        RIGHT("Right", 0, 0, 0, 0),
        NONE("None", 0, 0, 0, 0);

        private String location;
        private int craterAngle, depotAngle, craterDistance, depotDistance;

        TFLocation(String location, int craterAngle, int depotAngle, int craterDistance, int depotDistance) {
            this.location = location;
            this.craterAngle = craterAngle;
            this.depotAngle = depotAngle;
            this.craterDistance = craterDistance;
            this.depotDistance = depotDistance;
        }

        public int getCraterAngle(){ return craterAngle; }
        public int getDepotAngle(){ return depotAngle; }
        public int getCraterDistance(){ return craterDistance; }
        public int getDepotDistance(){ return depotDistance; }
    }

    /**
     * Returns updated location of mineral.
     */
    public TFLocation getLocation() {
        this.updateLocation();
        return location;
    }

    public TFLocation getDoubleMineralLocation() {
        updateTwoMineralsLocation();
        return location;
    }

    /**
     * Returns updated classification of mineral.
     */
    public TFDetector getDetector() {
        this.updateDetector();
        return detector;
    }

    /**
     * Identifies mineral based on TensorFlow's classification.
     * Updates detector in TFLocation.
     */
    private void updateDetector() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                if (updatedRecognitions.size() == 1) {
                    if (updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL)) {
                        this.detector = TFDetector.GOLD;
                    } else if (updatedRecognitions.get(0).getLabel().equals(LABEL_SILVER_MINERAL)) {
                        this.detector = TFDetector.SILVER;
                    } else {
                        this.detector = TFDetector.NONE;
                    }
                }
            }
        }
    }

    /**
     * Identifies mineral's location based on TensorFlow's classification.
     * Updates location in TFLocation.
     */
    private void updateLocation() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                if (updatedRecognitions.size() == 3) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }
                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            this.location = TFLocation.LEFT;
                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            this.location = TFLocation.RIGHT;
                        } else {
                            this.location = TFLocation.CENTER;
                        }
                    }
                } else {
                    this.location = TFLocation.NONE;
                }
            }
        }
    }

    /**
     * Identifies mineral's location based on two minerals.
     * Updates location in TFLocation.
     */
    private void updateTwoMineralsLocation() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                if (updatedRecognitions.size() == 2) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }

                    if (goldMineralX == -1 && silverMineral1X != -1) {
                        this.location = TFLocation.RIGHT;
                    } else if (goldMineralX != -1 && silverMineral1X != -1) {
                        if (goldMineralX > silverMineral1X) {
                            this.location = TFLocation.CENTER;
                        } else {
                            this.location = TFLocation.LEFT;
                        }
                    }
                } else {
                    this.location = TFLocation.NONE;
                }
            }
        }
    }

    private void updateLocationTest() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                if (updatedRecognitions.size() == 2) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;

                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }
                    if (goldMineralX == -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                        this.location = TFLocation.RIGHT;
                    } else if (goldMineralX != -1 && silverMineral1X != -1) {
                        if (goldMineralX > silverMineral1X) {
                            this.location = TFLocation.CENTER;
                        } else {
                            this.location = TFLocation.LEFT;
                        }
                    }
                }
            }
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initWebcamVuforia(HardwareMap hwMap) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hwMap.get(WebcamName.class, RCConfig.WEBCAM);

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        tfodParameters.minimumConfidence = .55;
    }

    public void vuforiaLights(boolean turn) {
        CameraDevice.getInstance().setFlashTorchMode(turn);
    }

}
