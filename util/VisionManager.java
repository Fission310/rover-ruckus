package org.firstinspires.ftc.teamcode.util;


import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.disnodeteam.dogecv.scoring.MaxAreaScorer;
import com.disnodeteam.dogecv.scoring.RatioScorer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

}