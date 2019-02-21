package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.Objects;

import static com.qualcomm.ftcrobotcontroller.BuildConfig.VUFORIA_API_KEY;

public class GoldFinder {

    private final MasterVision vision;
    private TFLite tfLite;

    public GoldFinder(HardwareMap hardwareMap) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;// recommended camera direction
        parameters.vuforiaLicenseKey = VUFORIA_API_KEY;
        vision = new MasterVision(parameters, hardwareMap, true, MasterVision.TFLiteAlgorithm.INFER_NONE);
    }

    /** Start preview and vision recognition after init is pressed at DS. */
    public void start() {
        vision.init();// enables the camera overlay. this will take a couple of seconds
        vision.enable();// enables the tracking algorithms. this might also take a little time
    }

    /**
     * Pause vision recognition after start is pressed at DS but keep preview.
     * Use getLastGoldPosition and companions.
     */
    public void pause() {
        vision.disable();// disables tracking algorithms. this will free up your phone's processing power for other jobs.
    }

    /** Terminate preview and vision recognition at the end of operation. */
    public void stop() {
        vision.shutdown();
    }

    public SampleRandomizedPositions getLastGoldPosition() {
        if(tfLite == null) {
            tfLite = vision.getTfLite();
        }
        return tfLite.getLastKnownSampleOrder();
    }

    /**
     * Returns deviation on the X axis from the center.
     * @return pixels from the center or Integer.MIN_VALUE if invalid.
     */
    public int getLastGoldPositionCenterX() {
        final int INVALID_POSITION = Integer.MIN_VALUE;
        if(tfLite == null) {
            tfLite = vision.getTfLite();
        }
        final Integer POS_X = tfLite.getLastGoldPositionX();
        final Integer MAX = tfLite.getPositionX_Max();
        if((MAX == null) || (POS_X == null)) {
            return INVALID_POSITION;
        }
        final Integer MID = MAX / 2;
        if(Objects.equals(POS_X, tfLite.getPositionX_Left())) {
            return -MID;
        } else if(Objects.equals(POS_X, tfLite.getPositionX_Right())) {
            return +MID;
        } else if(Objects.equals(POS_X, tfLite.getPositionX_Center())) {
            return 0;
        }
        return POS_X - MID;
    }
}
