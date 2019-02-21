package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.vision.MasterVision;
import org.firstinspires.ftc.teamcode.vision.SampleRandomizedPositions;
import org.firstinspires.ftc.teamcode.vision.TFLite;

import java.util.Objects;

import static com.qualcomm.ftcrobotcontroller.BuildConfig.VUFORIA_API_KEY;

@TeleOp(name="TestVuforiaOpMode", group="TestTeleOpModes")
//@Disabled
public class TestVuforiaOpMode extends LinearOpMode{
    MasterVision vision;
    SampleRandomizedPositions goldPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;// recommended camera direction
        parameters.vuforiaLicenseKey = VUFORIA_API_KEY;

        vision = new MasterVision(parameters, hardwareMap, true, MasterVision.TFLiteAlgorithm.INFER_NONE);
        vision.init();// enables the camera overlay. this will take a couple of seconds
        vision.enable();// enables the tracking algorithms. this might also take a little time

        waitForStart();

        vision.disable();// disables tracking algorithms. this will free up your phone's processing power for other jobs.

        goldPosition = vision.getTfLite().getLastKnownSampleOrder();

        while(opModeIsActive()){
            telemetry.addData("goldPosition was", goldPosition);// giving feedback

            switch (goldPosition){ // using for things in the autonomous program
                case LEFT:
                    telemetry.addLine("going to the left");
                    break;
                case CENTER:
                    telemetry.addLine("going straight");
                    break;
                case RIGHT:
                    telemetry.addLine("going to the right");
                    break;
                case UNKNOWN:
                    telemetry.addLine("staying put");
                    break;
            }
            //telemetry.addLine("positionX = " + getGoldPosition());
            }

            telemetry.update();
        }

        vision.shutdown();
    }

    /*@SuppressWarnings("ConstantConditions")
    private int getGoldPosition() {
        final int INVALID_POSITION = Integer.MIN_VALUE;
        TFLite tfLite = vision.getTfLite();
        Integer posX = tfLite.getLastGoldPositionX();
        final Integer MAX = tfLite.getPositionXMax();
        final Integer MID = MAX / 2;
        if(posX != null) {
            return INVALID_POSITION;
        }
        if(Objects.equals(posX, tfLite.getPositionXMid())) {
            return 0;
        }
        return tfLite.getLastGoldPositionX() - MID;
    }*/
}