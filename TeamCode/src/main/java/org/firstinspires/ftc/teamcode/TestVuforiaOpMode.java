package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.GoldFinder;
import org.firstinspires.ftc.teamcode.vision.SampleRandomizedPositions;

import static com.qualcomm.ftcrobotcontroller.BuildConfig.VUFORIA_API_KEY;

@TeleOp(name="TestVuforiaOpMode", group="TestTeleOpModes")
//@Disabled
public class TestVuforiaOpMode extends LinearOpMode{

    private GoldFinder goldFinder;
    private SampleRandomizedPositions goldPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        goldFinder = new GoldFinder(hardwareMap);
        goldFinder.start(); // Takes some seconds
        waitForStart();
        goldFinder.pause(); // disables tracking algorithms. this will free up your phone's processing power for other jobs.

        goldPosition = goldFinder.getLastGoldPosition();
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
            telemetry.addLine("gold deviation X = " + goldFinder.getLastGoldPositionCenterX());
            telemetry.update();
        }
        goldFinder.stop();
    }
}