package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.GamepadStick;
import org.firstinspires.ftc.teamcode.hardware.IMU;
import org.firstinspires.ftc.teamcode.hardware.RobotDrive;

import static java.lang.Math.abs;

@TeleOp(name="DegreeRotationOpMode", group="TeleOpModes")
public class DegreeRotationOpMode extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private RobotDrive robotDrive;
    private IMU compass;
    private float lastHeading;
    private float elapsedRotation;
    private float targetHeading;
    private boolean isRotating;


    @Override
    public void init() {
        // send message to telemetry
        telemetry.addData("Status", "Rotating initialing...");

        // declare drives
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        robotDrive = new RobotDrive(
                leftDrive, rightDrive,
                RobotDrive.DirectDrive.FONT_WHEEL_DRIVE,
                RobotDrive.EncoderMode.RUN_USING_ENCODERS
        );

        // send initialization ok message
        telemetry.addData("Status", "Rotating initialized");
        compass = new IMU(hardwareMap.get(BNO055IMU.class, "imu"));
        //rotate(90.0f);
    }

    public void init_loop() {}

    @Override
    public void start() {
        runtime.reset();
    }

    private void rotate(float degree) {
        telemetry.addData("Rotating", "Rotating of " + degree + "°");
        // try for the moment just to rotate for 3s
        lastHeading = compass.getHeadingAngleDeg();
        targetHeading = degree;
        isRotating = true;
        robotDrive.setLeftRightMotorOutputs(0.3f, -0.5f);
    }

    private void needsStopRotating() {
        if (isRotating) {
            elapsedRotation += (compass.getHeadingAngleDeg()-lastHeading);
            lastHeading = compass.getHeadingAngleDeg();
            if (abs(elapsedRotation) >= abs(targetHeading)) {
                robotDrive.setLeftRightMotorOutputs(0f, 0f);
                isRotating = false;
            }
            telemetry.addData("Elapsed", "" + elapsedRotation + "°");
        }
    }

    @Override
    public void loop() {
        compass.update();
        telemetry.addData("Current heading", "" + compass.getHeadingAngleDeg());
        GamepadStick stick = new GamepadStick(gamepad1, GamepadStick.Section.SIDE_RIGHT);
        if (stick.getY() > 0.5) {
            rotate(stick.getY()*100);
        }
        needsStopRotating();
    }
}
