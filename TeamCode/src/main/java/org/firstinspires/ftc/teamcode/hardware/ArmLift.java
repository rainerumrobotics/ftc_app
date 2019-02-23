package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ArmLift {
    private final DcMotor _armMotor;

    public ArmLift(DcMotor motor) {
        _armMotor = motor;
        _armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private static final float K_MAX_POWER = 0.7f;
    public static final int K_TICKS_PER_REVOLUTION_REV_HD_HEX_MOTOR_40 = 2240;
    public static final int K_TICKS_PER_REVOLUTION_REV_HD_HEX_MOTOR_20 = 1120;

    public void stop(){ _armMotor.setPower(0.0f); }
    public void muoviMotoreSu(){
        _armMotor.setPower(K_MAX_POWER);
    }
    public void muoviMotoreGiu(){
        _armMotor.setPower(-K_MAX_POWER);
    }
    public void muoviMotore(float power) {
        _armMotor.setPower(power * K_MAX_POWER);
    }
}