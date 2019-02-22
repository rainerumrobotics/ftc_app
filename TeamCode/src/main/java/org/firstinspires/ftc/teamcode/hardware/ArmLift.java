package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ArmMotor {
    private final DcMotor _armMotor;

    ArmLift(DcMotor motor) {
        _armMotor = motor;
        _armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static final float K_MAX_POWER = 1.0f;
    public static final int K_TICKS_PER_REVOLUTION_REV_HD_HEX_MOTOR_40 = 2240;
    public static final int K_TICKS_PER_REVOLUTION_REV_HD_HEX_MOTOR_20 = 1120;

    public void stop(){ _armMotor.setPower(0.0f); }
    public void muoviMotoreSu(){
        _armMotor.setPower(K_MAX_POWER);
    }
    public void muoviMotoreGiu(){
        _armMotor.setPower(-K_MAX_POWER);
    }
}