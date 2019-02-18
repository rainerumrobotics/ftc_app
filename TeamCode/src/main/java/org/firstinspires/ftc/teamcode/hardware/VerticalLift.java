package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class VerticalLift {

    public static final float K_MAX_POWER = 1.0f;

    private final DcMotor _liftMotor;
    private final TouchSensor _bottomEndStop;

    public VerticalLift(DcMotor liftMotor, TouchSensor bottomEndStop) {
        _liftMotor = liftMotor;
        _bottomEndStop = bottomEndStop;
    }

    public void moveUp() {
        //_liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //_liftMotor.setTargetPosition(1000);
        _liftMotor.setPower(K_MAX_POWER);
    }

    public void stop() {
        _liftMotor.setPower(0.0f);
    }

    public void moveDown() {
        if(isBottomEndStopHit()) {
            stop();
        } else {
            _liftMotor.setPower(-K_MAX_POWER);
        }
    }

    public boolean isBottomEndStopHit() {
        //_liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //_liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return _bottomEndStop.isPressed();
    }

}
