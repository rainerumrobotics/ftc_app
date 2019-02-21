package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class VerticalLift {

    public static final float K_MAX_POWER = 1.0f;
    public static final int K_TICKS_PER_REVOLUTION_REV_HD_HEX_MOTOR_40 = 2240;
    public static final int K_TICKS_PER_REVOLUTION_REV_HD_HEX_MOTOR_20 = 1120;
    public static final int K_SW_MAX_END_STOP_TICKS = 200;
    public static final int K_HOMING_BUMP_TICKS = 100;

    private final DcMotor _liftMotor;
    private final TouchSensor _bottomEndStop;
    private boolean _isHomed = false;

    public VerticalLift(DcMotor liftMotor, TouchSensor bottomEndStop) {
        _liftMotor = liftMotor;
        _bottomEndStop = bottomEndStop;
        _liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveUp() {
        //_liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //_liftMotor.setTargetPosition(1000);
        _liftMotor.setPower(K_MAX_POWER);
    }

    public void moveDOwn() {
        _liftMotor.setPower(-K_MAX_POWER);
    }

    public void moveToMax() {
        if(!_isHomed) {
            return;
        }
        moveToPositionTicks(K_SW_MAX_END_STOP_TICKS);
    }

    public void stop() {
        _liftMotor.setPower(0.0f);
    }

    public boolean moveDown() {
        if(isBottomEndStopHit()) {
            stop();
            return true;
        } else {
            _liftMotor.setPower(-K_MAX_POWER);
            return false;
        }
    }

    public void moveToMin() {
        while(!moveDown());
        _liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean isLiftHomed() {
        return _isHomed;
    }

    public void doHoming() {
        if(_isHomed) {
            return;
        }
        _liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        moveToPositionTicks(K_HOMING_BUMP_TICKS);
        moveToMin();
        _isHomed = true;
    }

    private void moveToPositionTicks(int ticks) {
        _liftMotor.setTargetPosition(ticks);
        _liftMotor.setPower(K_MAX_POWER);
        while(_liftMotor.isBusy());
        stop();
        _liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isBottomEndStopHit() {
        //_liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //_liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _isHomed = true;
        return _bottomEndStop.isPressed();
    }

    public void testAssembly() {
        if(_isHomed) {
            _isHomed = false;
        }
        doHoming();
        moveToMax();
        moveToMin();
    }
}
