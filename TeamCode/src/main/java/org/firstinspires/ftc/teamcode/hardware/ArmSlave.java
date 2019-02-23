package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class ArmSlave {

    private final Servo _armServo1;
    private final Servo _armServo2;
    private double rotazione1;
    private double rotazione2;

    public static final float K_MAX_POWER = 1.0f;

    public ArmSlave(Servo servo1, Servo servo2) {
        _armServo1=servo1;
        _armServo2=servo2;
        servo2.setDirection(Servo.Direction.REVERSE);
        rotazione1 = servo1.getPosition();
        rotazione2 = servo2.getPosition();
    }

    public void muoviSlaveSu() {
        rotazione1 = 0.9;
        rotazione2 = 0.1;
        /*if(rotazione<1) {
            _armServo1.setPosition(rotazione += 0.1);
            _armServo2.setPosition(rotazione -= 0.1);
            return false;
        }*/
    }
    public void muoviSlaveGiu() {
        rotazione1 = 0.1;
        rotazione2 = 0.9;
        /*if(rotazione>0) {
            _armServo1.setPosition(rotazione += 0.1);
            _armServo2.setPosition(rotazione -= 0.1);
            return false;
        }*/
    }

    public void updateServos() {
        _armServo1.setPosition(rotazione1);
        _armServo2.setPosition(rotazione2);
    }

    public double testPosition1() {
        return _armServo1.getPosition();
    }

    public double testPosition2() {
        return _armServo2.getPosition();
    }
}