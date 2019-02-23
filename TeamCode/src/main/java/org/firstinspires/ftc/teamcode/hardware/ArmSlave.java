package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmSlave {

    private final CRServo _armServo1;
    private final CRServo _armServo2;
    private double rotazione1;
    private double rotazione2;

    public static final float K_MAX_POWER = 1.0f;

    public ArmSlave(CRServo servo1, CRServo servo2) {
        _armServo1=servo1;
        _armServo2=servo2;
        servo2.setDirection(DcMotorSimple.Direction.REVERSE);
        //rotazione1 = servo1.getPosition();
        //rotazione2 = servo2.getPosition();
    }

    public void muoviSlaveSu() {
        rotazione1 = 1.0f;
        rotazione2 = 1.0f;
        /*if(rotazione<1) {
            _armServo1.setPosition(rotazione += 0.1);
            _armServo2.setPosition(rotazione -= 0.1);
            return false;
        }*/
    }
    public void muoviSlaveGiu() {
        rotazione1 = 0.1;
        rotazione2 = 0.1;
        /*if(rotazione>0) {
            _armServo1.setPosition(rotazione += 0.1);
            _armServo2.setPosition(rotazione -= 0.1);
            return false;
        }*/
    }

    public void updateServos() {
        _armServo1.setPower(rotazione1);
        _armServo2.setPower(rotazione2);

    }

    public void stop() {
        rotazione1=0f;
        rotazione2=0f;
    }

    public double testPosition1() {
        return _armServo1.getPower();
    }

    public double testPosition2() {
        return _armServo2.getPower();
    }
}