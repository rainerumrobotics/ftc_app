package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class ArmSlave {
    ArmSlave(Servo servo1, Servo servo2){
        _armServo1=servo1;
        _armServo2=servo2;
        rotazione = servo1.getPosition();
    }

    private final Servo _armServo1;
    private final Servo _armServo2;
    private int rotazione;

    public static final float K_MAX_POWER = 1.0f;

    public boolean muoviSlaveSu(){
        if(rotazione<1) {
            _armServo1.setPosition(rotazione += 0.1);
            _armServo2.setPosition(rotazione -= 0.1);
            return false;
        }
        return true
    }
    public boolean muoviSlaveGiu(){
        if(rotazione>0) {
            _armServo1.setPosition(rotazione -= 0.1);
            _armServo2.setPosition(rotazione += 0.1);
            return false;
        }
        return true;
    }
}