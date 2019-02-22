package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Arm {
    public ArmLift lift;
    public ArmSlave slave;
    private TouchSensor button;
    private int rotazione;

    public Arm(DcMotor armMotor, TouchSensor bottomEndStop, Servo servo1, Servo servo2, int r) {
        lift = new ArmLift(armMotor);
        button = bottomEndStop;
        slave = new ArmSlave(servo1,servo2);
        rotazione=r;
    }

    public int getRotazione(){ return rotazione; }

    public void stop(){
        lift.stop();
    }

    public boolean muoviBraccioSu(){
        if(!button.isPressed()) {
            lift.muoviMotoreSu();
            return false;
        } else {
            stop();
            return false;
        }
       // while((lift.getOccupato() || slave.getOccupato()) && !bottone.getPressed());
       // stop();
    }
    public boolean muoviBraccioGiu(){
        if(!button.isPressed()) {
            lift.muoviMotoreGiu();
            return false;
        } else {
            stop();
            return false;
        }
       // while((lift.getOccupato() || slave.getOccupato()) && !bottone.getPressed());
       // stop();
    }
    public void muoviCestaSu(){
        slave.muoviSlaveSu();
    }
    public void muoviCestaGiu(){
        slave.muoviSlaveGiu();
    }
}