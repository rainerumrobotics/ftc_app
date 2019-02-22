package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.TouchSensor;

public class Arm{
    private ArmLift lift;
    private ArmSlave slave;
    private TouchSensor button;
    private int rotazione;

    Arm(DcMotor armMotor, TouchSensor bottomEndStop, Servo servo1, Servo servo2, int r) {
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
        if(!buttonPressed()) {
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
        if(!buttonPressed()) {
            lift.muoviMotoreGiu();
            return false;
        } else {
            stop();
            return false;
        }
       // while((lift.getOccupato() || slave.getOccupato()) && !bottone.getPressed());
       // stop();
    }
    public boolean muoviCestaSu(){
        slave.muoviSlaveSu();
    }
    public boolean muoviCestaGiu(){
        slave.muoviSlaveGiu();
    }
}