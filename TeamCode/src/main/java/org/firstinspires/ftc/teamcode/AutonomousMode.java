/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotDrive;
import org.firstinspires.ftc.teamcode.vision.GoldFinder;
import org.firstinspires.ftc.teamcode.vision.SampleRandomizedPositions;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="CoffeleMode", group="AutonomousModes")
//@Disabled
public class AutonomousMode extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private RobotDrive robotDrive;
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor winchDrive;
    private GoldFinder goldFinder;
    private SampleRandomizedPositions goldPosition;
    private Boolean PHASE1 = false;
    private Boolean PHASE2 = false;
    private Boolean PHASE3 = false;
    private int PHASE1DURATION = 9200;
    private int PHASE2DURATION = PHASE1DURATION + 300;

    @Override
    public void init() {
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        winchDrive = hardwareMap.get(DcMotor.class,"winch_drive");
        robotDrive = new RobotDrive(
                leftDrive, rightDrive,
                RobotDrive.DirectDrive.FONT_WHEEL_DRIVE,
                RobotDrive.EncoderMode.RUN_USING_ENCODERS
        );
    }


    @Override
    public void start() {
        runtime.reset();
        winchDrive.setPower(0.2f);
        PHASE1 = true;
    }


    @Override
    public void loop() {
        // divide this into steps
        // step 1: get down
        if (PHASE1) {
            if (runtime.milliseconds() >= PHASE1DURATION) {
                winchDrive.setPower(0.0f);
                PHASE1 = false;
                PHASE2 = true;
            }
        }
        // step 2: rotate
        else if (PHASE2) {
            if (runtime.milliseconds() <= PHASE2DURATION) {
                robotDrive.setLeftRightMotorOutputs(-0.3f, 0.2f);
            }
            else {
                robotDrive.setLeftRightMotorOutputs(0.0f, 0.0f);
            }
        }
    }
}
