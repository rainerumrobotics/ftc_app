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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.GamepadStick;
import org.firstinspires.ftc.teamcode.hardware.VerticalLift;
import com.qualcomm.robotcore.hardware.TouchSensor;


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

@TeleOp(name="TeleOpProva", group="TeleOpModes")
//@Disabled
public class TeleOpProva extends OpMode {
    /*
     * TODO: Enable OpMode and configure your robot setup in the RC-App.
     *
     * To enable this opmode you can either remove the @Disabled annotation,
     * or place a // comment in front of it.
     *
     * To create a robot configuration you will need to upload this TeamCode project to the
     * Robot Controller (RC) phone. You can select/create then any configuration from the menu
     * Configure Robot.
     * If you install this app for the first time you will need to create a new configuration.
     * This can be done by either connecting USB controller hub to the phone and/or by performing
     * a scan of the current hardware, or by selecting a preinstalled template like SquareBot for
     * example, or an combination of both. Save the configuration with some name.
     *
     * Finally activate the configuration by selecting its name on the list and press Activate.
     *
     * Bot hardware configuration for REV Robotics Expansion Hub:
     *  - Motor Controller:
     *    Port 1 - Left DC motor, "left_drive", REV Robotics Core Hex Motor
     *    Port 2 - Right DC motor, "right_drive", REV Robotics Core Hex Motor
     *  - Device Interface Module (not used in this code):
     *    Analog Port 0 - Optical distance sensor
     *    Digital Port 0 - Touch sensor
     *
     * Bot hardware sketch:
     *  ===        ===
     * .-+----------+-.
     * |        [M_FL]|
     * |     -->      |
     * |        [M_FR]|
     * `-+----------+-´
     *  ===        ===
     */

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private VerticalLift verticalLift;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        DcMotor liftMotor = hardwareMap.get(DcMotor.class, "vertical_lift");
        TouchSensor touchSensor = hardwareMap.get(TouchSensor.class, "lift_end_stop");
        // Use front wheel direct drive with encoders for constant speed driving.
        verticalLift = new VerticalLift(liftMotor, touchSensor);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Choose to drive using either Tank Mode, or Arcade Mode
        // Comment out the method that's not used.  The default below is Arcade.

        // Arcade Mode uses one stick to go forward/backward, and left/right to turn.
        GamepadStick stick = new GamepadStick(gamepad1, GamepadStick.Section.SIDE_RIGHT);

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        //robotDrive.tankDrive(gamepad1);

        // Get calculated power from motor drivers
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
