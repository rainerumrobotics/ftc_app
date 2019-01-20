package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

/**
 * Operations on a robot drive train.
 *
 * The robot drive class handles basic driving for a robot. Currently, 2 and 4 motor drive trains
 * are supported. Motor definitions are passed supplied on creation of the class. Those are used for
 * either the drive function (intended for hand created drive code, such as autonomous) or with the
 * Arcade functions intended to be used for Tele Operator Control driving.
 */
public class RobotDrive {

    /** The location of a motor on the robot for the purpose of driving. */
    enum MotorType {
        /** Front left */
        LOCATION_FRONT_LEFT,
        /** Front right */
        LOCATION_FRONT_RIGHT,
        /** Rear left */
        LOCATION_REAR_LEFT,
        /** Rear right */
        LOCATION_REAR_RIGHT
    }

    public static final float K_DEFAULT_SENSITIVITY = 0.5f;
    public static final float K_DEFAULT_MAX_OUTPUT = 1.0f;

    private final DcMotor _frontLeftMotor;
    private final DcMotor _rearLeftMotor;
    private final DcMotor _frontRightMotor;
    private final DcMotor _rearRightMotor;

    private float _maxOutput;
    private float _sensitivity;

    /**
     * Constructor for RobotDrive.
     *
     * Two motors can be passed to this constructor to implement a two wheel drive system,
     * respectively.
     * @param leftMotor
     * @param rightMotor
     */
    public RobotDrive(DcMotor leftMotor, DcMotor rightMotor) {
        this(null, leftMotor, null, rightMotor);
    }

    /**
     * Constructor for RobotDrive.
     *
     * Four motors can be passed to this constructor to implement a four wheel drive system,
     * respectively.
     * @param frontLeftMotor
     * @param rearLeftMotor
     * @param frontRightMotor
     * @param rearRightMotor
     */
    public RobotDrive(DcMotor frontLeftMotor, DcMotor rearLeftMotor,
                      DcMotor frontRightMotor, DcMotor rearRightMotor) {
        // Defaults:
        _frontLeftMotor = frontLeftMotor;
        _rearLeftMotor = rearLeftMotor;
        _frontRightMotor = frontRightMotor;
        _rearRightMotor = rearRightMotor;
        _maxOutput = RobotDrive.K_DEFAULT_MAX_OUTPUT;
        _sensitivity = RobotDrive.K_DEFAULT_SENSITIVITY;
        // Start off not moving.
        drive(0, 0);
    }

    /**
     * Drive the motors at "outputMagnitude" and "curve".
     *
     * Both outputMagnitude and curve are -1.0 to +1.0 values, where 0.0 represents stopped and not
     * turning. 'curve < 0' will turn left and 'curve > 0' will turn right.
     *
     * The algorithm for steering provides a constant turn radius for any normal speed range, both
     * forward and backward. Increasing sensitivity causes sharper turns for fixed values of curve.
     *
     * This function will most likely be used in an autonomous routine.
     *
     * @param outputMagnitude: The speed setting for the outside wheel in a turn, forward or
     *                      backwards, +1 to -1.
     * @param curve: The rate of turn, constant for different forward speeds. Set 'curve < 0' for
     *            left turn or 'curve > 0' for right turn.
     *
     * Set 'curve = e^(-r/w)' to get a turn radius r for wheelbase w of your robot.
     * Conversely, turn radius 'r = -ln(curve)*w' for a given value of curve and wheelbase w.
     */
    public void drive(float outputMagnitude, float curve) {
        float leftOutput;
        float rightOutput;
        if(curve < 0) {
            double value = Math.log(-curve);
            double ratio = (value - _sensitivity) / (value + _sensitivity);
            if(ratio == 0) {
                ratio = 0.0000000001;
            }
            leftOutput = (float)(outputMagnitude / ratio);
            rightOutput = outputMagnitude;
        } else if(curve > 0) {
            double value = Math.log(curve);
            double ratio = (value - _sensitivity) / (value + _sensitivity);
            if(ratio == 0) {
                ratio = 0.0000000001;
            }
            leftOutput = outputMagnitude;
            rightOutput = (float)(outputMagnitude / ratio);
        } else {
            leftOutput = outputMagnitude;
            rightOutput = outputMagnitude;
        }
        setLeftRightMotorOutputs(leftOutput, rightOutput);
    }

    /**
     * Provide tank steering using the stored robot configuration.
     *
     * Either two joysticks (with optional specified axis) or two raw values may be passed, along
     * with an optional squaredInputs boolean.
     *
     * The valid positional combinations are:
     * - gamepad
     * - gamepad, squaredInputs
     * - leftStick, rightStick
     * - leftStick, rightStick, squaredInputs
     *
     * @param gamepad: The gamepad to control the robot.
     */
    public void tankDrive(Gamepad gamepad) {
        tankDrive(
                new GamepadStick(gamepad, GamepadStick.Section.SIDE_LEFT),
                new GamepadStick(gamepad, GamepadStick.Section.SIDE_RIGHT)
        );
    }

    /**
     * Provide tank steering using the stored robot configuration.
     *
     * Either two joysticks (with optional specified axis) or two raw values may be passed, along
     * with an optional squaredInputs boolean.
     *
     * The valid positional combinations are:
     * - gamepad
     * - gamepad, squaredInputs
     * - leftStick, rightStick
     * - leftStick, rightStick, squaredInputs
     *
     * @param gamepad: The gamepad to control the robot.
     * @param squaredInputs: Setting this parameter to true decreases the sensitivity at lower
     *                    speeds. Defaults to true if unspecified.
     */
    public void tankDrive(Gamepad gamepad, boolean squaredInputs) {
        tankDrive(
                new GamepadStick(gamepad, GamepadStick.Section.SIDE_LEFT),
                new GamepadStick(gamepad, GamepadStick.Section.SIDE_RIGHT),
                squaredInputs
        );
    }

    /**
     * Provide tank steering using the stored robot configuration.
     *
     * Either two joysticks (with optional specified axis) or two raw values may be passed, along
     * with an optional squaredInputs boolean.
     *
     * The valid positional combinations are:
     * - gamepad
     * - gamepad, squaredInputs
     * - leftStick, rightStick
     * - leftStick, rightStick, squaredInputs
     *
     * @param leftStick: The joystick to control the left side of the robot.
     * @param rightStick: The joystick to control the right side of the robot.
     */
    public void tankDrive(GamepadStick leftStick, GamepadStick rightStick) {
        tankDrive(leftStick, rightStick, true);
    }

    /**
     * Provide tank steering using the stored robot configuration.
     *
     * Either two joysticks (with optional specified axis) or two raw values may be passed, along
     * with an optional squaredInputs boolean.
     *
     * The valid positional combinations are:
     * - gamepad
     * - gamepad, squaredInputs
     * - leftStick, rightStick
     * - leftStick, rightStick, squaredInputs
     *
     * @param leftStick: The joystick to control the left side of the robot.
     * @param rightStick: The joystick to control the right side of the robot.
     * @param squaredInputs: Setting this parameter to true decreases the sensitivity at lower
     *                    speeds. Defaults to true if unspecified.
     */
    public void tankDrive(GamepadStick leftStick, GamepadStick rightStick, boolean squaredInputs) {
        float leftValue = leftStick.getY();
        float rightValue = rightStick.getY();
        //left -> Y, right -> Y, squaredInputs
        // square the inputs (while preserving the sign) to increase fine
        // control while permitting full power

        // Local variables to hold the computed PWM values for the motors.
        leftValue = Range.clip(leftValue, -1.0f, 1.0f);
        rightValue = Range.clip(rightValue, -1.0f, 1.0f);

        if(squaredInputs) {
            // Square the inputs (while preserving the sign) to increase fine control while
            // permitting full power.
            leftValue = Math.copySign(leftValue * leftValue, leftValue);
            rightValue = Math.copySign(rightValue * rightValue, rightValue);
        }
        setLeftRightMotorOutputs(leftValue, rightValue);
    }

    /**
     * Provide tank steering using the stored robot configuration.
     *
     * The valid positional combinations are:
     * - stick
     * - stick, squaredInputs
     *
     * @param stick: The joystick to use for Arcade single-stick driving. The Y-axis will be
     *            selected for forwards/backwards and the X-axis will be selected for rotation rate.
     */
    public void arcadeDrive(GamepadStick stick) {
        arcadeDrive(stick, true);
    }

    /**
     * Provide tank steering using the stored robot configuration.
     *
     * The valid positional combinations are:
     * - stick
     * - stick, squaredInputs
     *
     * @param stick: The joystick to use for Arcade single-stick driving. The Y-axis will be
     *            selected for forwards/backwards and the X-axis will be selected for rotation rate.
     * @param squaredInputs: Setting this parameter to true decreases the sensitivity at lower
     *                    speeds. Defaults to True if unspecified.
     */
    public void arcadeDrive(GamepadStick stick, boolean squaredInputs) {
        float leftMotorSpeed;
        float rightMotorSpeed;

        // Get value from stick provided.
        float moveValue = stick.getY();
        float rotateValue = stick.getX();

        // Local variables to hold the computed PWM values for the motors.
        moveValue = Range.clip(moveValue, -1.0f, 1.0f);
        rotateValue = Range.clip(rotateValue, -1.0f, 1.0f);

        if(squaredInputs) {
            // Square the inputs (while preserving the sign) to increase fine control while
            // permitting full power.
            moveValue = Math.copySign(moveValue * moveValue, moveValue);
            rotateValue = Math.copySign(rotateValue * rotateValue, rotateValue);
        }
        if(moveValue > 0.0f) {
            if(rotateValue > 0.0f) {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = Math.max(moveValue, rotateValue);
            } else {
                leftMotorSpeed = Math.max(moveValue, -rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            }
        } else {
            if(rotateValue > 0.0f) {
                leftMotorSpeed = -Math.max(-moveValue, rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            } else {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
            }
        }
        setLeftRightMotorOutputs(leftMotorSpeed, rightMotorSpeed);
    }

    /**
     * Set the speed of the right and left motors.
     *
     * This is used once an appropriate drive setup function is called such as
     * arcadeDrive(). The motors are set to "leftSpeed" and "rightSpeed"
     * and includes flipping the direction of one side for opposing motors.
     *
     * @param leftOutput: The speed to send to the left side of the robot.
     * @param rightOutput: The speed to send to the right side of the robot.
     */
    public void setLeftRightMotorOutputs(float leftOutput, float rightOutput) {
        leftOutput = Range.clip(leftOutput, -1.0f, 1.0f) * _maxOutput;
        rightOutput = Range.clip(rightOutput, -1.0f, 1.0f) * _maxOutput;

        if(_frontLeftMotor != null)
            _frontLeftMotor.setPower(leftOutput);
        _rearLeftMotor.setPower(leftOutput);

        if(_frontRightMotor != null)
            _frontRightMotor.setPower(-rightOutput);
        _rearRightMotor.setPower(-rightOutput);
    }

    /**
     * Invert a motor direction.
     *
     * This is used when a motor should run in the opposite direction as the drive code would
     * normally run it. Motors that are direct drive would be inverted, the drive code assumes that
     * the motors are geared with one reversal.
     *
     * @param motor: The motor type location to invert.
     * @param isInverted: true if the motor should be inverted when operated.
     */
    public void setMotorInverted(MotorType motor, boolean isInverted) {
        final DcMotor.Direction direction =
                isInverted ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD;
        if(motor == MotorType.LOCATION_FRONT_LEFT)
            _frontLeftMotor.setDirection(direction);
        else if(motor == MotorType.LOCATION_FRONT_RIGHT)
            _frontRightMotor.setDirection(direction);
        else if(motor == MotorType.LOCATION_REAR_LEFT)
            _rearLeftMotor.setDirection(direction);
        else if(motor == MotorType.LOCATION_REAR_RIGHT)
            _rearRightMotor.setDirection(direction);
    }

    /**
     * Set the turning sensitivity.
     *
     * This only impacts the drive() method.
     * @param sensitivity: Effectively sets the turning sensitivity (or turn radius).
     */
    public void setSensitivity(float sensitivity) {
        _sensitivity = sensitivity;
    }

    /**
     * Configure the scaling factor for using RobotDrive with motor controllers.
     *
     * Can be used to compensate the dropping voltage from the batteries during usage of the robot.
     * @param maxOutput: Multiplied with the output percentage computed by the drive functions.
     */
    public void setMaxOutput(float maxOutput) {
        _maxOutput = maxOutput;
    }

    public void stopMotors() {
        if(_frontLeftMotor != null)
            _frontLeftMotor.setPower(0.0);
        if(_frontRightMotor != null)
            _frontRightMotor.setPower(0.0);
        if(_rearLeftMotor != null)
            _rearLeftMotor.setPower(0.0);
        if(_rearRightMotor != null)
            _rearRightMotor.setPower(0.0);
    }

    public int getNumMotors() {
        int motors = 0;
        if(_frontLeftMotor != null)
            motors += 1;
        if(_frontRightMotor != null)
            motors += 1;
        if(_rearLeftMotor != null)
            motors += 1;
        if(_rearRightMotor != null)
            motors += 1;
        return motors;
    }
}
