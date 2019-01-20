package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadStick {

    public enum Section { SIDE_LEFT, SIDE_RIGHT }

    private final Gamepad _gamepad;
    private final Section _stick;

    public GamepadStick(Gamepad controller, Section side) {
        _gamepad = controller;
        _stick = side;
    }

    public float getY() {
        if(_stick == Section.SIDE_LEFT)
            return _gamepad.left_stick_y;
        return _gamepad.right_stick_y;
    }

    public float getX() {
        if(_stick == Section.SIDE_RIGHT)
            return _gamepad.right_stick_x;
        return _gamepad.left_stick_x;
    }
}
