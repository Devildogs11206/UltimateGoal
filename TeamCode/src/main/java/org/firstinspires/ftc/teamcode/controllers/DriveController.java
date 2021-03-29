package org.firstinspires.ftc.teamcode.controllers;

import android.icu.util.Measure;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

public class DriveController extends RobotController {
    private static final double HIGH = 1;
    private static final double MEDIUM = .5;
    private static final double LOW = .2;

    public DriveController(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void execute() {
        if (gamepad1.dpad_up && robot.drivePower == LOW) robot.drivePower = MEDIUM;
        else if (gamepad1.dpad_up && robot.drivePower == MEDIUM) robot.drivePower = HIGH;
        else if (gamepad1.dpad_down && robot.drivePower == HIGH) robot.drivePower = MEDIUM;
        else if (gamepad1.dpad_down && robot.drivePower == MEDIUM) robot.drivePower = LOW;

        robot.drive(
           -gamepad1.left_stick_y,
            gamepad1.left_stick_x,
            gamepad1.right_stick_x
        );
    }
}