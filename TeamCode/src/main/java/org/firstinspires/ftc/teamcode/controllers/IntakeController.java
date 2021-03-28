package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeWheelMode.OFF;
import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeWheelMode.ON;
import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeLiftMode.DOWN;
import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeLiftMode.UP;
import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeLatchPosition.CLOSED;
import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeLatchPosition.OPEN;

public class IntakeController extends RobotController {
    public IntakeController(OpMode opMode){
        super(opMode);
    }

    @Override
    public void execute() {
        if (gamepad2.right_trigger > .5) robot.intake(ON);
        else if (gamepad2.left_trigger > .5) robot.intake(OFF);
        else if (gamepad2.right_stick_button) robot.intake(UP);
        else if (gamepad2.left_stick_button) robot.intake(DOWN);
        else if (gamepad2.dpad_down) robot.intake(OPEN);
        else if (gamepad2.dpad_up) robot.intake(CLOSED);
    }
}
