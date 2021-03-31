package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeWheelMode.FORWARD;
import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeWheelMode.NEUTRAL;
import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeLiftMode.DOWN;
import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeLiftMode.UP;
import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeLatchPosition.CLOSED;
import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeLatchPosition.OPEN;
import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeWheelMode.REVERSE;

public class IntakeController extends RobotController {
    public IntakeController(OpMode opMode){
        super(opMode);
    }

    @Override
    public void execute() {
        //wheel

        if (gamepad2.right_trigger > .5 && robot.intakeWheelMode == NEUTRAL) robot.intake(FORWARD);
        else if (gamepad2.left_trigger > .5 && robot.intakeWheelMode == FORWARD) robot.intake(NEUTRAL);
        else if (gamepad2.left_trigger > .5 && robot.intakeWheelMode == NEUTRAL) robot.intake(REVERSE);
        else if (gamepad2.right_trigger > .5 && robot.intakeWheelMode == REVERSE) robot.intake(NEUTRAL);

        //latch

        else if (gamepad2.right_stick_button) robot.intake(OPEN);
        else if (gamepad2.left_stick_button) robot.intake(CLOSED);

        //lift

        else if (gamepad2.dpad_down) robot.intake(DOWN);
        else if (gamepad2.dpad_up) robot.intake(UP);
    }
}
