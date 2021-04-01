package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.opmodes.OpMode;

import static org.firstinspires.ftc.teamcode.internal.Robot.WobbleArmAction.FORWARD;
import static org.firstinspires.ftc.teamcode.internal.Robot.WobbleArmAction.STOP;
import static org.firstinspires.ftc.teamcode.internal.Robot.WobbleArmAction.BACKWARD;
import static org.firstinspires.ftc.teamcode.internal.Robot.WobbleLatchPosition.CLOSED;
import static org.firstinspires.ftc.teamcode.internal.Robot.WobbleLatchPosition.OPEN;

public class WobbleController extends RobotController {
    public WobbleController(OpMode opMode) {
        super(opMode);
    }

    @Override
    public void execute() {
        if (gamepad2.left_bumper) robot.wobbleArm(FORWARD);
        else if (gamepad2.right_bumper) robot.wobbleArm(BACKWARD);
        else robot.wobbleArm(STOP);
        if (gamepad2.dpad_left) robot.wobbleLatch(CLOSED);
        else if (gamepad2.dpad_right) robot.wobbleLatch(OPEN);
    }
}