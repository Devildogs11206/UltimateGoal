package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.internal.Robot;

public abstract class OpMode extends LinearOpMode {
    private boolean calibrate;

    public Robot robot;

    public OpMode() {
        this(true);
    }

    public OpMode(boolean calibrate) {
        this.calibrate = calibrate;
    }

    @Override
    public void runOpMode() {
        robot = new Robot(this);
        robot.init();
        if (calibrate) robot.calibrate();
        waitForStart();
        sleep(250);
        robot.start();
        sleep(250);
        execute();
    }

    public boolean isActive() {
        yield();
        return opModeIsActive();
    }

    public boolean isStopping() {
        yield();
        return isStopRequested() || gamepad1.back || gamepad2.back;
    }

    protected abstract void execute();

    private void yield() {
        Thread.yield();
    }
}