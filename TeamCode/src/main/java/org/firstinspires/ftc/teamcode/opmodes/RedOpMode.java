package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.internal.Alliance;

import static org.firstinspires.ftc.teamcode.internal.Alliance.RED;

public abstract class RedOpMode extends TeleOpMode {
    @Override
    protected Alliance getAlliance() {
        return RED;
    }
}