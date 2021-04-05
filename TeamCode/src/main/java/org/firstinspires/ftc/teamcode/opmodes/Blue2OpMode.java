package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeLiftMode.DOWN;
import static org.firstinspires.ftc.teamcode.internal.Robot.IntakeLiftMode.UP;
import static org.firstinspires.ftc.teamcode.internal.Robot.ShooterMode.ON;
import static org.firstinspires.ftc.teamcode.internal.Robot.ShooterMode.RESET;
import static org.firstinspires.ftc.teamcode.internal.Robot.ShooterMode.SHOOT;

@Autonomous
public class Blue2OpMode extends BlueOpMode {
    @Override
    protected void execute() {
        robot.drivePower = 1.0;

        robot.drive(1,0,0,37);

        String recognitionLabel = "";
        for (int i = 0; i < 50; i++) {
            sleep(50);
            if (robot.recognitions != null && !robot.recognitions.isEmpty()) {
                recognitionLabel = robot.recognitions.get(0).getLabel();
            }
        }

        robot.activeWebcam = robot.navigationWebcam;

        switch (recognitionLabel) {
            case "Quad": this.targetC(); break;
            case "Single": this.targetB(); break;
            default: this.targetA(); break;
        }

        this.shootHighGoal();
    }

    protected void targetA() {
        robot.drive(1,0,0,26);
        robot.drive(-1,0,0,60);
        robot.drive(0,1,0,36);
        robot.drive(1,0,28,64);
        robot.drive(-1,0,28,5);
        robot.drive(0,-1,0,15);  //change started here
    }

    protected void targetB() {
        robot.drive(1,0,-15,51);
        robot.drive(-1,0,-15,51);
        robot.drive(-1,0,0,30);
        robot.drive(0,1,0,35);
        robot.drive(1,0,0,42);
        robot.drive(1,0,9,40);
        robot.drive(-1,0,0,25);
        robot.drive(0,-1,0,39);
    }

    protected void targetC() {
        robot.drive(1,0,0,73);
        robot.drive(-1,0,0,110);
        robot.drive(0,1,0,38);
        robot.drive(1,0,0,42);
        robot.drive(1,0,25,71);
        robot.drive(-1,0,0,50);
        robot.drive(0,-1,0,12);
    }

    private void shootPowerShots() {
        robot.shooter(ON);
        robot.drive(1,0,-1,0); //robot turns to (guessed) orientation to shoot power shot target
        robot.shooter(SHOOT);
        sleep(1000);
        robot.shooter(RESET);
        sleep(1000);
        robot.drive(1,0,-6,0);
        robot.shooter(SHOOT);
        sleep(1000);
        robot.shooter(RESET);
        sleep(1000);
        robot.drive(1,0,-12,0);
        robot.shooter(SHOOT);
        sleep(1000);
        robot.shooter(RESET);
        robot.drive(1,0,0,12); //make sure to get back on line
    }

    private void shootHighGoal() {
        robot.drive(0,1,0,40);
        robot.drive(1,0,-7,9);
        robot.intake(UP);
        robot.shooter(ON);
        robot.shooter(SHOOT);
        sleep(500);
        robot.shooter(RESET);
        sleep(750);
        robot.shooter(SHOOT);
        sleep(500);
        robot.shooter(RESET);
        sleep(750);
        robot.shooter(SHOOT);
        sleep(500);
        robot.shooter(RESET);
        robot.drive(1,0,-5,16); //make sure to get back on line
        robot.intake(DOWN);
    }
}