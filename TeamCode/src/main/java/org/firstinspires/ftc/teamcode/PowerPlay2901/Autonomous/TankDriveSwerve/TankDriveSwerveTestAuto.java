package org.firstinspires.ftc.teamcode.PowerPlay2901.Autonomous.TankDriveSwerve;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Dwayne Tank Drive Test Auto")
public class TankDriveSwerveTestAuto extends TankDriveSwerveBaseAuto{

    @Override
    public void runOpMode() throws InterruptedException {
        moveInches(10);
        moveInches(-2);
        turnByAngle(90);
        moveInches(4);
        turnByAngle(180);
        moveInches(-2);
        turnByAngle(-45);
        moveInches(8.5);
    }
}
