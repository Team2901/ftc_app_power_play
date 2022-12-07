package org.firstinspires.ftc.teamcode.PowerPlay2901.Autonomous.TankDriveSwerve;

public class TankDriveSwerveTestAuto extends TankDriveSwerveBaseAuto{

    @Override
    public void runOpMode() throws InterruptedException {
        getOffWall();
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
