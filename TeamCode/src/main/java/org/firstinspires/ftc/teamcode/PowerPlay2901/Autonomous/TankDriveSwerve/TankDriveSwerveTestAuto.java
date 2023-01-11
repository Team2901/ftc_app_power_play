package org.firstinspires.ftc.teamcode.PowerPlay2901.Autonomous.TankDriveSwerve;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Dwayne Tank Drive Test Auto")
public class TankDriveSwerveTestAuto extends TankDriveSwerveBaseAuto{


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this.hardwareMap);

        waitForStart();
        moveInches(10);
    }
}
