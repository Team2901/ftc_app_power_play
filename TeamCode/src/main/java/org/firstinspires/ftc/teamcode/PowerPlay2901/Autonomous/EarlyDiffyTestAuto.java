package org.firstinspires.ftc.teamcode.PowerPlay2901.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "EarlyDiffyTestAuto")
public class EarlyDiffyTestAuto extends EarlyDiffyBaseAuto{
    public void runOpMode() throws InterruptedException{
        turnByAngle(0);
        turnByAngle(90);
        turnByAngle(-90);
        turnByAngle(0);
    }
}
