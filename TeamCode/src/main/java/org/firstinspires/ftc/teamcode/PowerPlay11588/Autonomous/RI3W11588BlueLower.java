package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "RI3W 11588 Blue Lower", group = "11588")
public class RI3W11588BlueLower extends RI3W11588BaseAutonomous{
    ElapsedTime runTime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this.hardwareMap);
        waitForStart();
        runTime.reset();
        while(runTime.milliseconds() < 8000){

        }
        park();
    }
}
