package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Angle turn test ", group = "11588")
public class angleTurnTestAuto extends RI3W11588BaseAutonomous{
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this.hardwareMap, telemetry);
        moveArm(Height.GROUND);
        waitForStart();
        moveAngle(-90);

    }
}
