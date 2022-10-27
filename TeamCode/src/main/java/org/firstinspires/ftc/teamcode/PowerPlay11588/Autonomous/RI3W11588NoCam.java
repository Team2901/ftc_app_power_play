package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "RI3W 11588 No Camera", group = "11588")
public class RI3W11588NoCam extends RI3W11588BaseAutonomous{
    ElapsedTime runTime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this.hardwareMap, telemetry);
        waitForStart();
        runTime.reset();
        moveArm(Height.LOW);
        while(runTime.milliseconds() < 5000){
            telemetry.addData("Arm Power", robot.arm.getPower());
            telemetry.update();
        }
        runTime.reset();
        moveXY(0, -24);
        while(runTime.milliseconds() < 2000){}
        moveXY(36, 0);
    }
}
