package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.OpenCVPipelines.RI3W11588OpenCV;
import org.firstinspires.ftc.teamcode.PowerPlay11588.TeleOp.Qual11588TeleOp;

@Autonomous(name = "RI3W 11588 Blue ", group = "11588")
public class Qual11588ConeAndPark extends Qual11588BaseAuto{
    ElapsedTime runTime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        this.teamColor = AllianceColor.BLUE;
        robot.init(this.hardwareMap, telemetry);
        waitForStart();
        runTime.reset();
        moveArm(Height.GROUND);
        while(runTime.milliseconds() < 2000){

        }
        //coneAndPark();
    }
}
