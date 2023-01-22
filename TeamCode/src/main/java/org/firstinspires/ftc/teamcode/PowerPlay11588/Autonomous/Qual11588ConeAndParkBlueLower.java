package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.Qual11588Hardware;
@Autonomous(name = "Qual 11588 Blue Lower Cone and Park", group = "11588")
public class Qual11588ConeAndParkBlueLower extends Qual11588BaseAuto{
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.autoInit(this.hardwareMap, telemetry, Qual11588Hardware.allianceColor.BLUE);
        waitForStart();
        timer.reset();
        timer.reset();
        moveArm(Qual11588BaseAuto.Height.MEDIUM);
        while(robot.pipeLine.framesProceeded < 30){

        }
        coneAndPark();
    }
}
