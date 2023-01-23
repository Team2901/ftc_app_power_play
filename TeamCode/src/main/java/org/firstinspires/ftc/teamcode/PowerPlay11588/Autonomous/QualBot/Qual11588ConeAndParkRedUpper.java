package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous.QualBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous.QualBot.Qual11588BaseAuto;
import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.Qual11588Hardware;
@Autonomous(name = "Qual 11588 Red Upper Cone and Park", group = "11588")
public class Qual11588ConeAndParkRedUpper extends Qual11588BaseAuto {
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.autoInit(this.hardwareMap, telemetry, Qual11588Hardware.allianceColor.RED);
        waitForStart();
        timer.reset();
        timer.reset();
        moveArm(Qual11588BaseAuto.Height.MEDIUM);
        while(robot.pipeLine.framesProceeded < 30){

        }
        coneAndPark(true);
    }
}
