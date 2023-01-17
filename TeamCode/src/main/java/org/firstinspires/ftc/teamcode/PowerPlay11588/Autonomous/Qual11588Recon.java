package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.Qual11588Hardware;
@Autonomous(name = "Qual 11588 Recon", group = "11588")
public class Qual11588Recon extends Qual11588BaseAuto {
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this.hardwareMap, telemetry);
        waitForStart();
        timer.reset();
        while(robot.pipeLine.framesProceeded < 30){

        }
        moveArm(Qual11588BaseAuto.Height.GROUND);
        recon();
    }
}