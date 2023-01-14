package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.Qual11588Hardware;

@Autonomous(name = "Qual 11588 Blue Recon-Park ", group = "11588")
public class Qual11588BlueReconPark extends Qual11588BaseAuto {
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(this.hardwareMap, telemetry, true, Qual11588Hardware.allianceColor.BLUE);
        waitForStart();
        timer.reset();
        while (timer.milliseconds() < 5000 && opModeIsActive()) {}
        moveArm(Height.GROUND);
        reconParkAuto();
    }
}
