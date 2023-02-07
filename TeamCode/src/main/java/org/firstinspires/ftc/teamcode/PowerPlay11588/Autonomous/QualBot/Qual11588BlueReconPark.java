package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous.QualBot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.Qual11588Hardware;

@Autonomous(name = "Qual 11588 Blue Recon-Park ", group = "11588")
public class Qual11588BlueReconPark extends Qual11588BaseAuto {
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.autoInit(this.hardwareMap, telemetry, Qual11588Hardware.allianceColor.BLUE);
        waitForStart();
        robot.pipeLine.stopCam();
        moveArm(Qual11588BaseAuto.Height.GROUND);
        reconParkAuto();
    }
}
