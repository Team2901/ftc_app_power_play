package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Qual 11588 Blue Recon-Park ", group = "11588")
public class Qual11588BlueReconPark extends Qual11588BaseAuto {
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this.hardwareMap, telemetry);
        waitForStart();
        timer.reset();
        timer.reset();
        while(robot.pipeLine.framesProceeded < 30 && timer.milliseconds() < 5000){

        }
        moveArm(Qual11588BaseAuto.Height.GROUND);
        reconParkAuto();
    }
}
