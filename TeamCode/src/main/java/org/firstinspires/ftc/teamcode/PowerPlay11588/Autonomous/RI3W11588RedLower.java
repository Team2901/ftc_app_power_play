package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.OpenCVPipelines.RI3W11588OpenCV;

@Autonomous(name = "RI3W 11588 Red Lower", group = "11588")
public class RI3W11588RedLower extends RI3W11588BaseAutonomous{
    ElapsedTime runTime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this.hardwareMap, telemetry);

        waitForStart();
        runTime.reset();
        while(runTime.milliseconds() < 2000){}
        if (robot.pipeLine.redAmount > robot.pipeLine.blueAmount && robot.pipeLine.redAmount > robot.pipeLine.greenAmount) {
            robot.pipeLine.coneColor = RI3W11588OpenCV.ConeColor.red;
        } else if (robot.pipeLine.blueAmount > robot.pipeLine.redAmount && robot.pipeLine.blueAmount > robot.pipeLine.greenAmount) {
            robot.pipeLine.coneColor = RI3W11588OpenCV.ConeColor.blue;
        } else if (robot.pipeLine.greenAmount > robot.pipeLine.redAmount && robot.pipeLine.greenAmount > robot.pipeLine.blueAmount) {
            robot.pipeLine.coneColor = RI3W11588OpenCV.ConeColor.green;
        }
        robot.camera.stopStreaming();
        moveArm(Height.GROUND);
        park();
    }
}
