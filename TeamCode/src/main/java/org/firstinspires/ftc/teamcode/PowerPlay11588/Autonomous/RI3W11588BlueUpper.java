package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.OpenCVPipelines.Qual11588OpenCV;

@Autonomous(name = "Blue upper", group = "11588")
public class RI3W11588BlueUpper extends RI3W11588BaseAutonomous{
    ElapsedTime runTime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this.hardwareMap, telemetry);

        waitForStart();
        runTime.reset();
        while(runTime.milliseconds() < 1000){}
        if (robot.pipeLine.redAmount > robot.pipeLine.blueAmount - 3 && robot.pipeLine.redAmount > robot.pipeLine.greenAmount) {
            robot.pipeLine.coneColor = Qual11588OpenCV.ConeColor.RED;
        } else if (robot.pipeLine.blueAmount - 3 > robot.pipeLine.redAmount && robot.pipeLine.blueAmount - 3 > robot.pipeLine.greenAmount) {
            robot.pipeLine.coneColor = Qual11588OpenCV.ConeColor.BLUE;
        } else if (robot.pipeLine.greenAmount > robot.pipeLine.redAmount && robot.pipeLine.greenAmount > robot.pipeLine.blueAmount - 3) {
            robot.pipeLine.coneColor = Qual11588OpenCV.ConeColor.GREEN;
        }
        /*Scuffed Solution but the reason we subtract 3 from all the blue values is because the camera is
         *more sensitive to the blue cone.
         */

        robot.camera.stopStreaming();
        moveArm(Height.GROUND);
        park();
    }
}
