package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous.QualBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.OpenCVPipelines.Qual11588OpenCV;
import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.Qual11588Hardware;

@Autonomous (name = "NotSoSmolBot Blue Lower", group = "Not So Smol Bot")
public class NotSoSmolBotBlueLower extends Qual11588BaseAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        robot.autoInit(hardwareMap, telemetry, Qual11588Hardware.allianceColor.BLUE);
        telemetryLog();
        waitForStart();
        robot.pipeLine.stopCam();
        Qual11588OpenCV.ConeColor color = robot.pipeLine.getColor();
        moveArm(Height.MEDIUM);
        moveXYPID(6, 0);
        moveXYPID(0, -24);
        moveXYPID(24,0);
        moveArm(Height.HIGH);
        safeWaitPID(500);
        moveXYPID(0, -16);
        moveXYPID(3, 0);
        robot.claw.setPosition(robot.OPEN_POSITION);
        safeWaitPID(500);
        moveXYPID(-6, 0);

        if (color == Qual11588OpenCV.ConeColor.RED) {
            moveXYPID(0, 15);
        } else if (color == Qual11588OpenCV.ConeColor.GREEN) {
            moveXYPID(0, 40);
        } else if (color == Qual11588OpenCV.ConeColor.BLUE) {
            moveXYPID(0, 66);
        }

        //if spot 2
        //

        //if spot 3

        moveArm(Height.GROUND);
    }
}
