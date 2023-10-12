package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous.QualBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.OpenCVPipelines.Qual11588OpenCV;
import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.Qual11588Hardware;

@Autonomous (name = "NotSoSmolBot Red Lower Cycle", group = "Not So Smol Bot")
public class NotSoSmolBotRedLowerCycle extends Qual11588BaseAuto{
    @Override
    public void runOpMode() throws InterruptedException{
        robot.autoInit(hardwareMap, telemetry, Qual11588Hardware.allianceColor.RED);
        telemetryLog();
        waitForStart();
        robot.pipeLine.stopCam();

        Qual11588OpenCV.ConeColor color = robot.pipeLine.getColor();
        moveArm(Height.MEDIUM);
        moveXYPID(54, 0, 0.8);
        moveXYPID(-4, 0, 0.7);
        moveArm(Height.HIGH);
        turnToAngle(0);
        moveXYPID(0, 14, 0.8);
        turnToAngle(0);
        moveXYPID(5, 0, 0.7);
        turnToAngle(0);
        safeWaitPID(100);
        robot.claw.setPosition(robot.OPEN_POSITION);
        safeWaitPID(100);
        moveXYPID(-6, 0, 0.7);
        moveXYPID(0, -12, 0.7);
        turnToAngle(270);
        turnToAngle(270);
        moveXYPID(10, 0, 0.8);
        turnToAngle(270);
        moveArm(Height.STACK4);

        moveXYPID(3, 0);
        turnToAngle(270);
        robot.claw.setPosition(robot.CLOSED_POSITION);
        safeWaitPID(250);
        moveXYPID(-2, 0, .5);
        moveArm(Height.HIGH);
        moveXYPID(-18, 0, .8);
        turnToAngle(0);
        moveXYPID(6, 0);
        turnToAngle(0);
        safeWaitPID(100);
        robot.claw.setPosition(robot.OPEN_POSITION);
        safeWaitPID(100);
        moveXYPID(-6, 0);

        if (color == Qual11588OpenCV.ConeColor.RED) {
            moveXYPID(0, 12);
            moveXYPID(-24, 0, .8);
        } else if (color == Qual11588OpenCV.ConeColor.GREEN) {
            moveXYPID(0, -12);
            moveXYPID(-24, 0, .8);
        } else if (color == Qual11588OpenCV.ConeColor.BLUE) {
            moveXYPID(0, 36, .8);
            moveXYPID(-24, 0, .8);
        }
        //if spot 1
//        moveXYPID(0, 12);
//        moveXYPID(-24, 0, .8);

        //if spot 2
        /*
        moveXYPID(0, -12);
        moveXYPID(-24, 0, .8);

         */

        //if spot 3
        /*
        moveXYPID(0, 36, .8);
        moveXYPID(-24, 0, .8);

         */
        moveArm(Height.GROUND);


    }
}
