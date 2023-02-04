package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous.QualBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.Qual11588Hardware;


@Autonomous (name = "NotSoSmolBot Blue Upper", group = "Not So Smol Bot")
public class NotSoSmolBotBlueUpper extends Qual11588BaseAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry, false, Qual11588Hardware.allianceColor.BLUE, true);
        telemetryStuff();
        waitForStart();
        //scan for cone
        moveArm(Height.MEDIUM);
        moveXYPID(6, 0);
        moveXYPID(0, 24);
        moveXYPID(24,0);
        moveArm(Height.HIGH);
        safeWaitPID(500);
        moveXYPID(0,16);
        moveXYPID(3,0);
        robot.claw.setPosition(robot.OPEN_POSITION);
        safeWaitPID(500);
        moveXYPID(-6,0);

        //if spot 1
        //moveXYPID(0,-15);

        //if spot 2
        //moveXYPID(0, -40);

        //if spot 3
        moveXYPID(0, -66);

        moveArm(Height.GROUND);
    }
}
