package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous.QualBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "NotSoSmolBot Auto Tester", group = "11588")
public class NotSoSmolBotAutoTester extends NotSoSmolBotBaseAuto {
    @Override
    public void runOpMode(){
        robot.init(hardwareMap, true);
        waitForStart();
        //scan for cone
        moveArm(Height.MEDIUM);
        moveXYPID(0, -36);
        moveXYAndArm(24, 0, Height.HIGH);
        robot.claw.setPosition(robot.OPEN_POSITION);
        moveXYPID(0, 12);
        //if spot 1
        moveXYAndArm(0, 12, Height.MEDIUM);

        //if spot 2
        //moveXYAndArm(0, 36, Height.MEDIUM);

        //if spot 3
        //moveXYAndArm(0, 60, Height.MEDIUM);

        moveArm(Height.LOW);
        moveArm(Height.GROUND);
    }
}
