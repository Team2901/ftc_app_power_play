package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous.QualBot.NotSoSmolBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous.QualBot.Qual11588BaseAuto;

@Autonomous (name = "NotSoSmolBot Auto Tester", group = "11588")
public class NotSoSmolBotAutoTester extends NotSoSmolBotBaseAuto{
    @Override
    public void runOpMode(){
        robot.init(hardwareMap, true);
        waitForStart();
        //scan for cone
        moveArm(Qual11588BaseAuto.Height.MEDIUM);
        moveXYPID(0, -36);
        moveXYAndArm(24, 0, Qual11588BaseAuto.Height.HIGH);
        robot.claw.setPosition(robot.OPEN_POSITION);
        moveXYPID(0, 12);
        moveArm(Qual11588BaseAuto.Height.LOW);
        moveArm(Qual11588BaseAuto.Height.GROUND);
    }
}
