package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous.QualBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.Qual11588Hardware;

@Autonomous (name = "NotSoSmolBot Blue Lower Cycle", group = "Not So Smol Bot")
public class NotSoSmolBotBlueLowerCycle extends Qual11588BaseAuto{
    @Override
    public void runOpMode() throws InterruptedException{
        robot.autoInit(hardwareMap, telemetry, Qual11588Hardware.allianceColor.BLUE);
        telemetryStuff();
        waitForStart();
        //identify cone
        moveArm(Height.MEDIUM);
        moveXY(48, 0);
        turnToAngle(270);
        robot.claw.setPosition(robot.OPEN_POSITION);
        moveArm(Height.STACK5);

        robot.claw.setPosition(robot.CLOSED_POSITION);
        moveArm(Height.HIGH);
        turnToAngle(45);
        robot.claw.setPosition(robot.OPEN_POSITION);
        turnToAngle(0);
        moveArm(Height.LOW);
    }
}
