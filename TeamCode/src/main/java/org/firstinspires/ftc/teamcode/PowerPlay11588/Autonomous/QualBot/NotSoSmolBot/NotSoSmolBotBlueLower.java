package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous.QualBot.NotSoSmolBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous.QualBot.Qual11588BaseAuto;
import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.Qual11588Hardware;

@Autonomous (name = "NotSoSmolBot Blue Lower", group = "11588")
public class NotSoSmolBotBlueLower extends Qual11588BaseAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry, false, Qual11588Hardware.allianceColor.BLUE, true);
        waitForStart();
        //scan for cone
        moveArm(Height.MEDIUM);
        moveXYPID(0, -36);
        moveXYAndArm(24, 0, Height.HIGH);
        robot.claw.setPosition(robot.OPEN_POSITION);
        moveXYPID(0, 12);
        moveArm(Height.LOW);
        moveArm(Height.GROUND);
    }
}
