package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous.QualBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.Qual11588Hardware;

@Autonomous (name = "NotSoSmolBot Red Lower", group = "11588")
public class NotSoSmolBotRedLower extends Qual11588BaseAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry, false, Qual11588Hardware.allianceColor.RED, true);
        waitForStart();
        //scan for cone
        moveArm(Height.MEDIUM);
        moveXYPID(0, 36);
        moveXYAndArm(24, 0, Height.HIGH);
        robot.claw.setPosition(robot.OPEN_POSITION);

        //if spot 1
        moveXYAndArm(0, -12, Height.MEDIUM);

        //if spot 2
        //moveXYAndArm(0, -36, Height.MEDIUM);

        //if spot 3
        //moveXYAndArm(0, -60, Height.MEDIUM);

        moveArm(Height.LOW);
        moveArm(Height.GROUND);
    }
}
