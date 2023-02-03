package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous.QualBot;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.Qual11588Hardware;

public class Qual211588BlueLowerCone extends Qual11588BaseAuto{
    @Override
    public void runOpMode() throws InterruptedException {
        robot.autoInit(hardwareMap, telemetry, Qual11588Hardware.allianceColor.BLUE);
        //Anything to happen on init before start

        waitForStart();
        //raise arm
        //moveArm(Height.MEDIUM);

        //move left 36in
        //moveXY(0, 36);

        //robot.claw.setPosition(CLAW_OPEN_POSITION)(actual name in hardware)
    }
}
