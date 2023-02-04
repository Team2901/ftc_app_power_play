package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous.QualBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous.QualBot.Qual11588BaseAuto;
import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.Qual11588Hardware;

@Autonomous (name = "NotSoSmolBot Blue Lower", group = "11588")
public class NotSoSmolBotBlueLower extends Qual11588BaseAuto {
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry, false, Qual11588Hardware.allianceColor.BLUE, true);
        telemetryStuff();
        waitForStart();
        //scan for cone
        moveArm(Height.MEDIUM);
        moveXY(6, 0);
        moveXY(0, -24);
        //moveXYAndArm(24, 0, Height.HIGH);
        robot.claw.setPosition(robot.OPEN_POSITION);
        timer.reset();
        while(opModeIsActive() && timer.milliseconds() < 1000){

        }
        robot.claw.setPosition(robot.CLOSED_POSITION);
        timer.reset();
        while(opModeIsActive() && timer.milliseconds() < 1000){

        }
        moveXY(24,0);
        moveArm(Height.HIGH);
        timer.reset();
        while(opModeIsActive() && timer.milliseconds() < 1000){

        }
        moveXY(0, -18);
        moveXY(4, 0);
        robot.claw.setPosition(robot.OPEN_POSITION);
        timer.reset();
        while(opModeIsActive() && timer.milliseconds() < 1000){

        }
        moveXY(-6, 0);

        //if spot 1
        //moveXYAndArm(0, 12, Height.MEDIUM);
        moveXY(0, 15);
        moveArm(Height.MEDIUM);

        //if spot 2
        //moveXYAndArm(0, 36, Height.MEDIUM);

        //if spot 3
        //moveXYAndArm(0, 60, Height.MEDIUM);

        moveArm(Height.GROUND);

    }
}
