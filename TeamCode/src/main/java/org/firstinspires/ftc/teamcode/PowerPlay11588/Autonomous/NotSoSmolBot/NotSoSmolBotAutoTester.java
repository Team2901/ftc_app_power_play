package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous.NotSoSmolBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous (name = "NotSoSmolBot Auto Tester", group = "11588")
public class NotSoSmolBotAutoTester extends NotSoSmolBotBaseAuto{
    @Override
    public void runOpMode(){
        robot.init(hardwareMap, true);
        waitForStart();
        /*
        moveXY(12, 0);
        moveXY(0,12);
        moveXY(-12,0);
        moveXY(0,-12);
         */
        //turnByAngle(90);
        moveXY(12, 0);
        turnByAngle(90);
        moveXY(12, 0);
        turnByAngle(90);
        moveXY(12, 0);
        turnByAngle(90);
        moveXY(12, 0);
        turnByAngle(90);
    }
}
