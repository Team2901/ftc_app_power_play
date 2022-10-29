package org.firstinspires.ftc.teamcode.PowerPlay2901.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PowerPlay2901.EarlyDiffy.EarlyDiffyHardware;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.Utility.CountDownTimer;

@Autonomous(name = "Swerve Park Auto", group = "Iterative OpMode")
public class QuickParkAuto extends LinearOpMode {
    EarlyDiffyHardware robot = new EarlyDiffyHardware();

    CountDownTimer countDownTimer = new CountDownTimer(ElapsedTime.Resolution.MILLISECONDS);
    @Override
    public void runOpMode() throws InterruptedException {
        countDownTimer.setTargetTime(1500);
        while (countDownTimer.hasRemainingTime()) {
            robot.leftOne.setVelocity(0.3 * 2500);
            robot.leftTwo.setVelocity(0.3 * 2500);
            robot.rightOne.setVelocity(0.3 * 2500);
            robot.rightTwo.setVelocity(0.3 * 2500);
        }
        robot.leftOne.setVelocity(0);
        robot.leftTwo.setVelocity(0);
        robot.rightOne.setVelocity(0);
        robot.rightTwo.setVelocity(0);

        countDownTimer.setTargetTime(500);
        while (countDownTimer.hasRemainingTime()) {
            robot.leftOne.setVelocity(-0.3 * 2500);
            robot.leftTwo.setVelocity(-0.3 * 2500);
            robot.rightOne.setVelocity(0.3 * 2500);
            robot.rightTwo.setVelocity(0.3 * 2500);
        }

        robot.liftOne.setTargetPosition(850);
        robot.liftOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftOne.setPower(0.9);
        robot.liftTwo.setTargetPosition(850);
        robot.liftTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftTwo.setPower(0.9);

        robot.clawOne.setPosition(0.00);
//        robot.clawTwo.setPosition(0.25);

        robot.liftOne.setTargetPosition(15);
        robot.liftOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftOne.setPower(-0.9);
        robot.liftTwo.setTargetPosition(15);
        robot.liftTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftTwo.setPower(-0.9);
    }
}
