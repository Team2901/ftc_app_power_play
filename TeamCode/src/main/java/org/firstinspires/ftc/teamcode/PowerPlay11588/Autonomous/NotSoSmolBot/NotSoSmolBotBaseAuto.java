package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous.NotSoSmolBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.NotSoSmolBotHardware;

public class NotSoSmolBotBaseAuto extends LinearOpMode {
    NotSoSmolBotHardware robot = new NotSoSmolBotHardware();

    double startAngle = 0;
    //Has targetAngle return -1 if it has not been defined, it is redefined before it is used
    double targetAngle = -1;
    double turnError = 0;
    double turnPower = .5;
    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void moveXY(double y, double x){
        int ticksY = (int) (y * robot.TICKS_PER_INCH);
        int ticksX = (int) (1.5 * x * robot.TICKS_PER_INCH);

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeft.setTargetPosition(ticksY + ticksX);
        robot.frontRight.setTargetPosition(ticksY - ticksX);
        robot.backLeft.setTargetPosition(ticksY - ticksX);
        robot.backRight.setTargetPosition(ticksY + ticksX);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeft.setPower(0.5);
        robot.frontRight.setPower(0.5);
        robot.backLeft.setPower(0.5);
        robot.backRight.setPower(0.5);
        while (opModeIsActive() && (robot.frontLeft.isBusy() || robot.frontRight.isBusy() ||
                robot.backLeft.isBusy() || robot.backRight.isBusy())){
            telemetry.addData("Front Left Target", robot.frontLeft.getTargetPosition());
            telemetry.addData("Front Left Position", robot.frontLeft.getCurrentPosition());
            telemetry.addData("Front Right Target", robot.frontRight.getTargetPosition());
            telemetry.addData("Front Right Position", robot.frontRight.getCurrentPosition());
            telemetry.addData("Back Left Target", robot.backLeft.getTargetPosition());
            telemetry.addData("Back Left Position", robot.backLeft.getCurrentPosition());
            telemetry.addData("Back Right Target", robot.backRight.getTargetPosition());
            telemetry.addData("Back Right Position", robot.backRight.getCurrentPosition());
            telemetry.addData("Front Left Power", robot.frontLeft.getPower());
            telemetry.addData("Front Right Power", robot.frontRight.getPower());
            telemetry.addData("Back Left Power", robot.backLeft.getPower());
            telemetry.addData("Back Right Power", robot.backRight.getPower());
            telemetry.addData("Robot Angle", robot.getAngle());
            telemetry.update();
        }
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnByAngle(double turnAngle){
        startAngle = robot.getAngle();
        targetAngle = AngleUnit.normalizeDegrees(startAngle + turnAngle);
        turnError = targetAngle - robot.getAngle();
        while (opModeIsActive() && !(turnError < 5 && turnError > -5)){
            turnError = targetAngle - robot.getAngle();
            if(turnError < 0){
                turnPower = -.5 * (turnError/50);
                if(turnPower < -.5){
                    turnPower = -.5;
                }
            }else if(turnError > 0){
                turnPower = .5 * (turnError/50);
                if(turnPower > .5){
                    turnPower = .5;
                }
            }
            robot.frontLeft.setPower(-turnPower);
            robot.frontRight.setPower(turnPower);
            robot.backLeft.setPower(-turnPower);
            robot.backRight.setPower(turnPower);
            turnError = targetAngle - robot.getAngle();

            telemetry.addData("Robot Angle", robot.getAngle());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Turn Error", turnError);
        }
    }
    public void turnToAngle(double turnAngle){
        targetAngle = turnAngle + 180;
        startAngle = robot.getAngle() + 180;


    }
}
