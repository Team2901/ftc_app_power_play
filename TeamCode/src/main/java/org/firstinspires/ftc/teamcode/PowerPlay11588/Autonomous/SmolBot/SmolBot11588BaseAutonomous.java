package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous.SmolBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.SmolBot11588Hardware;

public class SmolBot11588BaseAutonomous extends LinearOpMode {
    SmolBot11588Hardware robot = new SmolBot11588Hardware();

    //Movement Setup
    ElapsedTime turnPIDTimer = new ElapsedTime();
    double turnStart = 0.0;
    double turnTarget = 0.0;
    double turnError = 0.0;
    double turnTotal = 0.0;
    double pTurn = 0.0;
    double iTurn = 0.0;
    double dTurn = 0.0;
    double kpTurn = 0.0;
    double kiTurn = 0.0;
    double kdTurn = 0.0;

    //Everything to do with the lift pid
    public enum LiftHeight{
        GROUND,
        LOW,
        MID,
        HIGH,
        STACK
    }
    ElapsedTime liftPIDTimer = new ElapsedTime();
    int liftTarget = 50;
    int lastLiftTarget = liftTarget;
    int liftError = 0;
    double liftTotal = 0.0;
    double pLift = 0.0;
    double iLift = 0.0;
    double dLift = 0.0;
    double fLift = 0.0;
    double kpLift = 0.5;
    double kiLift = 0.0;
    double kdLift = 0.0;
    double kfLift = 0.0;
    double iLiftMax = 0.25;

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void moveXY(double y, double x){
        int ticksY = (int)(y * robot.TICKS_PER_INCH);
        int ticksX = (int)(x * robot.TICKS_PER_INCH);

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

        robot.frontLeft.setPower(.5);
        robot.frontRight.setPower(.5);
        robot.backLeft.setPower(.5);
        robot.backRight.setPower(.5);

        while(opModeIsActive() && (robot.frontLeft.isBusy() || robot.frontRight.isBusy() ||
                robot.backLeft.isBusy() || robot.backRight.isBusy())){
            autoTelemetry();
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

    public void moveLift(LiftHeight height){

    }

    public void turnByAngle(double turnAngle){
        turnPIDTimer.reset();
        turnStart = robot.getAngle();
        turnTarget = AngleUnit.normalizeDegrees(turnStart + turnAngle);
        turnError = turnTarget - robot.getAngle();

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(opModeIsActive() && !(turnError < 2 && turnError > -2)){
            turnError = turnTarget - robot.getAngle();
            dTurn = (turnError - pTurn)/turnPIDTimer.seconds();
            iTurn = iTurn + (turnError * turnPIDTimer.seconds());
            pTurn = turnError;

            turnTotal = ((kpTurn * pTurn) + (kiTurn * iTurn) + (kdTurn * dTurn))/100;
            if(turnTotal > .75){
                turnTotal = .75;
            }
            if(turnTotal < -.75){
                turnTotal = -.75;
            }

            robot.frontLeft.setPower(turnTotal);
            robot.frontRight.setPower(-turnTotal);
            robot.backLeft.setPower(turnTotal);
            robot.backRight.setPower(-turnTotal);

            autoTelemetry();
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

    public void autoTelemetry(){
        telemetry.addData("Front Left Target", robot.frontLeft.getTargetPosition());
        telemetry.addData("Front Left Position", robot.frontLeft.getCurrentPosition());
        telemetry.addData("Front Right Target", robot.frontRight.getTargetPosition());
        telemetry.addData("Front Right Position", robot.frontRight.getCurrentPosition());
        telemetry.addData("Back Left Target", robot.backLeft.getTargetPosition());
        telemetry.addData("Back Left Position", robot.backLeft.getCurrentPosition());
        telemetry.addData("Back Right Target", robot.backRight.getTargetPosition());
        telemetry.addData("Back Right Position", robot.backRight.getCurrentPosition());
        telemetry.addData("Robot Angle", robot.getAngle());
        telemetry.addData("Turn Target", turnTarget);
        telemetry.addData("Turn Error", turnError);
        telemetry.addData("Turning Power", turnTotal);
        telemetry.addData("Lift Position", robot.lift.getCurrentPosition());
        telemetry.addData("Lift Target", liftTarget);
        telemetry.addData("Lift Error", liftError);
        telemetry.addData("Lift Power", liftTotal);
        telemetry.update();
    }
}