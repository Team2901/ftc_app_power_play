package org.firstinspires.ftc.teamcode.PowerPlay2901.Autonomous.TankDriveSwerve;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PowerPlay2901.Hardware.RockBotHardware;

public class TankDriveSwerveBaseAuto extends LinearOpMode {
    RockBotHardware robot = new RockBotHardware();
    private ElapsedTime runtime = new ElapsedTime();

    double leftPodPower = 0;
    double rightPodPower = 0;
    double leftTurnPower = 0;
    double rightTurnPower = 0;

    @Override
    public void runOpMode() throws InterruptedException {}

    public void moveInches(double inches){
        int target = (int)(inches * 8192 / 8.9);
        int distance;
        int distanceToTarget;
        double startDiff = robot.encoderLeft.getCurrentPosition() - robot.encoderRight.getCurrentPosition();

        while(robot.encoderLeft.getCurrentPosition() < target-100 && robot.encoderRight.getCurrentPosition() < target-100 && robot.encoderLeft.getCurrentPosition() > target+100 && robot.encoderRight.getCurrentPosition() > target+100) {
            distance = Math.abs(robot.encoderLeft.getCurrentPosition());
            distanceToTarget = Math.abs(target) - distance;

            double forwardPower;

            if(distance > distanceToTarget) {
                forwardPower = (double)distance/10000 + .1;
            } else {
                forwardPower = (double)distanceToTarget/10000 + .1;
            }

            if(target < 0){
                forwardPower *= -1;
            }

            if(forwardPower > 1){
                forwardPower = 1;
            } else if(forwardPower < -1){
                forwardPower = -1;
            }

            double turnPower = (robot.encoderLeft.getCurrentPosition() - robot.encoderRight.getCurrentPosition()) - (startDiff) * (forwardPower+.3) / 10000;

            leftPodPower = forwardPower - turnPower;
            rightPodPower = forwardPower + turnPower;

            leftTurnPower = leftPodTurn(0);
            rightTurnPower = rightPodTurn(0);

            robot.leftOne.setVelocity((leftPodPower / 2 + leftTurnPower) * 2500);
            robot.leftTwo.setVelocity((leftPodPower / 2 - leftTurnPower) * 2500);
            robot.rightOne.setVelocity((rightPodPower / 2 + rightTurnPower) * 2500);
            robot.rightTwo.setVelocity((rightPodPower / 2 - rightTurnPower) * 2500);
        }
    }

    public void turnByAngle(double turnAngle){
        double startAngle = robot.getAngle();
        double targetAngle = AngleUnit.normalizeDegrees(startAngle + turnAngle);
        ElapsedTime runtime = new ElapsedTime();
        double p = 0;
        double i = 0;
        double d = 0;
        double kp = -5.6;
        double ki = -.5;
        double kd = -0.4;
        double error = turnAngle;

        robot.leftOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(opModeIsActive() && !(error < 1.5 && error > -1.5)){
            error = (targetAngle - robot.getAngle());
            double secs = runtime.seconds();
            runtime.reset();
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Current Angle", robot.getAngle());
            telemetry.addData("Loop Time", secs);
            d = (error - p) / secs;
            i = i + (error * secs);
            p = error;
            double total = (kp* p + ki* i + kd* d)/100;
            if(total > 1){
                i = 0;
                total = 1;
            }
            if(total < -1){
                i = 0;
                total = -1;
            }

            telemetry.addData("turnPower", total);

            leftPodPower = total;
            rightPodPower = -total;

            leftTurnPower = leftPodTurn(0);
            rightTurnPower = rightPodTurn(0);

            robot.leftOne.setPower((leftPodPower / 4 + leftTurnPower) * 2500);
            robot.leftTwo.setPower((leftPodPower / 4 - leftTurnPower) * 2500);
            robot.rightOne.setPower((rightPodPower / 4 + rightTurnPower) * 2500);
            robot.rightTwo.setPower((rightPodPower / 4 - rightTurnPower) * 2500);

            telemetry.update();
        }

        robot.leftOne.setPower(0);
        robot.leftTwo.setPower(0);
        robot.rightOne.setPower(0);
        robot.rightTwo.setPower(0);
    }

    public void getOffWall(){
        //this dead reckons lmao
        runtime.reset();

        while(runtime.milliseconds() < 50){
            leftTurnPower = leftPodTurn(90);
            rightTurnPower = rightPodTurn(90);

            robot.leftOne.setPower((0 + leftTurnPower) * 2500);
            robot.leftTwo.setPower((0 - leftTurnPower) * 2500);
            robot.rightOne.setPower((0 + rightTurnPower) * 2500);
            robot.rightTwo.setPower((0 - rightTurnPower) * 2500);
        }

        while(runtime.milliseconds() < 500){
            leftPodPower = .2;
            rightPodPower = .2;

            leftTurnPower = leftPodTurn(90);
            rightTurnPower = rightPodTurn(90);

            robot.leftOne.setPower((leftPodPower + leftTurnPower) * 2500);
            robot.leftTwo.setPower((leftPodPower - leftTurnPower) * 2500);
            robot.rightOne.setPower((rightPodPower + rightTurnPower) * 2500);
            robot.rightTwo.setPower((rightPodPower - rightTurnPower) * 2500);
        }

        while(runtime.milliseconds() < 550) {
            leftTurnPower = leftPodTurn(0);
            rightTurnPower = rightPodTurn(0);

            robot.leftOne.setPower((0 + leftTurnPower) * 2500);
            robot.leftTwo.setPower((0 - leftTurnPower) * 2500);
            robot.rightOne.setPower((0 + rightTurnPower) * 2500);
            robot.rightTwo.setPower((0 - rightTurnPower) * 2500);
        }

        robot.leftOne.setPower(0);
        robot.leftTwo.setPower(0);
        robot.rightOne.setPower(0);
        robot.rightTwo.setPower(0);

        this.turnByAngle(90);

        runtime.reset();

        while(runtime.milliseconds() < 500){
            leftPodPower = -.2;
            rightPodPower = -.2;
            leftTurnPower = leftPodTurn(0);
            rightTurnPower = rightPodTurn(0);

            robot.leftOne.setPower((leftPodPower + leftTurnPower) * 2500);
            robot.leftTwo.setPower((leftPodPower - leftTurnPower) * 2500);
            robot.rightOne.setPower((rightPodPower + rightTurnPower) * 2500);
            robot.rightTwo.setPower((rightPodPower - rightTurnPower) * 2500);
        }

        robot.leftOne.setPower(0);
        robot.leftTwo.setPower(0);
        robot.rightOne.setPower(0);
        robot.rightTwo.setPower(0);
    }

    double lp = 0;
    double li = 0;
    double ld = 0;

    double liftP = 0;
    double liftI = 0;
    double liftD = 0;

    public double liftTarget = 0;

    public void liftPower() {
        double total = 0;
        robot.liftOne.setPower(total + .05);
        robot.liftTwo.setPower(total + .05);
    }

    double kp = 1.2;
    double ki = 0;
    double kd = 0;

    private ElapsedTime runtimePodLeft = new ElapsedTime();
    double leftPodAngle = 0;
    double pAngleLeft = 0;
    double iAngleLeft = 0;
    double dAngleLeft = 0;

    public double leftPodTurn(double angle){
        leftPodAngle = (robot.leftOne.getCurrentPosition() - robot.leftTwo.getCurrentPosition())/8.95;
        double error = AngleUnit.normalizeDegrees(angle - leftPodAngle);
        if(!gamepad1.start && (error >= 90 || error <= -90)){
            error = AngleUnit.normalizeDegrees(error-180);
            leftPodPower = -leftPodPower;
        }
        double secs = runtimePodLeft.seconds();
        runtime.reset();
        dAngleLeft = (error - pAngleLeft) / secs;
        iAngleLeft = iAngleLeft + (error * secs);
        pAngleLeft = error;
        double total = (kp* pAngleLeft + ki* iAngleLeft + kd* dAngleLeft)/100;
        if(total > 1){
            iAngleLeft = 0;
            total = 1;
        }
        if(total < -1){
            iAngleLeft = 0;
            total = -1;
        }
        return total;
    }

    private ElapsedTime runtimePodRight = new ElapsedTime();
    double rightPodAngle = 0;
    double pAngleRight = 0;
    double iAngleRight = 0;
    double dAngleRight = 0;

    public double rightPodTurn(double angle){
        rightPodAngle = (robot.rightOne.getCurrentPosition() - robot.rightTwo.getCurrentPosition())/8.95;
        double error = AngleUnit.normalizeDegrees(angle - rightPodAngle);
        if(!gamepad1.start && (error >= 90 || error <= -90)){
            error = AngleUnit.normalizeDegrees(error-180);
            rightPodPower = -rightPodPower;
        }
        double secs = runtimePodRight.seconds();
        runtime.reset();
        dAngleRight = (error - pAngleRight) / secs;
        iAngleRight = iAngleRight + (error * secs);
        pAngleRight = error;
        double total = (kp* pAngleRight + ki* iAngleRight + kd* dAngleRight)/100;
        if(total > 1){
            iAngleRight = 0;
            total = 1;
        }
        if(total < -1){
            iAngleRight = 0;
            total = -1;
        }
        return total;
    }
}
