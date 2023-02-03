package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous.QualBot.NotSoSmolBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous.QualBot.Qual11588BaseAuto;
import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.NotSoSmolBotHardware;

public class NotSoSmolBotBaseAuto extends LinearOpMode {
    NotSoSmolBotHardware robot = new NotSoSmolBotHardware();

    double startAngle = 0;
    //Has targetAngle return -1 if it has not been defined, it is redefined before it is used
    double targetAngle = -1;
    double turnError = 0;
    double turnPower = .5;
    double currentAngle = 0;

    ElapsedTime PIDTimer = new ElapsedTime();
    //Defining the pidvariables outside the method so they can be used in a telemetry method
    int armTarget = 50;
    int lastArmTarget = armTarget;
    int error = 0;
    double total = 0.0;
    double kp = 0.9;
    double ki = 0.0;
    double kd = 0.0;
    double kCos = 0.3;
    double pArm = 0.0;
    double iArm = 0.0;
    double dArm = 0.0;
    double cosArm = 0.0;
    double iArmMax = .25;
    double armAngle = 0;

    public enum Height{
        GROUND,
        LOW,
        MEDIUM,
        HIGH
    }
    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void turnByAngle(double turnAngle){
        startAngle = robot.getAngle();
        targetAngle = AngleUnit.normalizeDegrees(startAngle + turnAngle);
        turnError = targetAngle - robot.getAngle();
        while (opModeIsActive() && !(turnError < 2 && turnError > -2)){
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
        //robot.getAngle is between -180 and 180, starting at 0
        targetAngle = AngleUnit.normalizeDegrees(turnAngle) + 180;
        startAngle = robot.getAngle() + 180;
        turnError = AngleUnit.normalizeDegrees(targetAngle - startAngle);
        while(opModeIsActive() && !(turnError < 2 && turnError > -2)){
            if(turnError >= 0){
                turnPower = turnError/50;
                if(turnPower > .5){
                    turnPower = .5;
                }
            }else if(turnError < 0){
                turnPower = turnError/50;
                if(turnPower < -.5){
                    turnPower = -.5;
                }
            }
            robot.frontLeft.setPower(-turnPower);
            robot.frontRight.setPower(turnPower);
            robot.backLeft.setPower(-turnPower);
            robot.backRight.setPower(turnPower);

            currentAngle = robot.getAngle() + 180;
            turnError = AngleUnit.normalizeDegrees(targetAngle - currentAngle);

            telemetry.addData("Robot Angle", robot.getAngle());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Turn Error", turnError);
        }
    }
    public void moveArm(Qual11588BaseAuto.Height height) {
        if (height == Qual11588BaseAuto.Height.GROUND) {
            armTarget = 200;
        } else if (height == Qual11588BaseAuto.Height.LOW) {
            armTarget = 550;
        } else if (height == Qual11588BaseAuto.Height.MEDIUM) {
            armTarget = 800;
        } else if (height == Qual11588BaseAuto.Height.HIGH) {
            armTarget = 1150;
        }
        error = armTarget - robot.arm.getCurrentPosition();
        PIDTimer.reset();

        while (opModeIsActive() && !(error < 10 && error > -10)) {
            error = armTarget - robot.arm.getCurrentPosition();
            dArm = (error - pArm) / PIDTimer.seconds();
            iArm = iArm + (error * PIDTimer.seconds());
            pArm = error;
            armAngle = (90.0 / (1200.0 - 400.0)) * (robot.arm.getCurrentPosition() - 400.0);
            //armAngle = 0.102856 * robot.arm.getCurrentPosition() - 43.6276;
            cosArm = Math.cos(Math.toRadians(armAngle));
            total = ((kp * pArm) + (ki * iArm) + (kd * dArm)) / 100 + (kCos * cosArm);
            robot.arm.setPower(total);
            PIDTimer.reset();
            if (iArm > iArmMax) {
                iArm = iArmMax;
            } else if (iArm < -iArmMax) {
                iArm = -iArmMax;
            }
            if (total > .5) {
                total = .5;
            }
            if (armAngle > 60 && total < -.5) {
                total = -.5;
            } else if (total < .025 && armAngle < 60) {
                total = .025;
            }
        }
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

    public void moveXYPID(double y, double x){
        int ticksY = (int) (y * robot.TICKS_PER_INCH);
        int ticksX = (int) (x * robot.TICKS_PER_INCH);



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
            armAngle = (90.0/(1200 - 400)) * (robot.arm.getCurrentPosition() - 400);
            //armAngle = 0.102856 * robot.arm.getCurrentPosition() - 43.6276;

            error = armTarget - robot.arm.getCurrentPosition();
            dArm = (error - pArm) / PIDTimer.seconds();
            iArm = iArm + (error * PIDTimer.seconds());
            pArm = error;
            cosArm = Math.cos(Math.toRadians(armAngle));
            total = ((pArm * kp) + (iArm * ki) + (dArm * kd))/100 + (cosArm * kCos);
            PIDTimer.reset();

            if(armTarget != lastArmTarget){
                iArm = 0;
            }

            if(iArm > iArmMax){
                iArm = iArmMax;
            }else if(iArm < -iArmMax){
                iArm = -iArmMax;
            }

            if(total > .65){
                total = .65;
            }
            if(armAngle > 60 && total < -.5){
                total = -.5;
            }else if(total < .005 && armAngle < 60){
                total = .005;
            }
            lastArmTarget = armTarget;

            cosArm = Math.cos(Math.toRadians(armAngle));
            double ffTotal = cosArm * kCos;
            robot.arm.setPower(ffTotal);
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

    public void moveXYAndArm(double y, double x, Qual11588BaseAuto.Height height){
        if(height == Qual11588BaseAuto.Height.GROUND){
            armTarget = 200;
        } else if(height == Qual11588BaseAuto.Height.LOW){
            armTarget = 550;
        } else if(height == Qual11588BaseAuto.Height.MEDIUM){
            armTarget = 800;
        } else if(height == Qual11588BaseAuto.Height.HIGH){
            armTarget = 1150;
        }
        error = armTarget - robot.arm.getCurrentPosition();
        PIDTimer.reset();

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

        while (opModeIsActive() && ((robot.frontLeft.isBusy() || robot.frontRight.isBusy() ||
                robot.backLeft.isBusy() || robot.backRight.isBusy()) || !(error < 10 && error > -10))){
            error = armTarget - robot.arm.getCurrentPosition();
            dArm = (error - pArm)/PIDTimer.seconds();
            iArm = iArm + (error * PIDTimer.seconds());
            pArm = error;
            armAngle = (90.0/(1200.0 - 400.0)) * (robot.arm.getCurrentPosition() - 400.0);
            //armAngle = 0.102856 * robot.arm.getCurrentPosition() - 43.6276;
            cosArm = Math.cos(Math.toRadians(armAngle));
            total = ((kp*pArm) + (ki*iArm) + (kd*dArm))/100 + (kCos *cosArm);
            robot.arm.setPower(total);
            PIDTimer.reset();
            if(iArm > iArmMax){
                iArm = iArmMax;
            } else if(iArm < -iArmMax){
                iArm = -iArmMax;
            }
            if(total > .5){
                total = .5;
            }
            if(armAngle > 60 && total < -.5){
                total = -.5;
            }else if(total < .025 && armAngle < 60){
                total = .025;
            }
        }

        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armAngle = (90.0/(1200.0 - 400.0)) * (robot.arm.getCurrentPosition() - 400.0);
        //armAngle = 0.102856 * robot.arm.getCurrentPosition() - 43.6276;
        cosArm = Math.cos(Math.toRadians(armAngle));
        double ffTotal = cosArm * kCos;
        robot.arm.setPower(ffTotal);
    }
}
