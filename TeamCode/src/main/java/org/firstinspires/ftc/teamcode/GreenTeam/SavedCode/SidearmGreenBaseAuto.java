package org.firstinspires.ftc.teamcode.GreenTeam.SavedCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.GreenTeam.SavedCode.SidearmGreenHardware;

public class SidearmGreenBaseAuto extends LinearOpMode {
    public SidearmGreenHardware robot = new SidearmGreenHardware();
    enum Direction{
        RIGHT,
        LEFT
    }

    enum Side{
        RED,
        BLUE
    }

    enum Height{
        INTAKE,
        LOW,
        MEDIUM,
        HIGH,
        SHELF_HIGH,
        SHELF_MID,
        SHELF
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
    public void moveInches(double inches){
        int ticks = (int)(inches * robot.TICKS_PER_INCH);

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeft.setTargetPosition(ticks);
        robot.frontRight.setTargetPosition(ticks);
        robot.backLeft.setTargetPosition(ticks);
        robot.backRight.setTargetPosition(ticks);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeft.setPower(1);
        robot.backLeft.setPower(1);
        robot.frontRight.setPower(1);
        robot.backRight.setPower(1);

        while(opModeIsActive() && !(robot.frontLeft.getCurrentPosition() - robot.frontLeft.getTargetPosition() <= 10 && robot.frontLeft.getCurrentPosition() - robot.frontLeft.getTargetPosition() >= -10)){

        }

        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void shafferInches(double inches, Direction direction){
        int ticks = (int)(inches * robot.TICKS_PER_INCH);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if(direction == Direction.LEFT){
            robot.frontLeft.setTargetPosition(-ticks);
            robot.frontRight.setTargetPosition(ticks);
            robot.backLeft.setTargetPosition(ticks);
            robot.backRight.setTargetPosition(-ticks);

            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.frontLeft.setPower(1);
            robot.backLeft.setPower(1);
            robot.frontRight.setPower(1);
            robot.backRight.setPower(1);

            while(opModeIsActive() && !(robot.frontLeft.getCurrentPosition() - robot.frontLeft.getTargetPosition() <= 10 && robot.frontLeft.getCurrentPosition() - robot.frontLeft.getTargetPosition() >= -10)){

            }

            robot.frontLeft.setPower(0);
            robot.backLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);

            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else if (direction == Direction.RIGHT){
            robot.frontLeft.setTargetPosition(ticks);
            robot.frontRight.setTargetPosition(-ticks);
            robot.backLeft.setTargetPosition(ticks);
            robot.backRight.setTargetPosition(-ticks);

            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.frontLeft.setPower(1);
            robot.backLeft.setPower(1);
            robot.frontRight.setPower(1);
            robot.backRight.setPower(1);

            while(opModeIsActive() && !(robot.frontLeft.getCurrentPosition() - robot.frontLeft.getTargetPosition() <= 10 && robot.frontLeft.getCurrentPosition() - robot.frontLeft.getTargetPosition() >= -10)){

            }

            robot.frontLeft.setPower(0);
            robot.backLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);

            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void strafeXY(int y, int x){
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeft.setTargetPosition((int) ((y+x)*robot.TICKS_PER_INCH));
        robot.frontRight.setTargetPosition((int) ((y-x)*robot.TICKS_PER_INCH));
        robot.backLeft.setTargetPosition((int) ((y-x)*robot.TICKS_PER_INCH));
        robot.backRight.setTargetPosition((int) ((y+x)*robot.TICKS_PER_INCH));

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeft.setPower((robot.frontLeft.getTargetPosition() - robot.frontLeft.getCurrentPosition())/-1000 + .3);
        robot.backLeft.setPower((robot.frontRight.getTargetPosition() - robot.frontRight.getCurrentPosition())/-1000 + .3);
        robot.frontRight.setPower((robot.backLeft.getTargetPosition() - robot.backLeft.getCurrentPosition())/-1000 + .3);
        robot.backRight.setPower((robot.backRight.getTargetPosition() - robot.backRight.getCurrentPosition())/-1000 + .3);

        while(!(robot.frontLeft.getTargetPosition() - robot.frontLeft.getCurrentPosition() > -10 && robot.frontLeft.getTargetPosition() - robot.frontLeft.getCurrentPosition() < 10
                    && robot.frontRight.getTargetPosition() - robot.frontRight.getCurrentPosition() > -10 && robot.frontRight.getTargetPosition() - robot.frontRight.getCurrentPosition() < 10)) {
            telemetry.addData("FL/BR Target", robot.frontLeft.getTargetPosition());
            telemetry.addData("FR/BL Target", robot.frontRight.getTargetPosition());
            telemetry.addData("FL/BR Position", robot.frontLeft.getCurrentPosition());
            telemetry.addData("FR/BL Position", robot.frontRight.getCurrentPosition());
            telemetry.update();
            robot.frontLeft.setPower((robot.frontLeft.getTargetPosition() - robot.frontLeft.getCurrentPosition())/-500 + .3);
            robot.frontRight.setPower((robot.frontRight.getTargetPosition() - robot.frontRight.getCurrentPosition())/-500 + .3);
            robot.backLeft.setPower((robot.backLeft.getTargetPosition() - robot.backLeft.getCurrentPosition())/-500 + .3);
            robot.backRight.setPower((robot.backRight.getTargetPosition() - robot.backRight.getCurrentPosition())/-500 + .3);
        }

        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
    }

    public void zeroRobotAngle(){
        turnByAngle(-robot.getAngle());
    }

    public void turnByAngle(double turnAngle) {
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

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

            robot.frontLeft.setPower(total);
            robot.frontRight.setPower(-total);
            robot.backLeft.setPower(total);
            robot.backRight.setPower(-total);

            telemetry.update();
        }

        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
    }

    public void spinDuck(int time, Side side){
        if(side == Side.RED){
            robot.spinner.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        robot.spinner.setPower(.6);
        robot.safeWait(time);
        robot.spinner.setPower(0);
    }

    public void moveArm(Height height){
        int armTarget = 0;
        double finishPower = 0;
        if(height == Height.SHELF_HIGH){
            armTarget = 475;
            finishPower = .05;
        } else if(height == Height.SHELF_MID){
            armTarget = 250;
            finishPower = .2;
        }else if(height == Height.HIGH){
            armTarget = 1250;
        } else if(height == Height.INTAKE){
            armTarget = 1875;
            finishPower = -.2;
        } else if(height == Height.MEDIUM){
            armTarget = 1450;
        } else if(height == Height.LOW){
            armTarget = 1600;
        }
        ElapsedTime runtime = new ElapsedTime();
        double pArm = 0;
        double iArm = 0;
        double dArm = 0;
        double kp = 0.7;
        double ki = .5;
        double kd = .025;
        double error = armTarget - robot.arm.getCurrentPosition();
        runtime.reset();

        while(opModeIsActive() && !(error < 28 && error > -28)){
            error = armTarget-robot.arm.getCurrentPosition();
            double secs = runtime.seconds();
            runtime.reset();
            telemetry.addData("Arm Target", armTarget);
            telemetry.addData("Arm Position", robot.arm.getCurrentPosition());
            telemetry.addData("Arm Loop Time", secs);
            telemetry.update();
            dArm = (error - pArm) / secs;
            iArm = iArm + (error * secs);
            pArm = error;
            double total = (kp*pArm + ki*iArm + kd*dArm)/100;
            if(total > 1){
                iArm = 0;
                total = 1;
            }
            if(total < -1){
                iArm = 0;
                total = -1;
            }
            robot.arm.setPower(finishPower);
        }
    }
}