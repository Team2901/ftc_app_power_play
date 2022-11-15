package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.Qual11588Hardware;

public class Qual11588BaseAuto extends LinearOpMode {
    Qual11588Hardware robot = new Qual11588Hardware();
    public enum Height{
        GROUND,
        LOW,
        MEDIUM,
        HIGH
    }
    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void moveXY(int y, int x){
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
            telemetry.addData("Front Left Target", robot.frontLeft.getTargetPosition());
            telemetry.addData("Front Left Position", robot.frontLeft.getCurrentPosition());
            telemetry.addData("Front Right Target", robot.frontRight.getTargetPosition());
            telemetry.addData("Front Right Position", robot.frontRight.getCurrentPosition());
            telemetry.addData("Back Left Target", robot.backLeft.getTargetPosition());
            telemetry.addData("Back Left Position", robot.backLeft.getCurrentPosition());
            telemetry.addData("Back Right Target", robot.backRight.getTargetPosition());
            telemetry.addData("Back Right Position", robot.backRight.getCurrentPosition());
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
    public void moveArm(Height height){
        int armTarget = 50;
        int lastTarget = armTarget;
        if(height == Height.GROUND){
            armTarget = 50;
        } else if(height == Height.LOW){
            armTarget = 100;
        } else if(height == Height.MEDIUM){
            armTarget = 150;
        } else if(height == Height.HIGH){
            armTarget = 200;
        }
        ElapsedTime PIDTimer = new ElapsedTime();

        double kp = 0;
        double ki = 0;
        double kd = 0;
        double pArm = 0;
        double iArm = 0;
        double dArm = 0;
        double iArmMax = .25;

        double error = armTarget - robot.arm.getCurrentPosition();
        PIDTimer.reset();

        while(opModeIsActive() && !(error < 5 && error > -5)){
            error = armTarget - robot.arm.getCurrentPosition();
            dArm = (error - pArm)/PIDTimer.seconds();
            iArm = iArm + (error * PIDTimer.seconds());
            pArm = error;
            double total = ((kp*pArm) + (ki*iArm) + (kd*dArm))/100;
            robot.arm.setPower(total);

            if(iArm > iArmMax){
                iArm = iArmMax;
            } else if(iArm < -iArmMax){
                iArm = -iArmMax;
            }

            PIDTimer.reset();
        }
    }
}