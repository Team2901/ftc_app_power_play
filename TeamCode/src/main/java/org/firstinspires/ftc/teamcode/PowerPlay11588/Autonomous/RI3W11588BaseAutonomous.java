package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.OpenCVPipelines.RI3W11588OpenCV;
import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.RI3W11588Hardware;

public class RI3W11588BaseAutonomous extends LinearOpMode {
    RI3W11588Hardware robot = new RI3W11588Hardware();
    RI3W11588OpenCV pipeline = new RI3W11588OpenCV(telemetry);
    enum Height{
        INTAKE,
        GROUND,
        LOW,
        MEDIUM,
        HIGH
    }
    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void moveXY(int y, int x){
        int frontTicksY = (int) (y * robot.FRONT_TICKS_PER_INCH);
        int frontTicksX = (int) (x * robot.FRONT_TICKS_PER_INCH);
        int backTicksY = (int) (y * robot.BACK_TICKS_PER_INCH);
        int backTicksX = (int) (x * robot.BACK_TICKS_PER_INCH);

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeft.setTargetPosition(frontTicksY + frontTicksX);
        robot.frontRight.setTargetPosition(frontTicksY - frontTicksX);
        robot.backLeft.setTargetPosition(backTicksY - backTicksX);
        robot.backRight.setTargetPosition(backTicksY + backTicksX);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeft.setPower(0.5);
        robot.frontRight.setPower(0.5);
        robot.backLeft.setPower(0.5);
        robot.backRight.setPower(0.5);

        while(opModeIsActive() || (robot.frontLeft.isBusy() || robot.frontRight.isBusy() || robot.backLeft.isBusy() ||
                robot.backRight.isBusy())){
            telemetry.addData("Front Left Position", robot.frontLeft.getCurrentPosition());
            telemetry.addData("Front Right Position", robot.frontRight.getCurrentPosition());
            telemetry.addData("Front Right Position", robot.backLeft.getCurrentPosition());
            telemetry.addData("Front Right Position", robot.backRight.getCurrentPosition());
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
    public void park(){
        if(pipeline.coneColor == RI3W11588OpenCV.ConeColor.red){
            telemetry.addData("Saw red, going to spot 1", "");
            // Move left 24 inches
            moveXY(0, -24);
            // Move forward 36 inches
            moveXY(36, 0);
        }else if(pipeline.coneColor == RI3W11588OpenCV.ConeColor.green) {
            telemetry.addData("Saw green, going to spot 2", "");
            // Move left 24 inches
            moveXY(0, -24);
            // Move forward 36 inches
            moveXY(36, 0);
            // Move right 24 inches
            moveXY(0, 24);
        }else if(pipeline.coneColor == RI3W11588OpenCV.ConeColor.blue){
            telemetry.addData("Saw blue, going to spot 3", "");
            // Move right 24 inches
            moveXY(0, 24);
            // Move forward 36 inches
            moveXY(36, 0);
        }
    }
    public void moveArm(Height height){
        int armTarget = 0;
        if(height == Height.INTAKE){
            armTarget = 0;
        }else if(height == Height.GROUND){
            armTarget = 50;
        }else if(height == Height.LOW){
            armTarget = 100;
        }else if(height == Height.MEDIUM){
            armTarget = 200;
        }else if(height == Height.HIGH){
            armTarget = 300;
        }

        ElapsedTime pidTimer = new ElapsedTime();

        double kp = 0;
        double ki = 0;
        double kd = 0;
        double pArm = 0;
        double iArm = 0;
        double dArm = 0;

        double error = armTarget - robot.arm.getCurrentPosition();

        while(opModeIsActive() && !(error < 30 && error > -30)){
            error = armTarget - robot.arm.getCurrentPosition();
            //Subtract the old error from the current one. Could use a separate variable
            // but pArm hasn't been updated to the new error yet so why not
            dArm = (error - pArm) / pidTimer.seconds();
            iArm = iArm + (error * pidTimer.seconds());
            pArm = error;

            double total = (kp * pArm) + (ki * iArm) + (kd * dArm);

            robot.arm.setPower(total);

            pidTimer.reset();
        }
    }
}