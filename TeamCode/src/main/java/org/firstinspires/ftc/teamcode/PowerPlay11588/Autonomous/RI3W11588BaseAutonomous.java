package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous;

import static org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.RI3W11588Hardware.FRONT_GEAR_RATIO;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.OpenCVPipelines.RI3W11588OpenCV;
import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.RI3W11588Hardware;

public class RI3W11588BaseAutonomous extends LinearOpMode {
    RI3W11588Hardware robot = new RI3W11588Hardware();
    enum Height{
        INTAKE,
        GROUND,
        LOW,
        MEDIUM,
        HIGH
    }
    //These are set positions for the arm

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void moveXY(int y, int x){
        int frontTicksY = (int) (y * robot.FRONT_TICKS_PER_INCH);
        int frontTicksX = (int) (x * robot.FRONT_TICKS_PER_INCH);
        int backTicksY = (int) (y * robot.BACK_TICKS_PER_INCH);
        int backTicksX = (int) (x * robot.BACK_TICKS_PER_INCH);
        //This method takes in a parameter of inches, we must convert these to ticks

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeft.setTargetPosition(frontTicksY + frontTicksX);
        robot.frontRight.setTargetPosition(frontTicksY - frontTicksX);
        robot.backLeft.setTargetPosition(backTicksY - backTicksX);
        robot.backRight.setTargetPosition(backTicksY + backTicksX);
        //We will want to be moving forwards, and some of the wheels are actually positioned backwards, so we must set their desitination negative

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeft.setPower(0.5);
        robot.frontRight.setPower(0.5);
        robot.backLeft.setPower(0.5);
        robot.backRight.setPower(0.5);

        while(opModeIsActive() && (robot.frontLeft.isBusy() || robot.frontRight.isBusy() || robot.backLeft.isBusy() ||
                robot.backRight.isBusy())){
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

    public void park(){
        if(robot.pipeLine.coneColor == RI3W11588OpenCV.ConeColor.red){
            telemetry.addData("Saw red, going to spot 1", "");
            // Move forward 24 inches
            moveXY((int) 27, 0);
            // Move left 27 inches
            moveXY(0, -30);
        }else if(robot.pipeLine.coneColor == RI3W11588OpenCV.ConeColor.green) {
            telemetry.addData("Saw green, going to spot 2", "");
            // Move forward 36 inches
            moveXY(36, 0);

        }else if(robot.pipeLine.coneColor == RI3W11588OpenCV.ConeColor.blue){
            telemetry.addData("Saw blue, going to spot 3", "");
            // Move forward 24 inches
            moveXY((int) 27, 0);
            // Move right 27 inches
            moveXY(0, 30);
        }
    }
    public void moveArm(Height height){
        int armTarget = 40;
        int lastTarget = armTarget;
        if(height == Height.INTAKE){
            armTarget = 40;
        }else if(height == Height.GROUND){
            armTarget = 40;
        }else if(height == Height.LOW){
            armTarget = 200;
        }else if(height == Height.MEDIUM){
            armTarget = 315;
        }else if(height == Height.HIGH){
            armTarget = 400;
        }

        ElapsedTime pidTimer = new ElapsedTime();

        double kp = 0.5;
        double ki = 0.5;
        double kd = 0.01;
        double pArm = 0;
        double iArm = 0;
        double dArm = 0;
        double iArmMax = .25;

        double error = armTarget - robot.arm.getCurrentPosition();
        pidTimer.reset();

        while(opModeIsActive() && !(error < 5 && error > -5) && !(dArm < .02 && dArm > 0)){
            error = armTarget - robot.arm.getCurrentPosition();
            //Subtract the old error from the current one. Could use a separate variable
            // but pArm hasn't been updated to the new error yet so why not
            dArm = (error - pArm) / pidTimer.seconds();
            iArm = iArm + (error * pidTimer.seconds());
            pArm = error;

            if(armTarget != lastTarget){
                iArm = 0;
            }
            if (iArm > iArmMax){
                iArm = iArmMax;
            }else if (iArm < -iArmMax){
                iArm = -iArmMax;
            }
            double total = ((kp * pArm) + (ki * iArm) + (kd * dArm))/100;

            if(total > .5) {
                total = .5;
            }

            robot.arm.setPower(total);

            telemetry.addData("Arm Error", error);
            telemetry.addData("Arm Target", armTarget);
            telemetry.addData("Arm Position", robot.arm.getCurrentPosition());
            telemetry.addData("Arm Power", total);
            telemetry.addData("Red", robot.pipeLine.redAmount);
            telemetry.addData("Blue", robot.pipeLine.blueAmount);
            telemetry.addData("Green", robot.pipeLine.greenAmount);
            telemetry.addData("Cone Color", robot.pipeLine.coneColor);
            telemetry.update();

            pidTimer.reset();
        }
        robot.arm.setPower(0.005);
    }
}