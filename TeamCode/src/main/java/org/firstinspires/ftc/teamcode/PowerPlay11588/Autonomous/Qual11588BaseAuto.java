package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.OpenCVPipelines.RI3W11588OpenCV;
import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.Qual11588Hardware;

public class Qual11588BaseAuto extends LinearOpMode {
    Qual11588Hardware robot = new Qual11588Hardware();
    ElapsedTime PIDTimer = new ElapsedTime();
    //Defining the variables outside the method so they can be used in a telemetry method
    int armTarget = 50;
    int error = 0;
    double total = 0;
    double kp = 0;
    double ki = 0;
    double kd = 0;
    double pArm = 0;
    double iArm = 0;
    double dArm = 0;
    double iArmMax = .25;
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
            telemetryStuff();
            /*
            telemetry.addData("Front Left Target", robot.frontLeft.getTargetPosition());
            telemetry.addData("Front Left Position", robot.frontLeft.getCurrentPosition());
            telemetry.addData("Front Right Target", robot.frontRight.getTargetPosition());
            telemetry.addData("Front Right Position", robot.frontRight.getCurrentPosition());
            telemetry.addData("Back Left Target", robot.backLeft.getTargetPosition());
            telemetry.addData("Back Left Position", robot.backLeft.getCurrentPosition());
            telemetry.addData("Back Right Target", robot.backRight.getTargetPosition());
            telemetry.addData("Back Right Position", robot.backRight.getCurrentPosition());
            telemetry.update();
            */
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
            //test
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
        if(height == Height.GROUND){
            armTarget = 50;
        } else if(height == Height.LOW){
            armTarget = 100;
        } else if(height == Height.MEDIUM){
            armTarget = 150;
        } else if(height == Height.HIGH){
            armTarget = 200;
        }
        error = armTarget - robot.arm.getCurrentPosition();
        PIDTimer.reset();

        while(opModeIsActive() && !(error < 5 && error > -5)){
            error = armTarget - robot.arm.getCurrentPosition();
            dArm = (error - pArm)/PIDTimer.seconds();
            iArm = iArm + (error * PIDTimer.seconds());
            pArm = error;
            total = ((kp*pArm) + (ki*iArm) + (kd*dArm))/100;
            robot.arm.setPower(total);

            if(iArm > iArmMax){
                iArm = iArmMax;
            } else if(iArm < -iArmMax){
                iArm = -iArmMax;
            }
            PIDTimer.reset();
        }
    }
    public void telemetryStuff(){
        telemetry.addData("Front Left Target", robot.frontLeft.getTargetPosition());
        telemetry.addData("Front Left Position", robot.frontLeft.getCurrentPosition());
        telemetry.addData("Front Right Target", robot.frontRight.getTargetPosition());
        telemetry.addData("Front Right Position", robot.frontRight.getCurrentPosition());
        telemetry.addData("Back Left Target", robot.backLeft.getTargetPosition());
        telemetry.addData("Back Left Position", robot.backLeft.getCurrentPosition());
        telemetry.addData("Back Right Target", robot.backRight.getTargetPosition());
        telemetry.addData("Back Right Position", robot.backRight.getCurrentPosition());
        telemetry.addData("Arm Target", armTarget);
        telemetry.addData("Arm Position", robot.arm.getCurrentPosition());
        telemetry.addData("Arm Error", error);
        telemetry.addData("Arm Power", total);
        telemetry.update();
    }
}