package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.OpenCVPipelines.RI3W11588OpenCV;
import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.Qual11588Hardware;
import org.firstinspires.ftc.teamcode.PowerPlay11588.TeleOp.Qual11588TeleOp;

public class Qual11588BaseAuto extends LinearOpMode {
    public AllianceColor teamColor;
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
    public enum AllianceColor{
        RED,
        BLUE
    }
    @Override
    public void runOpMode() throws InterruptedException {

    }
    public void setClaw(Qual11588TeleOp.ClawPosition clawPosition){
        switch (clawPosition){
            case Open:
                robot.claw.setPosition(.5);
                break;
            case Closed:
                // get numbers from Nick, position should not be zero because its bad for motor
                robot.claw.setPosition(0);
        }
    }
    public void pivot(double degrees, boolean clockwise){
        // To be written
        //
    }
    public void moveXY(double y, double x){
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
        RI3W11588OpenCV.ConeColor color = robot.pipeLine.getColor(teamColor);
        if(robot.pipeLine.coneColor == RI3W11588OpenCV.ConeColor.red){
            telemetry.addData("Saw red, going to spot 1", "");
            // Move left 24 inches
            moveXY(0, -24);
            // Move forward 26 inches
            moveXY((int) 26, 0);
        }else if(robot.pipeLine.coneColor == RI3W11588OpenCV.ConeColor.green) {
            telemetry.addData("Saw green, going to spot 2", "");
            // Move forward 26 inches
            moveXY(26, 0);

        }else if(robot.pipeLine.coneColor == RI3W11588OpenCV.ConeColor.blue){
            telemetry.addData("Saw blue, going to spot 3", "");
            // Move right 24 inches
            moveXY(0, 24);
            // Move forward 26 inches
            moveXY((int) 26, 0);

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

    public void placeCone() {
        moveArm(Height.MEDIUM);
        moveXY(0, 12);
        moveXY(6, 0);
        moveArm(Height.LOW);
        robot.claw.setPosition(1);
    }
    public void coneAndPark(){
        RI3W11588OpenCV.ConeColor color = robot.pipeLine.getColor(teamColor);
        /*
        step 1:
        move forward 37.5 inches
        */
        moveXY(37.5, 0);
        while(!gamepad1.a){

        }
        /*
        Step 2:
        Pivot clockwise 90 degrees
         */
        pivot(90, true);
        while(!gamepad1.a){

        }
        /*
        Step 3:
        raise the arm to the medium junction
         */
        moveArm(Height.MEDIUM);
        while(!gamepad1.a){

        }
        /*
        Step 4:
        Open the claw
         */
        setClaw(Qual11588TeleOp.ClawPosition.Open);
        while(!gamepad1.a){

        }
        /*
        Step 5:
        if you're in location 2, you're done.
         */
        if(robot.pipeLine.coneColor == RI3W11588OpenCV.ConeColor.green) {
            telemetry.addData("Saw green, finished", "");
            return;
        } else {
            //For both location 1 and 3
            moveXY(0, 12);
            while(!gamepad1.a){

            }
            if(robot.pipeLine.coneColor == RI3W11588OpenCV.ConeColor.red){
                telemetry.addData("Saw red, going to spot 1", "");
                moveXY(-24, 0);
            } else {
                telemetry.addData("Saw blue, going to spot 3", "");
                moveXY(24, 0);
            }
        }
    }

}