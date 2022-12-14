package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.OpenCVPipelines.Qual11588OpenCV;
import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.Qual11588Hardware;

public class Qual11588BaseAuto extends LinearOpMode {
    public AllianceColor teamColor;
    Qual11588Hardware robot = new Qual11588Hardware();
    ElapsedTime PIDTimer = new ElapsedTime();
    //Defining the pidvariables outside the method so they can be used in a telemetry method
    int armTarget = 50;
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

    double startAngle = 0;
    //Has targetAngle return -1 if it has not been defined, it is redefined before it is used
    double targetAngle = -1;
    double turnError = 0;
    double turnPower = .5;

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
            armAngle = 0.102856 * robot.arm.getCurrentPosition() - 43.6276;
            cosArm = Math.cos(Math.toRadians(armAngle));
            double ffTotal = cosArm * kCos;
            robot.arm.setPower(ffTotal);
            telemetryStuff();
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
        if(robot.pipeLine.coneColor == Qual11588OpenCV.ConeColor.RED){
            telemetry.addData("Saw red, going to spot 1", "");
            // Move left 24 inches
            moveXY(0, -24);
            // Move forward 26 inches
            moveXY((int) 26, 0);
        }else if(robot.pipeLine.coneColor == Qual11588OpenCV.ConeColor.GREEN) {
            telemetry.addData("Saw green, going to spot 2", "");
            // Move forward 26 inches
            moveXY(26, 0);
        }else if(robot.pipeLine.coneColor == Qual11588OpenCV.ConeColor.BLUE){
            telemetry.addData("Saw blue, going to spot 3", "");
            // Move right 24 inches
            moveXY(0, 24);
            // Move forward 26 inches
            moveXY((int) 26, 0);
        }
    }

    public void moveArm(int height){
        armTarget = height;
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
            telemetryStuff();
        }
    }

    public void moveArm(Height height){
        if(height == Height.GROUND){
            armTarget = 200;
        } else if(height == Height.LOW){
            armTarget = 475;
        } else if(height == Height.MEDIUM){
            armTarget = 775;
        } else if(height == Height.HIGH){
            armTarget = 1000;
        }
        error = armTarget - robot.arm.getCurrentPosition();
        PIDTimer.reset();

        while(opModeIsActive() && !(error < 5 && error > -5)){
            error = armTarget - robot.arm.getCurrentPosition();
            dArm = (error - pArm)/PIDTimer.seconds();
            iArm = iArm + (error * PIDTimer.seconds());
            pArm = error;
            armAngle = 0.102856 * robot.arm.getCurrentPosition() - 43.6276;
            cosArm = Math.cos(Math.toRadians(armAngle));
            total = ((kp*pArm) + (ki*iArm) + (kd*dArm))/100 + (kCos *cosArm);
            robot.arm.setPower(total);
            PIDTimer.reset();
            if(iArm > iArmMax){
                iArm = iArmMax;
            } else if(iArm < -iArmMax){
                iArm = -iArmMax;
            }
            if(total > .75){
                total = .75;
            }
            if(total < .01){
                total = .01;
            }
            telemetryStuff();
        }
    }

    public void turnByAngle(double turnAngle){
        startAngle = robot.getAngle();
        targetAngle = AngleUnit.normalizeDegrees(startAngle + turnAngle);
        turnError = targetAngle - robot.getAngle();
        while (opModeIsActive() && !(turnError < 5 && turnError > -5)){
            if(turnError < 0){
                turnPower = -.5;
            }else if(turnError > 0){
                turnPower = .5;
            }
            robot.frontLeft.setPower(turnPower);
            robot.frontRight.setPower(-turnPower);
            robot.backLeft.setPower(turnPower);
            robot.backRight.setPower(-turnPower);
            telemetryStuff();
        }
    }

    public void placeCone() {
        moveArm(Height.MEDIUM);
        moveXY(0, 12);
        moveXY(6, 0);
        moveArm(Height.LOW);
        robot.claw.setPosition(robot.OPEN_POSITION);
    }

    public void coneAndPark(){
        /*
        step 1:
        move forward 37.5 inches
        */
        moveXY(37.5, 0);
        while(opModeIsActive() && !gamepad1.a){
            telemetryStuff();
        }
        /*
        Step 2:
        Pivot clockwise 90 degrees
         */
        turnByAngle(90); // + = counter-clockwise, - = clockwise ??? yes right hand rule
        while(opModeIsActive()  && !gamepad1.a){
            telemetryStuff();
        }
        /*
        Step 3:
        raise the arm to the medium junction
         */
        moveArm(Height.MEDIUM);
        while(opModeIsActive() && !gamepad1.a){
            telemetryStuff();
        }
        /*
        Step 4:
        Open the claw
         */
        robot.claw.setPosition(robot.OPEN_POSITION);
        while(opModeIsActive() && !gamepad1.a){
            telemetryStuff();
        }
        /*
        Step 5:
        if you're in location 2, you're done.
         */
        robot.pipeLine.coneColor = Qual11588OpenCV.ConeColor.GREEN;
        if(robot.pipeLine.coneColor == Qual11588OpenCV.ConeColor.GREEN) {
            telemetry.addData("Saw green, finished", "");
            return;
        } else {
            //For both location 1 and 3
            moveXY(0, 12);
            while(opModeIsActive() && !gamepad1.a){

            }
            if(robot.pipeLine.coneColor == Qual11588OpenCV.ConeColor.RED){
                telemetry.addData("Saw red, going to spot 1", "");
                moveXY(-24, 0);
            } else {
                telemetry.addData("Saw blue, going to spot 3", "");
                moveXY(24, 0);
            }
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
        telemetry.addData("Alliance", teamColor);
        telemetry.addData("Current Angle", robot.getAngle());
        telemetry.addData("Target Angle", targetAngle);
        telemetry.update();
    }
}