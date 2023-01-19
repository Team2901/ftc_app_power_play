package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    String colorSeen = "none";

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
            armAngle = (90.0/(1200 - 400)) * (robot.arm.getCurrentPosition() - 400);
            //armAngle = 0.102856 * robot.arm.getCurrentPosition() - 43.6276;
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

    public void reconParkAuto() {
        Qual11588OpenCV.ConeColor color = robot.pipeLine.getColor();
        if(color == Qual11588OpenCV.ConeColor.RED){
            telemetry.addData("Saw red, going to spot 1", "");
            // Move left 24 inches
            moveXYPID(0, -26);
            // Move forward 26 inches
            moveXYPID((int) 35, 0);
        }else if(color == Qual11588OpenCV.ConeColor.GREEN) {
            telemetry.addData("Saw green, going to spot 2", "");
            // Move forward 26 inches
            moveXYPID(35, 0);
        }else if(color == Qual11588OpenCV.ConeColor.BLUE){
            telemetry.addData("Saw blue, going to spot 3", "");
            // Move right 24 inches
            moveXYPID(0, 24);
            // Move forward 26 inches
            moveXYPID((int) 35, 0);
        }
        moveArm(Height.GROUND);
    }

    public void recon() {
        if(robot.pipeLine.coneColor == Qual11588OpenCV.ConeColor.RED){
            colorSeen = "red";
        }else if(robot.pipeLine.coneColor == Qual11588OpenCV.ConeColor.GREEN) {
            colorSeen = "green";
        }else if(robot.pipeLine.coneColor == Qual11588OpenCV.ConeColor.BLUE){
            colorSeen = "blue";
        }
        ElapsedTime waitTimer = new ElapsedTime();
        while(waitTimer.milliseconds() < 10000) {}
    }

    public void moveArm(Height height){
        if(height == Height.GROUND){
            armTarget = 200;
        } else if(height == Height.LOW){
            armTarget = 550;
        } else if(height == Height.MEDIUM){
            armTarget = 800;
        } else if(height == Height.HIGH){
            armTarget = 1150;
        }
        error = armTarget - robot.arm.getCurrentPosition();
        PIDTimer.reset();

        while(opModeIsActive() && !(error < 10 && error > -10)){
            error = armTarget - robot.arm.getCurrentPosition();
            dArm = (error - pArm)/PIDTimer.seconds();
            iArm = iArm + (error * PIDTimer.seconds());
            pArm = error;
            armAngle = (90.0/(1200 - 400)) * (robot.arm.getCurrentPosition() - 400);
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
            if(total > .6){
                total = .6;
            }
            if(total < .025){
                total = .025;
            }
            telemetryStuff();
        }
        armAngle = (90.0/(1200 - 400)) * (robot.arm.getCurrentPosition() - 400);
        //armAngle = 0.102856 * robot.arm.getCurrentPosition() - 43.6276;
        cosArm = Math.cos(Math.toRadians(armAngle));
        double ffTotal = cosArm * kCos;
        robot.arm.setPower(ffTotal);
    }

    public void turnByAngle(double turnAngle){
        startAngle = robot.getAngle();
        targetAngle = AngleUnit.normalizeDegrees(startAngle + turnAngle);
        turnError = targetAngle - robot.getAngle();
        while (opModeIsActive() && (turnError > 5 || turnError < -5)){
            turnError = targetAngle - robot.getAngle();
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
            turnError = targetAngle - robot.getAngle();
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
        moveXY(0, 36);
        moveXY(32, 0);
    }

    public void telemetryStuff(){
        telemetry.addData("Frames Processed", robot.pipeLine.framesProceeded);
        telemetry.addData("Red seen", robot.pipeLine.redAmount);
        telemetry.addData("Green seen", robot.pipeLine.greenAmount);
        telemetry.addData("Blue seen", robot.pipeLine.blueAmount);
        telemetry.addData("Color decided", colorSeen);
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