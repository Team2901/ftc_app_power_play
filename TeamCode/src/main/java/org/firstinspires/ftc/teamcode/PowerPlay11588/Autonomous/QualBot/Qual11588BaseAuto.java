package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous.QualBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.OpenCVPipelines.Qual11588OpenCV;
import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.Qual11588Hardware;

public class Qual11588BaseAuto extends LinearOpMode {
    public AllianceColor teamColor;
    public Qual11588Hardware robot = new Qual11588Hardware();
    ElapsedTime PIDTimer = new ElapsedTime();
    ElapsedTime safeWaitTimer = new ElapsedTime();
    ElapsedTime PIDTimeout = new ElapsedTime();
    //Defining the pidvariables outside the method so they can be used in a telemetry method
    int armTarget = 200;
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
    double ffTotal = 0;
    String colorSeen = "none";

    double startAngle = 0;
    double targetAngle = 0;
    double currentAngle = 0;
    double turnError = 0;
    double turnPower = .5;

    public enum Height{
        GROUND,
        LOW,
        MEDIUM,
        HIGH,
        STACK5,
        STACK4,
        STACK3,
        STACK2,
        STACK1
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

    public void moveXYPID(double y, double x, double power){
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

        robot.frontLeft.setPower(power);
        robot.frontRight.setPower(power);
        robot.backLeft.setPower(power);
        robot.backRight.setPower(power);

        while (opModeIsActive() && (robot.frontLeft.isBusy() || robot.frontRight.isBusy() ||
                robot.backLeft.isBusy() || robot.backRight.isBusy())){

            armAngle = recalculateAngle();
            cosArm = Math.cos(Math.toRadians(armAngle));
            ffTotal = cosArm * kCos;
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

    public void moveXYPID(double y, double x){
        moveXYPID(y, x, .5);
    }

    public void moveXYAndArm(double y, double x, Height height){
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
            }else if(total < .005 && armAngle < 60){
                total = .005;
            }
            robot.arm.setPower(total);
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

        armAngle = (90.0/(1200.0 - 400.0)) * (robot.arm.getCurrentPosition() - 400.0);
        //armAngle = 0.102856 * robot.arm.getCurrentPosition() - 43.6276;
        cosArm = Math.cos(Math.toRadians(armAngle));
        ffTotal = cosArm * kCos;
        robot.arm.setPower(ffTotal);
    }

    public void moveArm(Height height){
        /*
        if(height == Height.GROUND){
            armTarget = 200;
        } else if(height == Height.LOW){
            armTarget = 550;
        } else if(height == Height.MEDIUM){
            armTarget = 800;
        } else if(height == Height.HIGH){
            armTarget = 1200;
        } else if(height == Height.STACK5){
            armTarget = 350;
        }

         */

        switch (height){
            case GROUND:
                armTarget = 200;
                break;
            case LOW:
                armTarget = 550;
                break;
            case MEDIUM:
                armTarget = 800;
                break;
            case HIGH:
                armTarget = 1150;
                break;
            case STACK5:
                armTarget = 310;
                break;
            case STACK4:
                armTarget = 275;
                break;
            case STACK3:
                armTarget = 240;
                break;
            case STACK2:
                armTarget = 205;
                break;
            case STACK1:
                armTarget = 170;
                break;
        }
        error = armTarget - robot.arm.getCurrentPosition();
        PIDTimer.reset();
        PIDTimeout.reset();

        while(opModeIsActive() && !(error < 10 && error > -10) && PIDTimeout.milliseconds() < 3000){
            error = armTarget - robot.arm.getCurrentPosition();
            dArm = (error - pArm)/PIDTimer.seconds();
            iArm = iArm + (error * PIDTimer.seconds());
            pArm = error;
            armAngle = (90.0/(1200.0 - 400.0)) * (robot.arm.getCurrentPosition() - 400.0);
            //armAngle = 0.102856 * robot.arm.getCurrentPosition() - 43.6276;
            cosArm = Math.cos(Math.toRadians(armAngle));
            total = ((kp*pArm) + (ki*iArm) + (kd*dArm))/100 + (kCos *cosArm);

            PIDTimer.reset();
            if(iArm > iArmMax){
                iArm = iArmMax;
            } else if(iArm < -iArmMax){
                iArm = -iArmMax;
            }
            if(total > .6){
                total = .6;
            }
            if(armAngle > 60 && total < -.3){
                total = -.3;
            }else if(total < .001 && armAngle < 60){
                total = .001;
            }

            robot.arm.setPower(total);

            telemetryStuff();
        }
        armAngle = recalculateAngle();
        //armAngle = 0.102856 * robot.arm.getCurrentPosition() - 43.6276;
        cosArm = Math.cos(Math.toRadians(armAngle));
        ffTotal = cosArm * kCos;
        robot.arm.setPower(ffTotal);

    }

    public double recalculateAngle(){
        //Placeholder variables that will be deleted
        double rightAngleDiff = 800;
        double slope = 90/((400 + rightAngleDiff) - 400);
        double newAngle = slope * (robot.arm.getCurrentPosition() - 400);
        return newAngle;
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

            armAngle = recalculateAngle();
            cosArm = Math.cos(Math.toRadians(armAngle));
            ffTotal = cosArm * kCos;
            robot.arm.setPower(ffTotal);
            telemetryStuff();
        }
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);
    }
    public void turnToAngle(double turnAngle){
        //robot.getAngle is between -180 and 180, starting at 0
        targetAngle = AngleUnit.normalizeDegrees(turnAngle) + 180;
        startAngle = robot.getAngle() + 180;
        turnError = AngleUnit.normalizeDegrees(targetAngle - startAngle);
        while(opModeIsActive() && !(turnError < .5 && turnError > -.5)){
            if(turnError >= 0){
                turnPower = turnError/50;
                if(turnPower > .75){
                    turnPower = .75;
                }
            }else if(turnError < 0){
                turnPower = turnError/50;
                if(turnPower < -.75){
                    turnPower = -.75;
                }
            }
            robot.frontLeft.setPower(-turnPower);
            robot.frontRight.setPower(turnPower);
            robot.backLeft.setPower(-turnPower);
            robot.backRight.setPower(turnPower);

            currentAngle = robot.getAngle() + 180;
            turnError = AngleUnit.normalizeDegrees(targetAngle - currentAngle);

            armAngle = recalculateAngle();
            cosArm = Math.cos(Math.toRadians(armAngle));
            ffTotal = cosArm * kCos;
            robot.arm.setPower(ffTotal);
            telemetryStuff();
        }
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);
    }

    public void safeWaitPID(int milliseconds){
        safeWaitTimer.reset();
        while(opModeIsActive() && safeWaitTimer.milliseconds() < milliseconds){
            armAngle = recalculateAngle();
            cosArm = Math.cos(Math.toRadians(armAngle));
            ffTotal = cosArm * kCos;
            robot.arm.setPower(ffTotal);
            telemetryStuff();
        }
    }

    public void coneAndPark(boolean goLeft){
        Qual11588OpenCV.ConeColor color = robot.pipeLine.getColor();
        if(goLeft) {
            moveXYPID(0, -42);
            moveXYPID(22, 0);
            moveArm(Height.HIGH);
            moveXYPID(4, 0);
            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            while (timer.milliseconds() > 1000) {}
            moveArm(Height.HIGH);
            robot.claw.setPosition(robot.OPEN_POSITION);
            moveXYPID(4, 0);
            if(color == Qual11588OpenCV.ConeColor.RED){
                telemetry.addData("Saw red, going to spot 1", "");
                moveXYPID(0, 18);
            }else if(color == Qual11588OpenCV.ConeColor.GREEN) {
                telemetry.addData("Saw green, going to spot 2", "");
                moveXYPID(0, 42);
            }else if(color == Qual11588OpenCV.ConeColor.BLUE){
                telemetry.addData("Saw blue, going to spot 3", "");
                moveXYPID(0, 68);
            }
            moveArm(Height.LOW);
            moveArm(Height.GROUND);
        } else {
            moveXYPID(0, 42);
            moveXYPID(22, 0);
            moveArm(Height.HIGH);
            moveXYPID(4, 0);
            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            while (timer.milliseconds() > 1000) {}
            moveArm(Height.HIGH);
            robot.claw.setPosition(robot.OPEN_POSITION);
            moveXYPID(4, 0);
            if(color == Qual11588OpenCV.ConeColor.RED){
                moveXYPID(0, -68);
                telemetry.addData("Saw red, going to spot 1", "");
            }else if(color == Qual11588OpenCV.ConeColor.GREEN) {
                telemetry.addData("Saw green, going to spot 2", "");
                moveXYPID(0, -42);
            }else if(color == Qual11588OpenCV.ConeColor.BLUE){
                telemetry.addData("Saw blue, going to spot 3", "");
                moveXYPID(0, -18);
            }
            moveArm(Height.LOW);
            moveArm(Height.GROUND);
        }
    }

    public void coneAndPark() {
        coneAndPark(true);
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
            moveXYPID(0, -28);
            // Move forward 26 inches
            moveXYPID((int) 35, 0);
        }else if(color == Qual11588OpenCV.ConeColor.GREEN) {
            telemetry.addData("Saw green, going to spot 2", "");
            // Move forward 26 inches
            moveXYPID(35, 0);
        }else if(color == Qual11588OpenCV.ConeColor.BLUE){
            telemetry.addData("Saw blue, going to spot 3", "");
            // Move right 24 inches
            moveXYPID(0, 28);
            // Move forward 26 inches
            moveXYPID((int) 35, 0);
        }
        moveArm(Height.LOW);
        moveArm(Height.GROUND);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.milliseconds() < 1000);
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

    public void telemetryStuff(){
        /*
        telemetry.addData("Frames Processed", robot.pipeLine.framesProceeded);
        telemetry.addData("Red seen", robot.pipeLine.redAmount);
        telemetry.addData("Green seen", robot.pipeLine.greenAmount);
        telemetry.addData("Blue seen", robot.pipeLine.blueAmount);
        telemetry.addData("Color decided", colorSeen);
         */
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
        telemetry.addData("Arm Angle", armAngle);
        telemetry.addData("Arm Error", error);
        telemetry.addData("Arm Power", robot.arm.getPower());
        telemetry.addData("PID Total", total);
        telemetry.addData("Feed Forward", ffTotal);
        telemetry.addData("Alliance", teamColor);
        telemetry.addData("Robot Angle", robot.getAngle());
        telemetry.addData("Target Angle", targetAngle);
        telemetry.addData("Turn Error", turnError);
        telemetry.update();
    }
}