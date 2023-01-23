package org.firstinspires.ftc.teamcode.PowerPlay11588.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.SmolBot11588Hardware;

@TeleOp(name = "SmolBot 11588 TeleOp", group = "11588")
public class SmolBot11588TeleOp extends OpMode {
    SmolBot11588Hardware robot = new SmolBot11588Hardware();
    public enum ClawPosition{Open, Closed}
    ClawPosition currentClawPosition = ClawPosition.Closed;

    double turningPower = 0.0;

    //Everything Lift Pid
    public enum LiftHeight{GROUND, LOW, MID, HIGH, STACK}
    LiftHeight currentLiftHeight = LiftHeight.GROUND;
    LiftHeight lastLiftHeight = currentLiftHeight;
    ElapsedTime LiftPIDTimer = new ElapsedTime();
    //All Lift Pid variables
    int liftTarget = 200;
    int groundLiftPosition = 200;
    int stackLiftPosition = 400;
    int lowLiftPosition = 550;
    int midLiftPosition = 800;
    int highLiftPosition = 1150;
    double liftError = 0.0;
    double liftTotal = 0.0;
    double pLift = 0.0;
    double iLift = 0.0;
    double dLift = 0.0;
    double fLift = 0.0;
    double kpLift = 0.5;
    double kiLift = 0.0;
    double kdLift = 0.0;
    double kfLift = 0.0;
    double iLiftMax = 0.25;
    /*
        I still can't for the life of me figure out which way to do gamepad.
        Switched back to GM0 way
    */
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        try {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
        } catch (RobotCoreException e) {
        }

        //Driving Code
        if(gamepad1.right_trigger > 0.1){
            turningPower = 0.3 * gamepad1.right_trigger;
        }else if(gamepad1.left_trigger > 0.1){
            turningPower = -0.3 * gamepad1.left_trigger;
        }else{
            turningPower = 0.75 * gamepad1.right_stick_x;
        }

        double y = 0.75 * currentGamepad1.left_stick_y;
        double x = 0.75 * currentGamepad1.left_stick_x;
        double rx = turningPower;

        robot.frontLeft.setPower(y + x + rx);
        robot.frontRight.setPower(y - x - rx);
        robot.backLeft.setPower(y - x + rx);
        robot.backRight.setPower(y + x - rx);

        //Claw Code
        switch (currentClawPosition){
            case Open:
                robot.claw.setPosition(robot.CLAW_OPEN);
                if(currentGamepad1.b && !previousGamepad1.b){
                    currentClawPosition = ClawPosition.Closed;
                }
                break;
            case Closed:
                robot.claw.setPosition(robot.CLAW_CLOSED);
                if(currentGamepad1.b && !previousGamepad1.b){
                    currentClawPosition = ClawPosition.Open;
                }
                break;
        }

        if(currentGamepad1.x && !previousGamepad1.x){
            lowLiftPosition = robot.lift.getCurrentPosition();
            groundLiftPosition = lowLiftPosition - 75;
            midLiftPosition = lowLiftPosition + 350;
            highLiftPosition = lowLiftPosition + 600;
            stackLiftPosition = lowLiftPosition - 25;
        }

        if(gamepad1.dpad_left){
            currentLiftHeight = LiftHeight.GROUND;
        }else if(gamepad1.dpad_down){
            currentLiftHeight = LiftHeight.LOW;
        }else if(gamepad1.dpad_right){
            currentLiftHeight = LiftHeight.MID;
        }else if(gamepad1.dpad_up){
            currentLiftHeight = LiftHeight.HIGH;
        }else if(currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
            if(currentLiftHeight == LiftHeight.STACK){
                stackLiftPosition += 50;
            }else{
                currentLiftHeight = LiftHeight.STACK;
            }
        }else if(currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
            if(currentLiftHeight == LiftHeight.STACK){
                stackLiftPosition -= 50;
            }else {
                currentLiftHeight = LiftHeight.STACK;
            }
        }

        if(currentGamepad1.y && !previousGamepad1.y){
            switch(currentLiftHeight){
                case GROUND:
                    groundLiftPosition += 10;
                    break;
                case LOW:
                    lowLiftPosition += 10;
                    break;
                case MID:
                    midLiftPosition += 10;
                    break;
                case HIGH:
                    highLiftPosition += 10;
                    break;
            }
        }

        if(currentGamepad1.a && !previousGamepad1.a){
            switch (currentLiftHeight){
                case GROUND:
                    groundLiftPosition -= 10;
                    break;
                case LOW:
                    lowLiftPosition -= 10;
                    break;
                case MID:
                    midLiftPosition -= 10;
                    break;
                case HIGH:
                    highLiftPosition -= 10;
                    break;
            }
        }
        //Should I do this as a switch statement?
        switch(currentLiftHeight){
            case GROUND:
                liftTarget = groundLiftPosition;
                break;
            case LOW:
                liftTarget = lowLiftPosition;
                break;
            case MID:
                liftTarget = midLiftPosition;
                break;
            case HIGH:
                liftTarget = highLiftPosition;
                break;
            case STACK:
                liftTarget = stackLiftPosition;
                break;
        }
        robot.lift.setPower(liftPower(liftTarget));
        teleOpTelemetry();
    }

    public double liftPower(int target){
        liftError = target - robot.lift.getCurrentPosition();
        dLift = (liftError - pLift) / LiftPIDTimer.seconds();
        iLift = iLift + (liftError * LiftPIDTimer.seconds());
        pLift = liftError;
        liftTotal = ((pLift * kpLift) + (iLift * kiLift) + (dLift * kdLift))/100;
        LiftPIDTimer.reset();

        if(currentLiftHeight != lastLiftHeight){
            iLift = 0;
        }
        if(iLift > iLiftMax){
            iLift = iLiftMax;
        }else if(iLift < -iLiftMax){
            iLift = -iLiftMax;
        }
        lastLiftHeight = currentLiftHeight;

        return liftTotal;
    }

    public void teleOpTelemetry(){
        telemetry.addData("Front Left Position", robot.frontLeft.getCurrentPosition());
        telemetry.addData("Front Left Power", robot.frontLeft.getPower());
        telemetry.addData("Front Right Position", robot.frontRight.getCurrentPosition());
        telemetry.addData("Front Right Power", robot.frontRight.getPower());
        telemetry.addData("Back Left Position", robot.backLeft.getCurrentPosition());
        telemetry.addData("Back Left Power", robot.backLeft.getPower());
        telemetry.addData("Back Right Position", robot.backRight.getCurrentPosition());
        telemetry.addData("Back Right Position", robot.backRight.getPower());
        telemetry.addData("Claw State", currentClawPosition);
        telemetry.addData("Lift Level", currentLiftHeight);
        telemetry.addData("Lift Position", robot.lift.getCurrentPosition());
        telemetry.addData("Lift Target", liftTarget);
        telemetry.addData("Ground Position", groundLiftPosition);
        telemetry.addData("Low Position", lowLiftPosition);
        telemetry.addData("Mid Position", midLiftPosition);
        telemetry.addData("High Position", highLiftPosition);
        telemetry.addData("Lift PID Total", liftTotal);
        telemetry.addData("P Lift", pLift);
        telemetry.addData("I Lift", iLift);
        telemetry.addData("D Lift", dLift);
        telemetry.addData("F Lift", fLift);
        telemetry.update();
    }
}
