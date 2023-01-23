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
    public enum LiftHeight{GROUND, STACK, LOW, MEDIUM, HIGH}
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


    }

    public double liftPower(int target){
        return liftTotal;
    }

    public void teleOpTelemetry(){

    }
}
