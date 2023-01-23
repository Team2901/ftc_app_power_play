package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous.SmolBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.SmolBot11588Hardware;

public class SmolBot11588BaseAutonomous extends LinearOpMode {
    SmolBot11588Hardware robot = new SmolBot11588Hardware();

    //Movement Setup
    double startAngle = 0.0;
    double targetAngle = -1.0;
    double turnError = 0.0;

    //Everything to do with the lift pid
    public enum LiftHeight{
        GROUND,
        STACK,
        LOW,
        MID,
        HIGH
    }
    ElapsedTime liftPIDTimer = new ElapsedTime();
    int liftTarget = 50;
    int lastLiftTarget = liftTarget;
    int liftError = 0;
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

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void moveXY(double y, double x){

    }

    public void moveLift(LiftHeight height){

    }

    public void turnByAngle(double turnAngle){
        startAngle = robot.getAngle();
        targetAngle = AngleUnit.normalizeDegrees(startAngle + turnAngle);
        turnError = targetAngle - robot.getAngle();

    }

    public void autoTelemetry(){

    }
}
