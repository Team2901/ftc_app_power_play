package org.firstinspires.ftc.teamcode.PowerPlay2901.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PowerPlay2901.Hardware.EarlyDiffyHardware;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Tank Odometry Swerve")
public class TankDriveOdometry extends LinearOpMode {
    EarlyDiffyHardware robot = new EarlyDiffyHardware();
    public XYhVector START_POS = new XYhVector(0, 0, 0);
    public XYhVector pos = new XYhVector(START_POS);

    public int currentLeftPosition = 0;
    public int currentRightPosition = 0;
    public int currentBackPosition = 0;
    private int oldLeftPosition = 0;
    private int oldRightPosition = 0;
    private int oldBackPosition = 0;

    ElapsedTime time = new ElapsedTime();

    public static final double encoderTicksPerWheelRev = 8192; //REV encoders
    public static final double wheelCircumference = (2.83465 * Math.PI); //72mm diameter wheels
    public static final double leftRightDistance = (8.375); //Distance between left and right odometry wheels
    public static final double midpointBackDistance = 8;
    public static final double inchPerTick = wheelCircumference / encoderTicksPerWheelRev;
    public static final double wheelCircumferenceBack = (2 * Math.PI);
    public static final double backInchPerTick = wheelCircumferenceBack / encoderTicksPerWheelRev;
    double p;
    double i;
    double d;

    //PID constants for moving
    double kp = 0.07;
    double ki = 0;
    double kd = 0;
    final double max_i = 1;
    double positionX, positionY;
    double targetAngle = 0;
    double outputLeft;
    double outputRight;


    double currentTime;
    double previousTime;
    double previousError;
    double currentError;

    double angleToTarget = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        robot.init(hardwareMap);
        currentTime = time.time(TimeUnit.MILLISECONDS);
        previousTime = currentTime;
        previousError = currentError;
        move(12, 24);
    }
    public void move(int x, int y){
        odometry();
        while(Math.abs(Math.atan2(y, x)-Math.toDegrees(pos.h)) > 5){
            double dx = ((positionX) - (pos.x));
            double dy = ((positionY) - (pos.y));
            double angle = Math.atan(dy / dx);
            angleToTarget = -Math.toDegrees(angle);

            //Adjusts angle for pods
            if (dy > 0) {
                angleToTarget += 180;
            } else if (dx < 0 && dy < 0) {
                angleToTarget += 360;
            }
            if (dx > 0 && dy < 0) {
                angleToTarget -= 180;
            }
            if (dx < 0 && dy > 0) {
                angleToTarget += 180;
            }
            if (angleToTarget > 180) {
                angleToTarget -= 360;
            }

            //If pods are moving perpendicular change this value by +90 or -90
            //If pods are moving opposite direction change this value by +180/-180
            angleToTarget -= 90;

            currentTime = time.time(TimeUnit.MILLISECONDS);

            p = kp * angleToTarget/36;
            i += ki * (angleToTarget/36 * (currentTime - previousTime));

            if (i > max_i) {
                i = max_i;
            } else if (i < -max_i) {
                i = -max_i;
            }

            d = kd * (angleToTarget/36 - previousError) / (currentTime - previousTime);

            outputLeft = p + i + d;

            previousError = angleToTarget/36;
            previousTime = currentTime;
            if (outputLeft > 1) {
                outputLeft = 1;
            } else if (outputLeft < -1) {
                outputLeft = -1;
            }
            outputRight = -outputLeft;
            robot.leftOne.setVelocity((outputLeft/3)*2500);
            robot.leftTwo.setVelocity((outputLeft/3)*2500);
            robot.rightOne.setVelocity((outputRight/3)*2500);
            robot.rightTwo.setVelocity((outputRight/3)*2500);
        }
        while(Math.abs(x-pos.x) > 1 || Math.abs(y-pos.y) > 1){
            double dx = ((positionX) - (pos.x));
            double dy = ((positionY) - (pos.y));
            double angle = Math.atan(dy / dx);
            currentError = Math.sqrt((Math.pow(dx, 2) + Math.pow(dy, 2)));
            angleToTarget = -Math.toDegrees(angle);

            //Adjusts angle for pods
            if (dy > 0) {
                angleToTarget += 180;
            } else if (dx < 0 && dy < 0) {
                angleToTarget += 360;
            }
            if (dx > 0 && dy < 0) {
                angleToTarget -= 180;
            }
            if (dx < 0 && dy > 0) {
                angleToTarget += 180;
            }
            if (angleToTarget > 180) {
                angleToTarget -= 360;
            }

            //If pods are moving perpendicular change this value by +90 or -90
            //If pods are moving opposite direction change this value by +180/-180
            angleToTarget -= 90;

            currentTime = time.time(TimeUnit.MILLISECONDS);

            p = kp * currentError;
            i += ki * (currentError * (currentTime - previousTime));

            if (i > max_i) {
                i = max_i;
            } else if (i < -max_i) {
                i = -max_i;
            }

            d = kd * (currentError - previousError) / (currentTime - previousTime);

            outputLeft = p + i + d;

            previousError = currentError;
            previousTime = currentTime;
            if (outputLeft > 1) {
                outputLeft = 1;
            } else if (outputLeft < -1) {
                outputLeft = -1;
            }
            outputRight = outputLeft;
            robot.leftOne.setVelocity((outputLeft/3)*2500);
            robot.leftTwo.setVelocity((outputLeft/3)*2500);
            robot.rightOne.setVelocity((outputRight/3)*2500);
            robot.rightTwo.setVelocity((outputRight/3)*2500);

        }

    }
    public void odometry() {
        oldRightPosition = currentRightPosition;
        oldLeftPosition = currentLeftPosition;
        oldBackPosition = currentBackPosition;

        currentRightPosition = robot.odoRight.getCurrentPosition();
        currentLeftPosition = robot.odoLeft.getCurrentPosition();
        currentBackPosition = robot.liftTwo.getCurrentPosition();

        int dn1 = currentRightPosition - oldRightPosition;
        int dn2 = currentLeftPosition - oldLeftPosition;
        int dn3 = currentBackPosition - oldBackPosition;

        dn2 = -dn2;

        double dtheta = ((dn2 - dn1) / leftRightDistance) * inchPerTick;
        double dx = ((dn1 + dn2) / 2) * inchPerTick;
        double dy = ((backInchPerTick * dn3) - (inchPerTick * (dn2 - dn1) * midpointBackDistance) / leftRightDistance);

        double theta = pos.h + (dtheta / 2.0);
        pos.y += dx * Math.cos(theta) - dy * Math.sin(theta);
        pos.x += dx * Math.sin(theta) + dy * Math.cos(theta);
        pos.h += dtheta;
    }
}
