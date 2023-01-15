package org.firstinspires.ftc.teamcode.PowerPlay2901.Autonomous;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.NewProgrammers.Y2023.Mecanum.ObjectDetectionPipeline;
import org.firstinspires.ftc.teamcode.PowerPlay2901.Hardware.EarlyDiffyHardware;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.Utility.CountDownTimer;

import java.util.concurrent.TimeUnit;

@TeleOp(name="Test T265 2", group="Iterative Opmode")
public class IntelRealsense2 extends OpMode {
    // We treat this like a singleton because there should only ever be one object per camera
    private static T265Camera slamra = null;
    public double initTheta;

    //Don't need to use this unless camera is not facing a cardinal direction in relation to robot
    final double angleOffset = 0;

    T265Camera.CameraUpdate up;

    CountDownTimer countDownTimer = new CountDownTimer(ElapsedTime.Resolution.SECONDS);

    //Grants access to the T265 coordinates and current rotational angle
    Translation2d translation;
    Rotation2d rotation;

    public ObjectDetectionPipeline pipeline;

    ElapsedTime time = new ElapsedTime();
    ElapsedTime impTime = new ElapsedTime();

    private ElapsedTime runtime = new ElapsedTime();

    private ElapsedTime matchTimer = new ElapsedTime();

    EarlyDiffyHardware robot = new EarlyDiffyHardware();

    //Constants for odometry
    public static final double encoderTicksPerWheelRev = 8192; //REV encoders
    public static final double wheelCircumference = (2.83465 * Math.PI); //72mm diameter wheels
    public static final double leftRightDistance = 8.375; //Distance between left and right odometry wheels
    public static final double midpointBackDistance = 7.25;
    public static final double inchPerTick = wheelCircumference / encoderTicksPerWheelRev;
    public static final double wheelCircumferenceBack = (2 * Math.PI);
    public static final double backInchPerTick = wheelCircumferenceBack / encoderTicksPerWheelRev;

    //Variables for odometry
    public int currentLeftPosition = 0;
    public int currentRightPosition = 0;
    public int currentBackPosition = 0;
    private int oldLeftPosition = 0;
    private int oldRightPosition = 0;
    private int oldBackPosition = 0;

    double currentTime;
    double previousTime;
    double previousError;
    double currentError;

    double p;
    double i;
    double d;

    //PID constants for moving
    double kp = 0.07;
    double ki = 0;
    double kd = 0;
    final double max_i = 1;

    double addX = 0;
    double addY = 0;

    //positionX and positionY are the target coordinates. (set them to make the robot move)
    double positionX, positionY;
    double targetAngle = 0;

    double cameraXOffset = 6; //lies 6 inches in front of middle
    double cameraYOffset = -3.5; //lies 3.5 inches up from the middle

    double angleToTarget = 0;

    double turnByAngle;

    //Keeps track of current actions of robot for purposes of switching AutoState
    boolean isMoving = true;
    boolean isTurning = false;

    double liftPower = 0;


    public enum AutoState {
        MOVE_FORWARD,
        REVERSE,
        TURN_45,
        LIFT_SLIDES,
        INCH_FORWARD,
        EXTEND_CLAW,
        RETRACT_SLIDES,
        EXTEND_SLIDES,
        INCH_BACK,
        TURN_452,
        MOVE_BACK,
        RETRACT_CLAW,
        RETRACT_SLIDES_2,
        MOVE_FORWARD2,
        TURN_N45,
        PARK,
        FINAL_TURN
    }
    AutoState autoState;

    double outputLeft;
    double outputRight;
    double speedMod = 3;

    double turnPower;
    double leftTurnPower;
    double rightTurnPower;

    ImprovedGamepad improvedGamepad;
    ImprovedGamepad improvedGamepad2;

    double turnAngle;

    //Turning PID constants
    double turnKp = -0.44;
    double turnKi = 0;
    double turnKd = 0;

    //Adjusted position for camera's offset from robot's center
    double offsetX;
    double offsetY;
    //Position based on the average readings of the T265 and the odometry wheels
    double averagedX;
    double averagedY;

    double targetAngle2 = 0;

    int liftTarget = 65;
    double feedForward = .3;


    //XYhVector stores the odometry's x, y, and angle values (accessed with pos.x, pos.y, or pos.h)
    public XYhVector START_POS = new XYhVector(-cameraXOffset, -cameraYOffset, 0);
    public XYhVector pos = new XYhVector(START_POS);

    //Ignore for now, use later for parking locations using camera
    public int parking = -1;

    boolean firstRound = true;
    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, true);
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
        }
        initTheta = slamra.getLastReceivedCameraUpdate().pose.getRotation().getDegrees() + angleOffset;
        currentTime = time.time(TimeUnit.MILLISECONDS);
        previousTime = currentTime;
        previousError = currentError;
        improvedGamepad = new ImprovedGamepad(gamepad1, impTime, "GP");
        improvedGamepad2 = new ImprovedGamepad(gamepad2, impTime, "GP2");


        //Sets the target position to offsets to prevent initial movement upon starting
        positionX = -cameraXOffset;
        positionY = -cameraYOffset;
        autoState = AutoState.MOVE_FORWARD;

        //julia circle vision
        ElapsedTime stopwatch = new ElapsedTime();
        double seconds = stopwatch.seconds();
        pipeline = new ObjectDetectionPipeline(this.telemetry);

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        slamra.start();
    }

    @Override
    public void loop() {
        telemetry.addData("parking", parking);

        up = slamra.getLastReceivedCameraUpdate();
        // We divide by 0.0254 to convert meters to inches
        translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        rotation = up.pose.getRotation();
        improvedGamepad.update();
        improvedGamepad2.update();
        if (up == null) return;


//        arrowX = Math.cos(28.17859 + rotation.getRadians()) * robotRadius;
//        arrowY = Math.sin(28.17859 + rotation.getRadians()) * robotRadius;
        double x1 = translation.getX(), y1 = translation.getY();
        double fieldTheta = rotation.getDegrees() + angleOffset - initTheta;
        offsetX = (x1 - (cameraXOffset * Math.cos(rotation.getRadians())));
        offsetY = (y1 - (cameraYOffset * Math.cos(rotation.getRadians())));
        double adjustX = ((offsetX * Math.cos(angleOffset)) - (offsetY * Math.sin(angleOffset)));
        double adjustY = ((offsetX * Math.sin(angleOffset)) + (offsetY * Math.cos(angleOffset)));

        averagedX = ((offsetX*1) + (pos.x*0));
        averagedY = ((offsetY*1) + (pos.y*0));

        //Changes target Position
        if (improvedGamepad.dpad_right.isInitialPress()) {
            move(24, 0);
        } else if (improvedGamepad.dpad_left.isInitialPress()) {
            move(-24, 0);
        } else if (improvedGamepad.dpad_up.isInitialPress()) {
            move(0, 72);
        } else if (improvedGamepad.dpad_down.isInitialPress()) {
            move(0, -72);
        }
        //Changes target angle
        if(improvedGamepad.a.isInitialPress()) {
            liftTarget = 415;
        } else if(improvedGamepad.b.getValue()){
            targetAngle = 90;
        } else if(improvedGamepad.y.getValue()){
            targetAngle = 180;
        } else if(improvedGamepad.x.getValue()){
            targetAngle = -90;
        }



        if(firstRound) {
            move(0, 72);
            firstRound = false;
        } else if (autoState == AutoState.MOVE_FORWARD) {
            if (!isTurning && !isMoving) {
                autoState = AutoState.REVERSE;
                move(0, -24);
            }
        }else if(autoState == AutoState.REVERSE){
            if(!isTurning && !isMoving) {
                autoState = AutoState.PARK;
                if(parking == 0||parking == -1) {
                    moveTo(-26, 48);
                }else if(parking == 1){
                    moveTo(0, 48);
                } else if(parking == 2){
                    moveTo(26, 48);
                }

            }
        } /*else if(autoState == AutoState.REVERSE){
            if(!isTurning && !isMoving) {
                autoState = AutoState.TURN_45;
                isTurning = true;
                targetAngle = 45;

            }
        }else if(autoState == AutoState.TURN_452) {
            if (!isTurning && !isMoving) {
                autoState = AutoState.LIFT_SLIDES;
                liftTarget = 815;
                isMoving = true;
            }
        }else if(autoState == AutoState.LIFT_SLIDES){
            if (!isTurning && !isMoving) {
                autoState = AutoState.MOVE_BACK;
                move(26, 0);
            }
        }else if(autoState == AutoState.TURN_45){
            if(!isTurning && !isMoving) {
                autoState = AutoState.TURN_452;
                isTurning = true;
                targetAngle = 90;

            }
        }else if(autoState == AutoState.MOVE_BACK){
            if(!isTurning && !isMoving) {
                autoState = AutoState.MOVE_FORWARD2;
                move(-24, 0);
            }
        } *//*else if(autoState == AutoState.MOVE_FORWARD2){
            if(!isTurning && !isMoving) {
                autoState = AutoState.PARK;
                isTurning = true;
                if(parking == 1) {
                    moveTo(48, -24);
                }else if(parking == 2){
                    moveTo(48, 0);
                } else if(parking == 3){
                    moveTo(48, 24);
                }

            }
        } else if(autoState == AutoState.PARK){
            if(!isTurning && !isMoving) {
                autoState = AutoState.FINAL_TURN;
                isTurning = true;
                targetAngle = 45;
            }
        }*/

        //updates odometry
        odometry();

        //Movement PID code
        if (!isTurning && isMoving && (Math.abs(((positionX) - (averagedX))) > 0 || Math.abs((positionY) - (averagedY)) > 0)) {

            double dx = ((positionX) - (averagedX));
            double dy = ((positionY) - (averagedY));
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
            telemetry.addData("dx", dx);
            telemetry.addData("dy", dy);


        } else {
            outputLeft = 0;
        }

        outputLeft *= -1;
        //pos.h change (from rotation.toDegrees)



        telemetry.addData("position x", positionX);
        telemetry.addData("position y", positionY);
        telemetry.addData("camera x", averagedX);
        telemetry.addData("camera y", averagedY);

        //Creates dead zone radius larger than target

        turnPower = -AngleUnit.normalizeDegrees(targetAngle - Math.toDegrees(robot.getAngle())) / 100;

        //pos.h change
        outputRight = outputLeft;
        outputRight -= (turnPower*Math.cos(Math.toRadians(angleToTarget)));
        outputLeft += (turnPower*Math.cos(Math.toRadians(angleToTarget)));
        leftTurnPower = leftPodTurn(angleToTarget+(45*turnPower*Math.sin(Math.toRadians(angleToTarget))));
        rightTurnPower = rightPodTurn(angleToTarget-(45*turnPower*Math.sin(Math.toRadians(angleToTarget))));

        if (isMoving && (Math.abs(((positionX) - (averagedX))) < 0.5 && Math.abs((positionY) - (averagedY)) < 0.5)) {
            outputLeft = 0;
            outputRight = 0;
            leftTurnPower = 0;
            rightTurnPower = 0;
            isMoving = false;
        }
        telemetry.addData("leftTurnPower", leftTurnPower);
        telemetry.addData("rightTurnPower", rightTurnPower);
        telemetry.addData("outputLeft", outputLeft);
        telemetry.addData("outputRight", outputRight);
        telemetry.addData("Distance to Target x", (positionX - averagedX));
        telemetry.addData("Distance to Target y", (positionY - averagedY));

        /*if(autoState == AutoState.TURN_45 || autoState == AutoState.TURN_452 || autoState == AutoState.TURN_N45|| autoState == AutoState.FINAL_TURN) {
            leftTurnPower = leftPodTurn(0);
            rightTurnPower = rightPodTurn(0);
            outputLeft = turnToAngle(targetAngle);
            outputRight = -outputLeft;
            if (Math.abs(outputLeft) < 0.01) {
                isTurning = false;
            }
        }*/

        //runLift(liftTarget, false);

        robot.leftOne.setVelocity((outputLeft/speedMod+leftTurnPower)*2500);
        robot.leftTwo.setVelocity((outputLeft/speedMod-leftTurnPower)*2500);
        robot.rightOne.setVelocity((outputRight/speedMod+rightTurnPower)*2500);
        robot.rightTwo.setVelocity((outputRight/speedMod-rightTurnPower)*2500);

        if(improvedGamepad2.dpad_up.isInitialPress()){
            turnKp += 0.01;
        } else if(improvedGamepad2.dpad_down.isInitialPress()){
            turnKp -= 0.01;
        } else if(improvedGamepad2.dpad_left.isInitialPress()){
            turnKi -= 0.01;
        } else if(improvedGamepad2.dpad_right.isInitialPress()){
            turnKi += 0.01;
        } else if(improvedGamepad2.y.isInitialPress()){
            turnKd += 0.01;
        } else if(improvedGamepad2.a.isInitialPress()){
            turnKd -= 0.01;
        }

        telemetry.addData("isturning", isTurning);
        telemetry.addData("isMoving", isMoving);
        telemetry.addData("Auto State", autoState);
//        telemetry.addData("output", outputLeft);
//        telemetry.addData("output right", outputRight);
//        telemetry.addData("x1", String.format("%.2f", x1));
//        telemetry.addData("y1", String.format("%.2f", y1));
////        telemetry.addData("Adjusted Rotation", String.format("%.2f", fieldTheta) + "°");
////        telemetry.addData("Adjusted x", String.format("%.2f", adjustX) + " inches");
////        telemetry.addData("Adjusted y", String.format("%.2f", adjustY) + " inches");
//        telemetry.addData("Translation", translation);
//        telemetry.addData("addX", addX);
//        telemetry.addData("addY", addY);
////        telemetry.addData("Slide Position", robot.liftOne.getCurrentPosition());
////        telemetry.addData("Slide Target Position", robot.liftOne.getTargetPosition());
//        telemetry.addData("kp", kp);
//        telemetry.addData("kd", kd);
//        telemetry.addData("ki", ki);
//        telemetry.addData("turnkp", turnKp);
//        telemetry.addData("turnkd", turnKd);
//        telemetry.addData("turnki", turnKi);
//        telemetry.addData("Angle to Target", angleToTarget + "°");
//        telemetry.addData("Raw Rotation", rotation);
//        telemetry.addData("Offsetted X", offsetX);
//        telemetry.addData("Offsetted Y", offsetY);
//        telemetry.addData("odo x", pos.x);
//        telemetry.addData("odo y", pos.y);
//        telemetry.addData("odo h", pos.h);
    }

    @Override
    public void stop() {
        slamra.stop();
    }

    //enter (x, y) coordinates to move robot by
    public void move(double x, double y) {
        positionX = x + (averagedX);
        positionY = y + (averagedY);
        isMoving = true;
    }

    //enter (x, y) coordinates to move robot to
    public void moveTo(double x, double y) {
        positionX = x;
        positionY = y;
        isMoving = true;
    }

    //Left Pod PID
    private ElapsedTime runtimePodLeft = new ElapsedTime();
    double leftPodAngle = 0;
    double pAngleLeft = 0;
    double iAngleLeft = 0;
    double dAngleLeft = 0;

    double kpPod = 1.2;
    double kiPod = 0;
    double kdPod = 0;

    public double leftPodTurn(double angle) {
        leftPodAngle = (robot.leftOne.getCurrentPosition() - robot.leftTwo.getCurrentPosition()) / 8.95;
        double error = AngleUnit.normalizeDegrees(angle - leftPodAngle);
        if (!improvedGamepad.start.getValue() && (error >= 90 || error <= -90)) {
            error = AngleUnit.normalizeDegrees(error - 180);
            outputLeft = -outputLeft;
        }
        double secs = runtimePodLeft.seconds();
        runtime.reset();
        dAngleLeft = (error - pAngleLeft) / secs;
        iAngleLeft = iAngleLeft + (error * secs);
        pAngleLeft = error;
        double total = (kpPod * pAngleLeft + kiPod * iAngleLeft + kdPod * dAngleLeft) / 100;
        if (total > 1) {
            iAngleLeft = 0;
            total = 1;
        }
        if (total < -1) {
            iAngleLeft = 0;
            total = -1;
        }
        return total;
    }

    //Right Pod PID
    private ElapsedTime runtimePodRight = new ElapsedTime();
    double rightPodAngle = 0;
    double pAngleRight = 0;
    double iAngleRight = 0;
    double dAngleRight = 0;

    public double rightPodTurn(double angle) {
        rightPodAngle = (robot.rightOne.getCurrentPosition() - robot.rightTwo.getCurrentPosition()) / 8.95;
        double error = AngleUnit.normalizeDegrees(angle - rightPodAngle);
        if (!improvedGamepad.start.getValue() && (error >= 90 || error <= -90)) {
            error = AngleUnit.normalizeDegrees(error - 180);
            outputRight = -outputRight;
        }
        double secs = runtimePodRight.seconds();
        runtime.reset();
        dAngleRight = (error - pAngleRight) / secs;
        iAngleRight = iAngleRight + (error * secs);
        pAngleRight = error;
        double total = (kpPod * pAngleRight + kiPod * iAngleRight + kdPod * dAngleRight) / 100;
        if (total > 1) {
            iAngleRight = 0;
            total = 1;
        }
        if (total < -1) {
            iAngleRight = 0;
            total = -1;
        }
        return total;
    }
    //Enter angle for robot to turn by, robot oriented
    public double turnToAngle(double turnAngle){
        double targetAngle = turnAngle;
        ElapsedTime runtime = new ElapsedTime();
        double p = 0;
        double i = 0;
        double d = 0;
        double error = turnAngle;

        while(!(error < 1 && error > -1)){
            error = AngleUnit.normalizeDegrees(targetAngle - Math.toDegrees(robot.getAngle()));
            double secs = runtime.seconds();
            runtime.reset();
            d = (error - p) / secs;
            i = i + (error * secs);
            p = error;
            double total = (turnKp* p + turnKi* i + turnKd* d)/100;
            if(total > 1){
                i = 0;
                total = 1;
            }
            if(total < -1){
                i = 0;
                total = -1;
            }
            return total;
        }

        return 0;
    }

    //Gets coordinates of robot using 3 dead wheels with encoders
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
        pos.x -= dx * Math.sin(theta) + dy * Math.cos(theta);
        pos.h += dtheta;
    }



    public void runLift(int target, boolean drop){
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        double scaleFactor = 12/result;

        if(drop){
            liftPower = liftPower(target - 65);
            feedForward = 0;
            liftI = 0;
        } else {
            liftPower = liftPower(target);
            feedForward = .3;
        }

        robot.liftOne.setPower((liftPower - feedForward) * scaleFactor);
        robot.liftTwo.setPower((liftPower - feedForward) * scaleFactor);
    }

    double klp = 0.7;
    double kli = 0.0005;
    double kld = 0.015;

    public ElapsedTime runtimeLift = new ElapsedTime();
    double liftPosition = 0;
    double liftP = 0;
    double liftI = 0;
    double liftD = 0;

    public double liftPower(int target){
        int error = robot.liftOne.getCurrentPosition() - target;
        telemetry.addData("error", error);
        double secs = runtimeLift.seconds();
        runtime.reset();
        liftD = (error - liftP) / secs;
        liftI = liftI + (error * secs);
        liftP = error;
        double total = (klp* liftP + kli* liftI + kld* liftD)/100;
        if(total > .65){
            liftI = 0;
            total = .65;
        }
        if(total < -1){
            liftI = 0;
            total = -1;
        }
        if(Math.abs(error) < 2){
            isMoving = false;
        }
        return total;
    }

}
