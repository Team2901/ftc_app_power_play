package org.firstinspires.ftc.teamcode.PowerPlay2901.Autonomous;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PowerPlay2901.Hardware.EarlyDiffyHardware;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.Utility.CountDownTimer;

import java.util.concurrent.TimeUnit;

@TeleOp(name="Test T265", group="Iterative Opmode")
public class IntelRealsense extends OpMode {
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

    public enum AutoState {
        MOVE_FORWARD,
        TURN_45,

        //loop next part
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

        //Final State
        PARK
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
    double turnKp = 0.82;
    double turnKi = 0;
    double turnKd = 0;

    //Adjusted position for camera's offset from robot's center
    double offsetX;
    double offsetY;
    //Position based on the average readings of the T265 and the odometry wheels
    double averagedX;
    double averagedY;

    double targetAngle2 = 0;

    //XYhVector stores the odometry's x, y, and angle values (accessed with pos.x, pos.y, or pos.h)
    public XYhVector START_POS = new XYhVector(-cameraXOffset, -cameraYOffset, 0);
    public XYhVector pos = new XYhVector(START_POS);

    //Ignore for now, use later for parking locations using camera
    int parking = 2;

    boolean firstRound = true;

    @Override
    public void init() {
        robot.init(hardwareMap);
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

        averagedX = (offsetX + pos.x)/2;
        averagedY = (offsetY + pos.y)/2;

        //Changes target Position
        if (improvedGamepad.dpad_right.isInitialPress()) {
            move(24, 0);
        } else if (improvedGamepad.dpad_left.isInitialPress()) {
            move(-24, 0);
        } else if (improvedGamepad.dpad_up.isInitialPress()) {
            move(0, 24);
        } else if (improvedGamepad.dpad_down.isInitialPress()) {
            move(0, -24);
        }
        //Changes target angle
        if(improvedGamepad.a.getValue()) {
            targetAngle = 0;
        } else if(improvedGamepad.b.getValue()){
            targetAngle = 90;
        } else if(improvedGamepad.y.getValue()){
            targetAngle = 180;
        } else if(improvedGamepad.x.getValue()){
            targetAngle = -90;
        }



        if(firstRound){
            move(0, 48);
            firstRound = false;
            telemetry.addLine("First Round");
        }else if(autoState == AutoState.MOVE_FORWARD){
            if(!isTurning && !isMoving) {
                autoState = AutoState.TURN_45;
                move(24, 0);
            }
        }else if(autoState == AutoState.TURN_45){
            if(!isTurning && !isMoving) {
                autoState = AutoState.LIFT_SLIDES;
                move(-24, 0);
            }
        }

        /*//Auto States cycle
        switch(autoState){
            case MOVE_FORWARD:
                if(!isTurning && !isMoving && !firstRound) {
                    autoState = AutoState.TURN_45;
                    move(24, 0);
                }
                break;
            case TURN_45:
                break;
            case LIFT_SLIDES:

                break;
            case INCH_FORWARD:

                break;
            case EXTEND_CLAW:

                break;
            case RETRACT_SLIDES:

                break;
            case EXTEND_SLIDES:

                break;
            case INCH_BACK:

                break;
            case TURN_452:
                turnByAngle(45);
                if(!isTurning && !isMoving) {
                    autoState = AutoState.MOVE_BACK;
                }
                break;
            case MOVE_BACK:
                move(0, -24);
                if(!isTurning && !isMoving) {
                    autoState = AutoState.MOVE_FORWARD2;
                }
                break;
            case MOVE_FORWARD2:

                break;
            case TURN_N45:
                turnByAngle(-45);
                if(!isTurning && !isMoving){
                    autoState = AutoState.LIFT_SLIDES;
                } else if(!isTurning && !isMoving){
                    autoState = AutoState.PARK;
                }
                break;
            case PARK:
                if(parking == 0){
                    moveTo(24, -48);
                } else if(parking == 1){
                    moveTo(0, -48);
                } else if (parking == 2){
                    moveTo(-24, -48);
                }
                break;
        }*/



        //IGNORE
        /*if(autoState == AutoState.MOVE_FORWARD){
            moveTo(-50, 0);
        } else if(autoState == AutoState.TURN_45){
            moveTo(-50, 20);
        }
        if(autoState == AutoState.MOVE_FORWARD){
            moveTo(-48, 0);
        } else if(autoState == AutoState.TURN_45){
            turnByAngle(45);
        } else if(autoState == AutoState.LIFT_SLIDES){
            robot.liftOne.setTargetPosition(850);
            robot.liftOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftOne.setPower(0.9);
            countDownTimer.setTargetTime(10);
            if(countDownTimer.hasRemainingTime()) {
            } else {
                autoState = AutoState.RELEASE_CLAW;
            }
        } else if(autoState == AutoState.RELEASE_CLAW){
            robot.clawOne.setPosition(0);
            robot.clawTwo.setPosition(0.25);
            autoState = AutoState.RETRACT_SLIDES;
        } else if(autoState == AutoState.RETRACT_SLIDES){
            robot.liftOne.setTargetPosition(10);
            robot.liftOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftOne.setPower(-0.9);
            countDownTimer.setTargetTime(10);
            if(!countDownTimer.hasRemainingTime()) {
            } else {
                autoState = AutoState.TURN_452;
            }
        } else if(autoState == AutoState.TURN_452){
            turnByAngle(45);
        } else if(autoState == AutoState.MOVE_BACK){
            moveTo(-48, 24);
        } else if(autoState == AutoState.GRIP_CLAW){
            robot.clawOne.setPosition(.12);
            robot.clawTwo.setPosition(.18);
            autoState = AutoState.MOVE_FORWARD2;
        } else if(autoState == AutoState.MOVE_FORWARD2){
            moveTo(48, 0);
        } else if(autoState == AutoState.TURN_N45){
            turnByAngle(-45);
        }*/

        //Set lift heights
//        if(improvedGamepad.x.isInitialPress()){
//            robot.liftOne.setTargetPosition(850);
//            robot.liftOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.liftOne.setPower(0.9);
//
//        } else if(improvedGamepad.b.isInitialPress()){
//            robot.liftOne.setTargetPosition(10);
//            robot.liftOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.liftOne.setPower(-0.9);
//
//        }
//        if(robot.liftOne.getCurrentPosition() == robot.liftOne.getTargetPosition()){
//            robot.liftOne.setPower(0);
//        }
//
//        if(robot.liftOne.getPower() != 0){
//            robot.liftTwo.setPower(robot.liftOne.getPower());
//        } else {
//            robot.liftTwo.setPower(0);
//        }

        //updates odometry
        odometry();

        //Movement PID code
        if (!isTurning && isMoving && (Math.abs(((positionX) - (pos.x))) > 0 || Math.abs((positionY) - (pos.y)) > 0)) {

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
            telemetry.addData("dx", dx);
            telemetry.addData("dy", dy);


        } else {
            outputLeft = 0;
        }

        outputLeft *= -1;
        //pos.h change (from rotation.toDegrees)



        telemetry.addData("position x", positionX);

        //Creates dead zone radius larger than target
        if (isMoving && (Math.abs(((positionX) - (pos.x))) < 0.5 && Math.abs((positionY) - (pos.y)) < 0.5)) {
            outputLeft = 0;
            outputRight = 0;
            leftTurnPower = 0;
            rightTurnPower = 0;
            isMoving = false;
        }
        turnPower = -AngleUnit.normalizeDegrees(targetAngle - Math.toDegrees(robot.getAngle())) / 100;
        angleToTarget += Math.toDegrees(robot.getAngle());

        //pos.h change
        outputRight = outputLeft;
        outputRight -= (turnPower*Math.cos(Math.toRadians(angleToTarget)));
        outputLeft += (turnPower*Math.cos(Math.toRadians(angleToTarget)));
        leftTurnPower = leftPodTurn(angleToTarget+(45*turnPower*Math.sin(Math.toRadians(angleToTarget))));
        rightTurnPower = rightPodTurn(angleToTarget-(45*turnPower*Math.sin(Math.toRadians(angleToTarget))));

        robot.leftOne.setVelocity((outputLeft/speedMod+leftTurnPower)*2500);
        robot.leftTwo.setVelocity((outputLeft/speedMod-leftTurnPower)*2500);
        robot.rightOne.setVelocity((outputRight/speedMod+rightTurnPower)*2500);
        robot.rightTwo.setVelocity((outputRight/speedMod-rightTurnPower)*2500);

//        if(Math.abs(((positionX) - (translation.getX()))) < 1 || Math.abs((positionY) - (translation.getY())) < 1){
//            isMoving = false;
//        } else {isMoving = true;}

//        if(autoState == AutoState.MOVE_FORWARD && robot.leftOne.getVelocity() == 0 && isSecond){
//            autoState = AutoState.TURN_45;
//        } else if(autoState == AutoState.MOVE_BACK && robot.leftOne.getVelocity() == 0){
//            autoState = AutoState.GRIP_CLAW;
//        } else if(autoState == AutoState.MOVE_FORWARD2 && robot.leftOne.getVelocity() == 0) {
//            autoState = AutoState.TURN_N45;
//        }
//        if(isTurning){
//            if(Math.abs(leftPodAngle) > 5) {
//                leftTurnPower = leftPodTurn(0);
//                rightTurnPower = rightPodTurn(0);
//                robot.leftOne.setVelocity((leftTurnPower)*2500);
//                robot.leftTwo.setVelocity((-leftTurnPower)*2500);
//                robot.rightOne.setVelocity((rightTurnPower)*2500);
//                robot.rightTwo.setVelocity((-rightTurnPower)*2500);
//            }else {
//                if (targetAngle2 - robot.getAngle() != 0) {
//                    robot.leftOne.setVelocity(((targetAngle2 - robot.getAngle()) / 180) * 2500);
//                    robot.leftTwo.setVelocity(((targetAngle2 - robot.getAngle()) / 180) * 2500);
//                    robot.rightOne.setVelocity(((targetAngle2 - robot.getAngle()) / -180) * 2500);
//                    robot.rightTwo.setVelocity(((targetAngle2 - robot.getAngle()) / -180) * 2500);
//                }
//                if ((targetAngle2 - robot.getAngle()) < 5) {
//                    robot.leftOne.setVelocity(0);
//                    robot.leftTwo.setVelocity(0);
//                    robot.rightOne.setVelocity(0);
//                    robot.rightTwo.setVelocity(0);
//                    isTurning = false;
//                }
//            }
//        }

        telemetry.addData("isturning", isTurning);
        telemetry.addData("Auto State", autoState);
        telemetry.addData("output", outputLeft);
        telemetry.addData("output right", outputRight);
        telemetry.addData("x1", String.format("%.2f", x1));
        telemetry.addData("y1", String.format("%.2f", y1));
//        telemetry.addData("Adjusted Rotation", String.format("%.2f", fieldTheta) + "°");
//        telemetry.addData("Adjusted x", String.format("%.2f", adjustX) + " inches");
//        telemetry.addData("Adjusted y", String.format("%.2f", adjustY) + " inches");
        telemetry.addData("Translation", translation);
        telemetry.addData("addX", addX);
        telemetry.addData("addY", addY);
//        telemetry.addData("Slide Position", robot.liftOne.getCurrentPosition());
//        telemetry.addData("Slide Target Position", robot.liftOne.getTargetPosition());
        telemetry.addData("kp", kp);
        telemetry.addData("kd", kd);
        telemetry.addData("ki", ki);
        telemetry.addData("turnkp", turnKp);
        telemetry.addData("turnkd", turnKd);
        telemetry.addData("turnki", turnKi);
        telemetry.addData("Angle to Target", angleToTarget + "°");
        telemetry.addData("Raw Rotation", rotation);
        telemetry.addData("Offsetted X", offsetX);
        telemetry.addData("Offsetted Y", offsetY);
        telemetry.addData("odo x", pos.x);
        telemetry.addData("odo y", pos.y);
        telemetry.addData("odo h", pos.h);
    }

    @Override
    public void stop() {
        slamra.stop();
    }

    //enter (x, y) coordinates to move robot by
    public void move(double x, double y) {
        positionX = x + (pos.x);
        positionY = y + (pos.y);
        isMoving = true;
    }

    //enter (x, y) coordinates to move robot to
    public void moveTo(double x, double y) {
        positionX = x;
        positionY = y;
        isMoving = true;
    }

    //enter (x, y, angle), to change pod angle and move
    public void move(double x, double y, double angle) {
        positionX = x + translation.getX();
        positionY = y + translation.getY();
        //targetAngle = angle;
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

    //Enter angle for robot to turn to, field oriented
    public void turnToAngle(double turnAngle) {
        isTurning = true;
        this.turnAngle = turnAngle;
        isMoving = false;

//        if((autoState == AutoState.TURN_45 && robot.leftTwo.getPower() == 0) || (autoState == AutoState.TURN_N45 && robot.leftTwo.getPower() == 0)){
//            autoState = AutoState.LIFT_SLIDES;
//        } else if(autoState == AutoState.TURN_452 && robot.leftTwo.getPower() == 0){
//            autoState = AutoState.MOVE_BACK;
//        }
    }

    //Enter angle for robot to turn by, robot oriented
    public void turnByAngle(double turnAngle){
        isTurning = true;
        this.turnAngle = turnAngle + Math.toDegrees(robot.getAngle());
        isMoving = false;
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
        pos.x += dx * Math.sin(theta) + dy * Math.cos(theta);
        pos.h += dtheta;
    }
}
