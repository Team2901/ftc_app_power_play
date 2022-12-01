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

@Disabled
@TeleOp(name="Test T265", group="Iterative Opmode")
public class IntelRealsense extends OpMode
{
    // We treat this like a singleton because there should only ever be one object per camera
    private static T265Camera slamra = null;
    public double initTheta;

    final double angleOffset = 0;

    T265Camera.CameraUpdate up;

    CountDownTimer countDownTimer = new CountDownTimer(ElapsedTime.Resolution.SECONDS);
    Translation2d translation;
    Rotation2d rotation;

    ElapsedTime time = new ElapsedTime();
    ElapsedTime impTime = new ElapsedTime();

    private ElapsedTime runtime = new ElapsedTime();

    private ElapsedTime matchTimer = new ElapsedTime();

    EarlyDiffyHardware robot = new EarlyDiffyHardware();

    double currentTime;
    double previousTime;
    double previousError;
    double currentError;

    double p;
    double i;
    double d;

    double kp = 0.07;
    double ki = 0;
    double kd = 0;
    final double max_i = 1;

    double addX = 0;
    double addY = 0;

    double positionX, positionY;
    double targetAngle = 0;

    double cameraXOffset = 6; //lies 7 inches in front of middle
    double cameraYOffset = 0; //lies 3.75 inches up from the middle

    double angleToTarget = 0;

    double turnByAngle;
    double arrowX, arrowY;

    boolean isMoving = true;
    boolean isTurning = false;

    public enum AutoState {
        MOVE_FORWARD,
        TURN_45,

        //loop next part
        LIFT_SLIDES,
        RELEASE_CLAW,
        RETRACT_SLIDES,
        TURN_452,
        MOVE_BACK,
        GRIP_CLAW,
        MOVE_FORWARD2,
        TURN_N45
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

    double turnKp = 0.4;
    double turnKi = 0;
    double turnKd = 0.1;

    /*public XYhVector START_POS = new XYhVector(0, 0, getAngle());
    public XYhVector pos = new XYhVector(START_POS);*/

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
        translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        rotation = up.pose.getRotation();
        improvedGamepad.update();
        improvedGamepad2.update();
        final int robotRadius = 8; // inches

        if (up == null) return;

        // We divide by 0.0254 to convert meters to inches


//        arrowX = Math.cos(28.17859 + rotation.getRadians()) * robotRadius;
//        arrowY = Math.sin(28.17859 + rotation.getRadians()) * robotRadius;
        double x1 = translation.getX(), y1 = translation.getY();
        double fieldTheta = rotation.getDegrees() + angleOffset - initTheta;
        double offsetX = (x1 - (cameraXOffset*Math.cos(rotation.getRadians())));
        double offsetY = (y1 - (cameraYOffset*Math.cos(rotation.getRadians())));
        double adjustX = ((offsetX*Math.cos(angleOffset)) - (offsetY*Math.sin(angleOffset)));
        double adjustY = ((offsetX*Math.sin(angleOffset)) + (offsetY*Math.cos(angleOffset)));

        /*double averagedX = (offsetX + pos.x)/2;
        double averagedY = (offsetY + pos.y)/2;*/

        //Changes target Position
        if(improvedGamepad.dpad_right.isInitialPress()){
            move(48, 0);
//           kp +=0.01;
        } else if(improvedGamepad.dpad_left.isInitialPress()){
            move(-48, 0);
//            kp -=0.01;
        }
        if(improvedGamepad.dpad_up.isInitialPress()){
            move(0, 24);
//           kd += 0.01;
        } else if(improvedGamepad.dpad_down.isInitialPress()){
            move(0, -24);
//           kd -= 0.01;
        }
        if(improvedGamepad.left_bumper.isInitialPress()){
            turnToAngle(90);
        } else if(improvedGamepad.right_bumper.isInitialPress()){
            turnToAngle(-90);
        }


        switch(autoState){
            case MOVE_FORWARD:
                move(36, 0);
                if(!isTurning && !isMoving) {
                    autoState = AutoState.TURN_45;
                }
                break;
            case TURN_45:
                turnToAngle(45);
                if(!isTurning && !isMoving) {
                    autoState = AutoState.LIFT_SLIDES;
                }
                break;
            case LIFT_SLIDES:

                break;
            case RELEASE_CLAW:

                break;
            case RETRACT_SLIDES:

                break;
            case TURN_452:
                turnToAngle(90);
                if(!isTurning && !isMoving) {
                    autoState = AutoState.MOVE_BACK;
                }
                break;
            case MOVE_BACK:
                move(0, -24);
                if(!isTurning && !isMoving) {
                    autoState = AutoState.GRIP_CLAW;
                }
                break;
            case GRIP_CLAW:

                break;
            case MOVE_FORWARD2:

                break;
            case TURN_N45:

                break;
        }


//        if(autoState == AutoState.MOVE_FORWARD){
//            moveTo(-50, 0);
//        } else if(autoState == AutoState.TURN_45){
//            moveTo(-50, 20);
//        }
//        if(autoState == AutoState.MOVE_FORWARD){
//            moveTo(-48, 0);
//        } else if(autoState == AutoState.TURN_45){
//            turnByAngle(45);
//        } else if(autoState == AutoState.LIFT_SLIDES){
//            robot.liftOne.setTargetPosition(850);
//            robot.liftOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.liftOne.setPower(0.9);
//            countDownTimer.setTargetTime(10);
//            if(countDownTimer.hasRemainingTime()) {
//            } else {
//                autoState = AutoState.RELEASE_CLAW;
//            }
//        } else if(autoState == AutoState.RELEASE_CLAW){
//            robot.clawOne.setPosition(0);
//            robot.clawTwo.setPosition(0.25);
//            autoState = AutoState.RETRACT_SLIDES;
//        } else if(autoState == AutoState.RETRACT_SLIDES){
//            robot.liftOne.setTargetPosition(10);
//            robot.liftOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.liftOne.setPower(-0.9);
//            countDownTimer.setTargetTime(10);
//            if(!countDownTimer.hasRemainingTime()) {
//            } else {
//                autoState = AutoState.TURN_452;
//            }
//        } else if(autoState == AutoState.TURN_452){
//            turnByAngle(45);
//        } else if(autoState == AutoState.MOVE_BACK){
//            moveTo(-48, 24);
//        } else if(autoState == AutoState.GRIP_CLAW){
//            robot.clawOne.setPosition(.12);
//            robot.clawTwo.setPosition(.18);
//            autoState = AutoState.MOVE_FORWARD2;
//        } else if(autoState == AutoState.MOVE_FORWARD2){
//            moveTo(48, 0);
//        } else if(autoState == AutoState.TURN_N45){
//            turnByAngle(-45);
//        }

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

        /*odometry();*/ //reinstate this
        //Movement PID code
        if(isMoving && (Math.abs(((positionX) - (translation.getX()))) > 0 || Math.abs((positionY) - (translation.getY())) > 0)) {

            double dx = ((positionX) - (translation.getX()));
            double dy = ((positionY) - (translation.getY()));
            double angle = Math.atan(dy / dx);
            double hypotenuse = Math.sqrt((Math.pow(dx, 2) + Math.pow(dy, 2)));
            currentError = hypotenuse;
            angleToTarget = -Math.toDegrees(angle);

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
            isMoving = false;
        }

        outputLeft *= -1;
        angleToTarget += rotation.getDegrees();
        outputLeft *= 100.0;
        outputLeft = (int)outputLeft;
        outputLeft /= 100.0;

        turnPower = AngleUnit.normalizeDegrees(targetAngle - rotation.getDegrees())/500;
        outputRight = outputLeft;
        leftTurnPower = leftPodTurn(angleToTarget);
        rightTurnPower = rightPodTurn(angleToTarget);


        telemetry.addData("position x", positionX);
        telemetry.addData("translation x", translation.getX());
        if(isMoving && (Math.abs(((positionX) - (translation.getX()))) < 0.5 && Math.abs((positionY) - (translation.getY())) < 0.5)){
            outputLeft = 0;
            outputRight = 0;
            leftTurnPower = 0;
            rightTurnPower = 0;
        }

        //
        //
        //
        //

        //Angle turn code
        double startAngle = rotation.getDegrees();
        double targetAngle = turnAngle;
        ElapsedTime turnRuntime = new ElapsedTime();
        double turnP = 0;
        double turnI = 0;
        double turnD = 0;
        double turnError = turnAngle;
        if(improvedGamepad2.a.isInitialPress()){
            turnKp -= 0.01;
        } else if(improvedGamepad2.y.isInitialPress()){
            turnKp += 0.01;
        } else if(improvedGamepad2.x.isInitialPress()){
            turnKd -= 0.01;
        } else if(improvedGamepad2.b.isInitialPress()){
            turnKd += 0.01;
        }

        if(isTurning && !(turnError < 1.5 && turnError > -1.5)){
            turnError = (targetAngle - robot.getAngle());
            double turnSecs = turnRuntime.seconds();
            runtime.reset();
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Current Angle", robot.getAngle());
            telemetry.addData("Loop Time", turnSecs);
            turnD = (turnError - turnP) / turnSecs;
            turnI = turnI + (turnError * turnSecs);
            turnP = turnError;
            double turnTotal = (turnKp* turnP + turnKi* turnI + turnKd* turnD)/100;
            if(turnTotal > 1){
                turnI = 0;
                turnTotal = 1;
            }
            if(turnTotal < -1){
                turnI = 0;
                turnTotal = -1;
            }

            outputLeft = turnTotal;
            turnPower = 0;
            outputRight = -outputLeft;
            leftTurnPower = 0;
            rightTurnPower = 0;

        } else if(outputLeft < 0.01){
            positionX = translation.getX();
            positionY = translation.getY();
            robot.leftOne.setPower(0);
            robot.leftTwo.setPower(0);
            robot.rightOne.setPower(0);
            robot.rightTwo.setPower(0);
            isTurning = false;
        }
        //
        //
        //
        //



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

        telemetry.addData("Auto State", autoState);
        telemetry.addData("output", outputLeft);
        telemetry.addData("x1", String.format("%.2f", x1));
        telemetry.addData("y1", String.format("%.2f", y1));
        telemetry.addData("Adjusted Rotation", String.format("%.2f", fieldTheta) + "°");
        telemetry.addData("Adjusted x", String.format("%.2f", adjustX) + " inches");
        telemetry.addData("Adjusted y", String.format("%.2f", adjustY) + " inches");
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
        telemetry.addData("Angle to Target", angleToTarget + "°");
        telemetry.addData("Raw Rotation", rotation);
        telemetry.addData("Offsetted X", offsetX);
        telemetry.addData("Offsetted Y", offsetY);
//        telemetry.addData("offsetted x", robotCenterX);
//        telemetry.addData("offsetted y", robotCenterY);
    }

    @Override
    public void stop() {slamra.stop();}

//    public double getAngleDegrees(){
//        return slamra.getLastReceivedCameraUpdate().pose.getRotation().getDegrees();
//    }
//    public double getAngleRadians(){
//        return slamra.getLastReceivedCameraUpdate().pose.getRotation().getRadians();
//    }
    //enter (x, y) coordinates to move robot wheels to angle and position
    public void move(double x, double y) {
        positionX = x + translation.getX();
        positionY = y + translation.getY();
        isMoving = true;
    }

    public void moveTo(double x, double y) {
        positionX = x;
        positionY = y;
        isMoving = true;
    }

    public void move(double x, double y, double angle){
        positionX = x + translation.getX();
        positionY = y + translation.getY();
        targetAngle = angle;
        isMoving = true;
    }


    private ElapsedTime runtimePodLeft = new ElapsedTime();
    double leftPodAngle = 0;
    double pAngleLeft = 0;
    double iAngleLeft = 0;
    double dAngleLeft = 0;

    double kpPod = 1.2;
    double kiPod = 0;
    double kdPod = 0;

    public double leftPodTurn(double angle){
        leftPodAngle = (robot.leftOne.getCurrentPosition() - robot.leftTwo.getCurrentPosition())/8.95;
        double error = AngleUnit.normalizeDegrees(angle - leftPodAngle);
        if(!gamepad1.start && (error >= 90 || error <= -90)){
            error = AngleUnit.normalizeDegrees(error-180);
            outputLeft = -outputLeft;
        }
        double secs = runtimePodLeft.seconds();
        runtime.reset();
        dAngleLeft = (error - pAngleLeft) / secs;
        iAngleLeft = iAngleLeft + (error * secs);
        pAngleLeft = error;
        double total = (kpPod* pAngleLeft + kiPod* iAngleLeft + kdPod* dAngleLeft)/100;
        if(total > 1){
            iAngleLeft = 0;
            total = 1;
        }
        if(total < -1){
            iAngleLeft = 0;
            total = -1;
        }
        return total;
    }

    private ElapsedTime runtimePodRight = new ElapsedTime();
    double rightPodAngle = 0;
    double pAngleRight = 0;
    double iAngleRight = 0;
    double dAngleRight = 0;

    public double rightPodTurn(double angle){
        rightPodAngle = (robot.rightOne.getCurrentPosition() - robot.rightTwo.getCurrentPosition())/8.95;
        double error = AngleUnit.normalizeDegrees(angle - rightPodAngle);
        if(!gamepad1.start && (error >= 90 || error <= -90)){
            error = AngleUnit.normalizeDegrees(error-180);
            outputRight = -outputRight;
        }
        double secs = runtimePodRight.seconds();
        runtime.reset();
        dAngleRight = (error - pAngleRight) / secs;
        iAngleRight = iAngleRight + (error * secs);
        pAngleRight = error;
        double total = (kpPod* pAngleRight + kiPod* iAngleRight + kdPod* dAngleRight)/100;
        if(total > 1){
            iAngleRight = 0;
            total = 1;
        }
        if(total < -1){
            iAngleRight = 0;
            total = -1;
        }
        return total;
    }

    public void turnToAngle(double turnAngle) {
        isTurning = true;
        this.turnAngle = turnAngle;

//        if((autoState == AutoState.TURN_45 && robot.leftTwo.getPower() == 0) || (autoState == AutoState.TURN_N45 && robot.leftTwo.getPower() == 0)){
//            autoState = AutoState.LIFT_SLIDES;
//        } else if(autoState == AutoState.TURN_452 && robot.leftTwo.getPower() == 0){
//            autoState = AutoState.MOVE_BACK;
//        }
    }

    /*public void odometry(){
        oldRightPosition = currentRightPosition;
        oldLeftPosition = currentLeftPosition;
        oldBackPosition = currentBackPosition;

        currentRightPosition = rightOne.getCurrentPosition(); CHANGE NAME :))
        currentLeftPosition = leftOne.getCurrentPosition();
        currentBackPosition = rightTwo.getCurrentPosition();

        int dn1 = currentRightPosition - oldRightPosition;
        int dn2 = currentLeftPosition - oldLeftPosition;
        int dn3 = currentBackPosition - oldBackPosition;

        double dtheta = ((dn2 - dn1) / leftRightDistance) * inchPerTick;
        double dx = ((dn1 + dn2) / 2) * inchPerTick;
        double dy = (dn3 - (dn2 - dn1) * midpointBackDistance / leftRightDistance) * inchPerTick;

        double theta = pos.h + (dtheta / 2.0);
        pos.x += dx * Math.cos(theta) - dy * Math.sin(theta);
        pos.y += dx * Math.sin(theta) + dy * Math.cos(theta);
        pos.h += dtheta;
    }*/
}
