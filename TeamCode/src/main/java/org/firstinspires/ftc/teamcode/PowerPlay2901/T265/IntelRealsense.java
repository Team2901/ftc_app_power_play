package org.firstinspires.ftc.teamcode.PowerPlay2901.T265;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PowerPlay2901.EarlyDiffy.EarlyDiffyHardware;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

import java.util.concurrent.TimeUnit;


@TeleOp(name="Test T265", group="Iterative Opmode")
public class IntelRealsense extends OpMode
{
    // We treat this like a singleton because there should only ever be one object per camera
    private static T265Camera slamra = null;
    public double initTheta;

    final double angleOffset = 0;

    T265Camera.CameraUpdate up;

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

    double kp = 0.03;
    double ki = 0;
    double kd = 0;
    final double max_i = 1;

    double addX = 0;
    double addY = 0;

    double positionX, positionY;
    double targetAngle = 0;

    double cameraXOffset = 7; //lies 7 inches in front of middle
    double cameraYOffset = 3.75; //lies 3.75 inches up from the middle

    double angleToTarget = 0;

    double turnByAngle;
    double arrowX, arrowY;

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

    boolean isSecond = false;

    ImprovedGamepad improvedGamepad;


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
        final int robotRadius = 8; // inches

        if (up == null) return;

        // We divide by 0.0254 to convert meters to inches


        arrowX = Math.cos(28.17859 + rotation.getRadians()) * robotRadius;
        arrowY = Math.sin(28.17859 + rotation.getRadians()) * robotRadius;
        double x1 = translation.getX(), y1 = translation.getY();
        double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
        double fieldTheta = rotation.getDegrees() + angleOffset - initTheta;
        double adjustX = ((x2*Math.cos(angleOffset)) - (y2*Math.sin(angleOffset)));
        double adjustY = ((x2*Math.sin(angleOffset)) + (y2*Math.cos(angleOffset)));

        //Changes target Position
        if(improvedGamepad.dpad_right.isInitialPress()){
            moveTo(24, 0);
           // kp +=0.01;
        } else if(improvedGamepad.dpad_left.isInitialPress()){
            moveTo(-52, 0);
            //kp -=0.01;
        }
        if(improvedGamepad.dpad_up.isInitialPress()){
            moveTo(-52, 24);
           // kd += 0.01;
        } else if(improvedGamepad.dpad_down.isInitialPress()){
            moveTo(0, -24);
          //  kd -= 0.01;
        }



        if(improvedGamepad.left_bumper.isInitialPress()){
            turnByAngle(90);
        } else if(improvedGamepad.right_bumper.isInitialPress()){
            turnByAngle(-90);
        }


//        if(autoState == AutoState.MOVE_FORWARD){
//            moveTo(-50, 0);
//        } else if(autoState == AutoState.TURN_45){
//            moveTo(-50, 20);
//        }
//        if(autoState == AutoState.MOVE_FORWARD){
//            moveTo(48, 0);
//        } else if(autoState == AutoState.TURN_45){
//            move(0,0, 45);
//        } else if(autoState == AutoState.LIFT_SLIDES){
//            //Slides stuff
//        } else if(autoState == AutoState.RELEASE_CLAW){
//            robot.clawOne.setPosition(0.1);
//            robot.clawTwo.setPosition(0.9);
//            autoState = AutoState.RETRACT_SLIDES;
//        } else if(autoState == AutoState.RETRACT_SLIDES){
//            //Slides stuff
//        } else if(autoState == AutoState.TURN_452){
//            turnByAngle(45);
//        } else if(autoState == AutoState.MOVE_BACK){
//            moveTo(48, 24);
//        } else if(autoState == AutoState.GRIP_CLAW){
//            robot.clawOne.setPosition(.2);
//            robot.clawTwo.setPosition(.8);
//            autoState = AutoState.MOVE_FORWARD2;
//        } else if(autoState == AutoState.MOVE_FORWARD2){
//            moveTo(48, 0);
//        } else if(autoState == AutoState.TURN_N45){
//            turnByAngle(-45);
//        }

        //Set lift heights
        if(improvedGamepad.x.isInitialPress()){
            robot.liftOne.setTargetPosition(850);
            robot.liftOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftOne.setPower(0.9);

        } else if(improvedGamepad.b.isInitialPress()){
            robot.liftOne.setTargetPosition(10);
            robot.liftOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftOne.setPower(-0.9);

        }
        if(robot.liftOne.getCurrentPosition() == robot.liftOne.getTargetPosition()){
            robot.liftOne.setPower(0);
        }

        if(robot.liftOne.getPower() != 0){
            robot.liftTwo.setPower(robot.liftOne.getPower());
        } else {
            robot.liftTwo.setPower(0);
        }

        //Movement PID code
        if(Math.abs(((positionX) - (translation.getX()))) > 1 && Math.abs((positionY) - (translation.getY())) > 1) {

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
        }

        outputLeft *= -1;
        angleToTarget += rotation.getDegrees();
        outputLeft *= 100.0;
        outputLeft = (int)outputLeft;
        outputLeft /= 100.0;

        double turnPower = AngleUnit.normalizeDegrees(targetAngle - rotation.getDegrees())/500;
        outputRight = outputLeft - (turnPower*Math.cos(Math.toRadians(angleToTarget)));
        outputLeft += turnPower*Math.cos(Math.toRadians(angleToTarget));
        double leftTurnPower = leftPodTurn(angleToTarget-(45*turnPower*Math.sin(Math.toRadians(angleToTarget))));
        double rightTurnPower = rightPodTurn(angleToTarget+(45*turnPower*Math.sin(Math.toRadians(angleToTarget))));

        robot.leftOne.setVelocity((outputLeft/speedMod+leftTurnPower)*2500);
        robot.leftTwo.setVelocity((outputLeft/speedMod-leftTurnPower)*2500);
        robot.rightOne.setVelocity((outputRight/speedMod+rightTurnPower)*2500);
        robot.rightTwo.setVelocity((outputRight/speedMod-rightTurnPower)*2500);

        if(autoState == AutoState.MOVE_FORWARD && robot.leftOne.getPower() == 0 && isSecond){
            autoState = AutoState.TURN_45;
        } else if(autoState == AutoState.MOVE_BACK && robot.leftOne.getPower() == 0){
            autoState = AutoState.GRIP_CLAW;
        } else if(autoState == AutoState.MOVE_FORWARD2 && robot.leftOne.getPower() == 0){
            autoState = AutoState.TURN_N45;
        } else if((autoState == AutoState.TURN_45 && robot.leftTwo.getPower() == 0) || (autoState == AutoState.TURN_N45 && robot.leftTwo.getPower() == 0)){
            autoState = AutoState.LIFT_SLIDES;
        } else if(autoState == AutoState.TURN_452 && robot.leftTwo.getPower() == 0){
            autoState = AutoState.MOVE_BACK;
        }

        telemetry.addData("Auto State", autoState);
        telemetry.addData("output", outputLeft);
        telemetry.addData("x1", String.format("%.2f", x1));
        telemetry.addData("y1", String.format("%.2f", y1));
//        telemetry.addData("x2", String.format("%.2f", x2));
//        telemetry.addData("y2", String.format("%.2f", y2));
        telemetry.addData("Adjusted Rotation", String.format("%.2f", fieldTheta) + "°");
        telemetry.addData("Adjusted x", String.format("%.2f", adjustX) + " inches");
        telemetry.addData("Adjusted y", String.format("%.2f", adjustY) + " inches");
        telemetry.addData("Translation", translation);
        telemetry.addData("addX", addX);
        telemetry.addData("addY", addY);
        telemetry.addData("Slide Position", robot.liftOne.getCurrentPosition());
        telemetry.addData("Slide Target Position", robot.liftOne.getTargetPosition());
        telemetry.addData("kp", kp);
        telemetry.addData("kd", kd);
        telemetry.addData("ki", ki);
        telemetry.addData("Angle to Target", angleToTarget + "°");
        telemetry.addData("Raw Rotation", rotation);

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
    }

    public void moveTo(double x, double y) {
        positionX = x;
        positionY = y;
        isSecond = true;
    }

    public void move(double x, double y, double angle){
        positionX = x + translation.getX();
        positionY = y + translation.getY();
        targetAngle = angle;
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

    public void turnByAngle(double turnAngle) {
        double startAngle = rotation.getDegrees();
        double targetAngle = AngleUnit.normalizeDegrees(startAngle + turnAngle);
        ElapsedTime runtime = new ElapsedTime();
        double p = 0;
        double i = 0;
        double d = 0;
        double kp = 0.1;
        double ki = 0;
        double kd = 0;
        double error = turnAngle;


        if(!(error < 1.5 && error > -1.5)){
            error = (targetAngle - robot.getAngle());
            double secs = runtime.seconds();
            runtime.reset();
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Current Angle", robot.getAngle());
            telemetry.addData("Loop Time", secs);
            d = (error - p) / secs;
            i = i + (error * secs);
            p = error;
            double total = (kp* p + ki* i + kd* d)/100;
            if(total > 1){
                i = 0;
                total = 1;
            }
            if(total < -1){
                i = 0;
                total = -1;
            }

            robot.leftOne.setPower(total);
            robot.rightOne.setPower(total);
            robot.leftTwo.setPower(total);
            robot.rightTwo.setPower(total);
        }

        robot.leftOne.setPower(0);
        robot.leftTwo.setPower(0);
        robot.rightOne.setPower(0);
        robot.rightTwo.setPower(0);

//        if((autoState == AutoState.TURN_45 && robot.leftTwo.getPower() == 0) || (autoState == AutoState.TURN_N45 && robot.leftTwo.getPower() == 0)){
//            autoState = AutoState.LIFT_SLIDES;
//        } else if(autoState == AutoState.TURN_452 && robot.leftTwo.getPower() == 0){
//            autoState = AutoState.MOVE_BACK;
//        }
    }
}
