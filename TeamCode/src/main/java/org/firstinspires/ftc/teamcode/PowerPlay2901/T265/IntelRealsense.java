package org.firstinspires.ftc.teamcode.PowerPlay2901.T265;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

    final double angleOffset = 45;

    T265Camera.CameraUpdate up;

    Translation2d translation;
    Rotation2d rotation;

    ElapsedTime time = new ElapsedTime();
    ElapsedTime impTime = new ElapsedTime();

    private ElapsedTime runtime = new ElapsedTime();

    EarlyDiffyHardware robot = new EarlyDiffyHardware();

    double currentTime;
    double previousTime;
    double previousError;
    double currentError;

    double p;
    double i;
    double d;

    double kp = 0.1;
    final double ki = 0;
    final double kd = 0;
    final double max_i = 1;

    double addX = 0;
    double addY = 0;

    double positionX;
    double positionY;
    double targetAngle = 0;

    double angleToTarget;

    double outputLeft;
    double outputRight;
    double speedMod = 3;

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
        final int robotRadius = 0; // inches

        if (up == null) return;

        // We divide by 0.0254 to convert meters to inches


        double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
        double x1 = translation.getX(), y1 = translation.getY();
        double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
        double fieldTheta = rotation.getDegrees() + angleOffset - initTheta;
        double adjustX = ((x2*rotation.getCos()) - (y2*rotation.getSin()));
        double adjustY = ((x2*rotation.getSin()) + (y2*rotation.getCos()));

        if(improvedGamepad.dpad_right.isInitialPress()){
            addX ++;
            move(addX, addY);
        } else if(improvedGamepad.dpad_left.isInitialPress()){
            addX --;
            move(addX, addY);
        }

        if(improvedGamepad.dpad_up.isInitialPress()){
//            kp += 0.01;
//            kp *= 100;
//            kp = (int)kp;
//            kp /= 100;
            addY++;
            move(addX, addY);
        } else if(improvedGamepad.dpad_down.isInitialPress()){
//            kp -= 0.01;
//            kp *= 100;
//            kp = (int)kp;
//            kp /= 100;
            addY--;
            move(addX, addY);
        }

        if(Math.abs(positionX - translation.getX()) > .1 || Math.abs(positionY - translation.getY()) > .1){
            double dx = positionX - translation.getX();
            double dy = positionY - translation.getY();
            double angle = Math.atan(dy/dx) + angleOffset;
            double hypotenuse = Math.sqrt((Math.pow(dx, 2)+Math.pow(dy, 2)));
            currentError = hypotenuse;
            angleToTarget = Math.toDegrees(angle);

            if(dx < 0){
                angleToTarget += 180;
            } else if(dx > 0 && dy < 0){
                angleToTarget += 360;
            }


            currentTime = time.time(TimeUnit.MILLISECONDS);

            p = kp * currentError;
            i += ki * (currentError * (currentTime-previousTime));

            if(i > max_i) {
                i = max_i;
            }else if(i < -max_i) {
                i = -max_i;
            }

            d = kd * (currentError - previousError) / (currentTime - previousTime);

            outputLeft = p + i + d;

            previousError = currentError;
            previousTime = currentTime;
            if(outputLeft > 1) {
                outputLeft = 1;
            } else if (outputLeft < -1){
                outputLeft = -1;
            }
            if(currentError - Math.sqrt(Math.pow(translation.getX(), 2) + Math.pow(translation.getY(), 2)) < 0){
                outputLeft = -outputLeft;
            }
        } else {
            outputLeft = 0;}
        double turnPower = AngleUnit.normalizeDegrees(targetAngle - rotation.getDegrees())/500;
        outputRight = outputLeft - (turnPower*rotation.getCos());
        outputLeft += turnPower*rotation.getCos();
        double leftTurnPower = leftPodTurn(-angleToTarget-(Math.abs(1/(outputRight/speedMod))*turnPower*rotation.getSin()));
        double rightTurnPower = rightPodTurn(-angleToTarget+(Math.abs(1/(outputRight/speedMod))*turnPower*rotation.getSin()));
        robot.leftOne.setPower((outputLeft/speedMod + leftTurnPower)*2500);
        robot.leftTwo.setPower((outputLeft/speedMod - leftTurnPower)*2500);
        robot.leftOne.setPower((outputRight/speedMod + rightTurnPower)*2500);
        robot.leftTwo.setPower((outputRight/speedMod - rightTurnPower)*2500);

        telemetry.addData("output", outputLeft);
        telemetry.addData("x1", String.format("%.2f", x1));
        telemetry.addData("y1", String.format("%.2f", y1));
        telemetry.addData("x2", String.format("%.2f", x2));
        telemetry.addData("y2", String.format("%.2f", y2));
        telemetry.addData("Raw Rotation", rotation);
        telemetry.addData("Adjusted Rotation", String.format("%.2f", fieldTheta) + "°");
        telemetry.addData("Adjusted x", String.format("%.2f", adjustX) + " inches");
        telemetry.addData("Adjusted y", String.format("%.2f", adjustY) + " inches");
        telemetry.addData("Translation", translation);
        telemetry.addData("addX", addX);
        telemetry.addData("addY", addY);
        telemetry.addData("kp", kp);
        telemetry.addData("Angle to Target", angleToTarget + "°");
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
}
