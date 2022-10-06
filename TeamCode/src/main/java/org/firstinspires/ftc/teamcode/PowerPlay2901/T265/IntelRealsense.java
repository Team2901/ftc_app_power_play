package org.firstinspires.ftc.teamcode.PowerPlay2901.T265;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.PowerPlay2901.EarlyDiffy.EarlyDiffyHardware;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

import java.util.concurrent.TimeUnit;


@TeleOp(name="Test T265", group="Iterative Opmode")
public class IntelRealsense extends OpMode
{
    // We treat this like a singleton because there should only ever be one object per camera
    private static T265Camera slamra = null;
    public double initTheta;

    T265Camera.CameraUpdate up;

    Translation2d translation;
    Rotation2d rotation;

    ElapsedTime time = new ElapsedTime();
    ElapsedTime impTime = new ElapsedTime();

//    EarlyDiffyHardware robot = new EarlyDiffyHardware();

    double currentTime;
    double previousTime;
    double previousError;
    double currentError;

    double p;
    double i;
    double d;
    final double kp = 0;
    final double ki = 0;
    final double kd = 0;
    final double max_i = 0.5;

    double addX = 0;

    ImprovedGamepad improvedGamepad;


    @Override
    public void init() {
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
        }
        initTheta = slamra.getLastReceivedCameraUpdate().pose.getRotation().getDegrees();
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
        double fieldTheta = rotation.getDegrees() - initTheta;
        double adjustX = ((x2*rotation.getCos()) - (y2*rotation.getSin()));
        double adjustY = ((x2*rotation.getSin()) + (y2*rotation.getCos()));

        if(improvedGamepad.a.isInitialPress()){
            addX += 10;
        }
        move(translation.getX() + addX, 0);
        telemetry.addData("x1", String.format("%.2f", x1));
        telemetry.addData("y1", String.format("%.2f", y1));
        telemetry.addData("x2", String.format("%.2f", x2));
        telemetry.addData("y2", String.format("%.2f", y2));
        telemetry.addData("Raw Rotation", rotation);
        telemetry.addData("Adjusted Rotation", String.format("%.2f", fieldTheta) + "°");
        telemetry.addData("Adjusted x", String.format("%.2f", adjustX) + " inches");
        telemetry.addData("Adjusted y", String.format("%.2f", adjustY) + " inches");
        telemetry.addData("Translation", translation);
    }

    @Override
    public void stop() {slamra.stop();}

    public double getX(){
        return translation.getX();
    }
    public double getY(){
        return translation.getY();
    }

//    public double getAngleDegrees(){
//        return slamra.getLastReceivedCameraUpdate().pose.getRotation().getDegrees();
//    }
//    public double getAngleRadians(){
//        return slamra.getLastReceivedCameraUpdate().pose.getRotation().getRadians();
//    }
    //enter (x, y) coordinates to move robot wheels to angle and position
    public void move(double x, double y) {
        double positionX = translation.getX() + x;
        double positionY = translation.getY() + y;
        if(Math.abs(positionX - translation.getX()) > 1 || Math.abs(positionY - translation.getY()) > 1){
            double dx = positionX - translation.getX();
            double dy = positionY - translation.getY();
            double angle = Math.atan(dy/dx);
            double hypotenuse = Math.sqrt((Math.pow(dx, 2)+Math.pow(dy, 2)));
            currentError = hypotenuse;

            currentTime = time.time(TimeUnit.MILLISECONDS);

            p = kp * currentError;
            i += ki * (currentError * (currentTime-previousTime));

            if(i > max_i) {
                i = max_i;
            }else if(i < -max_i) {
                i = -max_i;
            }

            d = kd * (currentError - previousError) / (currentTime - previousTime);

            double output = p + i + d;

//            robot.leftOne.setVelocity(output);
//            robot.leftTwo.setVelocity(output);
//            robot.rightOne.setVelocity(output);
//            robot.rightTwo.setVelocity(output);

            telemetry.addData("output power", output);
            previousError = currentError;
            previousTime = currentTime;

        }


    }

}
