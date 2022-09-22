package org.firstinspires.ftc.teamcode.NewProgrammers.Y2023;

import android.annotation.SuppressLint;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name = "Mary Ellen First Teleop", group = "Shared")
public class MaryEllenTFirstTeleop extends OpMode implements OpenCvCamera.AsyncCameraOpenListener{

    //public DcMotor backLeftDrive;
    //public DcMotor backRightDrive;
    public DcMotor frontLeftDrive;
    public DcMotor frontRightDrive;
    DistanceSensor rangeSensor;
    TouchSensor touchSensor;

    private static final String VUFORIA_KEY =
            "AYD16hT/////AAABmXE6CDo7rkFAsUSbY13SCFpmNZfOZ2H9yRC2ZprpwNQP8r9jOK4at4fblE9DWyvYH/SDJlO7f8Q9kEQEAt2kclE4WnZEJOuRKABBLmPUV4QoMIh0XKZmT7DOeHjRzQLNWsXxbHTYCW9nIJb7Hvl4ZVdYgoAE6hlIswHPl5j6gIm7XbcbbJZ2LYnSukM0c+hYnNmhf0L8+7OG+AzChMxJ7E3LE4v5X2HLR5pet//M7lKkxYmvBJPZD9OSxkX6zAxuC94Qonb27xtddZTyihG0MQfxUSULLU7xoaPJ8DAm+iSsIoPXd0lyc8O8FEcliIEoE8z5UjsN7bGEoRUu0LpsCqBRTGUbZ9dAs0tq1ts1Xzbm";

    OpenCvCamera camera;

    class MyPipeline extends OpenCvPipeline {
        Mat lastImage = null;

        @Override
        public Mat processFrame(Mat input) {
            if (input == null)
                return null;

            if (lastImage != null) {
                lastImage.release();
            }
            lastImage = new Mat();
            input.copyTo(lastImage);
            // make any changes to the image before returning
            return lastImage;
        }
    }
    MyPipeline myPipeline= new MyPipeline();

    @Override
    public void init() {
        //backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        //backRightDrive = hardwareMap.get(DcMotor.class,"backRight");

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class,"frontRight");

        //backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        rangeSensor = hardwareMap.get(DistanceSensor.class, "range_sensor");
        touchSensor = hardwareMap.get(TouchSensor.class, "touch_sensor");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);


        camera.setPipeline(myPipeline);
        camera.openCameraDeviceAsync(this);
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        //backLeftDrive.setPower(gamepad1.left_stick_y);
        //backRightDrive.setPower(gamepad1.right_stick_y);

        frontLeftDrive.setPower(gamepad1.left_stick_y);
        frontRightDrive.setPower(gamepad1.right_stick_y);

        telemetry.addData("range", String.format("%.01f in", rangeSensor.getDistance(DistanceUnit.INCH)));
        telemetry.addData("touch", touchSensor.isPressed() ? "Pressed": "*not* Pressed");


        if (touchSensor.isPressed()) {
            if (myPipeline.lastImage != null) {
                String imageFilePath = String.format("%s/FIRST/data/toebes.png", Environment.getExternalStorageDirectory().getAbsolutePath());
                Imgcodecs.imwrite(imageFilePath, myPipeline.lastImage);
                telemetry.addData("saved image", "");
            }
        }
        telemetry.update();
    }

    @Override
    public void onOpened() {
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    @Override
    public void onError(int errorCode) {
        //This will be called if the camera could not be opened
    }
}
