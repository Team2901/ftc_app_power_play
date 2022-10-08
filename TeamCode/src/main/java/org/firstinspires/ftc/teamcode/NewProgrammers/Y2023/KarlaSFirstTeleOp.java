package org.firstinspires.ftc.teamcode.NewProgrammers.Y2023;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;

/*@TeleOp(name= "KarlaSTeleOp", group= "Shared")
class KarlaSFirstTeleOp extends OpMode implements OpenCvCamera.AsyncCameraOpenListener {


    public DcMotor frontLeftDrive;
    public DcMotor frontRightDrive;

    //Defining the Servo
    public Servo Servo;
    //Defining touch sensor
    public TouchSensor touchSensor;
    DistanceSensor distanceSensor;
    OpenCvCamera camera;


    MyPipeline myPipeline = new MyPipeline();


    @Override
    public void init() {

        int cameraMonitorViewId = hardwareMap.appContext.
                getResources().getIdentifier( "cameraMonitorViewId", "id", hardwareMap.
                        appContext.getPackageName());


        camera.setPipeline(myPipeline);
        camera.openCameraDeviceAsync(this);




        // Involved with OpenCvCamera
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        Servo = hardwareMap.get(Servo.class, "servo");
        touchSensor = hardwareMap.get(TouchSensor.class, "touch_sensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance sensor");
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            Servo.setPosition(0);
        }else if (gamepad1.y){
            Servo.setPosition(1);
        }
        frontLeftDrive.setPower(gamepad1.left_stick_y);
        frontRightDrive.setPower(gamepad1.right_stick_y);
        telemetry.addData("Is touching", touchSensor.isPressed());
        telemetry.update();

    }

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
        // make any changes to image before returning
        return lastImage;

        }
    }


    @Override
    public void onOpened() {

    }

    @Override
    public void onError(int errorCode) {

    }
}*/

