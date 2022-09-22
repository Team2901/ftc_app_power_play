package org.firstinspires.ftc.teamcode.NewProgrammers.Y2023;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name = "Jacob C Tele Op", group = "Shared")
public class JacobCTeleOp extends OpMode implements OpenCvCamera.AsyncCameraOpenListener {

    public DcMotor frontLeftDrive;
    public DcMotor frontRightDrive;
    DistanceSensor rangeSensor;
    TouchSensor touchSensor;
    OpenCvCamera camera;
    @Override
    public void init() {

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightName");
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        servo = hardwareMap.get(Servo.class, "servo");
        int cameraMonitorViewId =
            hardwareMap.appContext.getResources().getIdentifier(
    "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName ());

        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        camera.setPipeline(myPipeline);
        camera.openCameraDeviceAsync(this);
    }

    public DcMotor leftDrive;
    public DcMotor rightDrive;

    Servo servo;

    @Override
    public void loop() {
        leftDrive.setPower(gamepad1.left_stick_y);
        rightDrive.setPower(gamepad1.right_stick_y);
        if(gamepad1.a){
            servo.setPosition(1);
        } else if(gamepad1.y){
            servo.setPosition(0);
        }

        telemetry.addData("Servo Position", servo.getPosition(

        ));
    }

    @Override
    public void onOpened() {

    }

    @Override
    public void onError(int errorCode) {

    }

    class MyPipeline extends OpenCvPipeline {
        Mat lastImage = null;

        @Override
        public Mat processFrame(Mat input) {

            if(input == null)
                return null;

            if (lastImage != null) {
                lastImage.release();
            }
            lastImage = new Mat ();
            input.copyTo(lastImage);
            return lastImage;

        }
    }
    MyPipeline myPipeline= new MyPipeline();

    }
