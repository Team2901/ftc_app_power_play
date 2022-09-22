package org.firstinspires.ftc.teamcode.NewProgrammers.Y2023;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name = "Nikki A First TeleOp")
public class NikkiAFirstTeleOp extends OpMode implements OpenCvCamera.AsyncCameraOpenListener {
    MyPipeline myPipeline = new MyPipeline();
    public DcMotor leftDrive;
    public DcMotor rightDrive;
    public Servo hand;
    OpenCvCamera camera;

    @Override
    public void onOpened() {

    }

    @Override
    public void onError(int errorCode) {

    }

    private enum HandWavingState{Stopped, MovingCCW, MovingCW};
    private HandWavingState currentHandWavingState = HandWavingState.Stopped;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime servoTimer = new ElapsedTime();
    ImprovedGamepad impGamepad;

    @Override
    public void init() {
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        hand = hardwareMap.get(Servo.class, "hand");
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        impGamepad  = new ImprovedGamepad(this.gamepad1, this.timer, "GP");
        int cameraMoniterViewId = hardwareMap.appContext.getResources().getIdentifier
                ("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera  = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMoniterViewId);
        camera.setPipeline(myPipeline);
        camera.openCameraDeviceAsync(this);
    }

    @Override
    public void loop() {
        impGamepad.update();
        leftDrive.setPower(gamepad1.left_stick_y);
        rightDrive.setPower(gamepad1.right_stick_y);
        switch(currentHandWavingState){
            case Stopped:
                if(impGamepad.x.isInitialPress()) {
                    hand.setPosition(1);
                    currentHandWavingState = HandWavingState.MovingCCW;
                    servoTimer.reset();
                }
                break;
            case MovingCCW:
                if(impGamepad.x.isInitialPress()){
                    hand.setPosition(.5);
                    currentHandWavingState = HandWavingState.Stopped;
                }
                if(servoTimer.milliseconds() > 500){
                    hand.setPosition(0);
                    servoTimer.reset();
                    currentHandWavingState = HandWavingState.MovingCW;
                }
                break;
            case MovingCW:
                if(impGamepad.x.isInitialPress()){
                    hand.setPosition(.5);
                    currentHandWavingState = HandWavingState.Stopped;
                }
                if(servoTimer.milliseconds() > 500){
                    hand.setPosition(1);
                    servoTimer.reset();
                    currentHandWavingState = HandWavingState.MovingCCW;
                }
                break;
        }
    }
    class MyPipeline extends OpenCvPipeline{
        Mat lastImage = null;

        @Override
        public Mat processFrame(Mat input) {
            if (input == null) {
                return null;
            }
            if (lastImage != null){
                lastImage.release();
            }
            lastImage = new Mat();
            input.copyTo(lastImage);
            return lastImage;
        }
    }
}
