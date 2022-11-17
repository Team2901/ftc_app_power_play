package org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.OpenCVPipelines.RI3W11588OpenCV;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Qual11588Hardware implements OpenCvCamera.AsyncCameraOpenListener {
    //Ticks per motor rev for a gobilda yellowjacket 312
    public static final double TICKS_PER_MOTOR_REV = 537.7;
    public static final double DRIVE_GEAR_RATIO = 1;
    public static final double TICKS_PER_DRIVE_REV = TICKS_PER_MOTOR_REV * DRIVE_GEAR_RATIO;
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * 4;
    public static final double TICKS_PER_INCH = TICKS_PER_DRIVE_REV / WHEEL_CIRCUMFERENCE;

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;
    public DcMotorEx arm;
    public Servo claw;
    public OpenCvCamera camera;
    public RI3W11588OpenCV pipeLine;
    public Telemetry telemetry;

    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        claw = hardwareMap.servo.get("claw");
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewID);
        this.telemetry = telemetry;
        pipeLine = new RI3W11588OpenCV(telemetry);
        camera.setPipeline(pipeLine);
        camera.openCameraDeviceAsync(this);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        arm.setPower(0);
        claw.setPosition(0);
    }

    @Override
    public void onOpened() {
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    @Override
    public void onError(int errorCode) {

    }
}
