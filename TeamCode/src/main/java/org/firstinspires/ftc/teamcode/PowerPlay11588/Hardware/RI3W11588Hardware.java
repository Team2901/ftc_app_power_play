package org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.OpenCVPipelines.Qual11588OpenCV;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.OpenCVPipelines.Qual11588OpenCV;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class RI3W11588Hardware implements OpenCvCamera.AsyncCameraOpenListener {
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor arm;
    public Servo claw;
    public OpenCvCamera camera;
    public Telemetry telemetry;
    public Qual11588OpenCV pipeLine;
    public BNO055IMU imu;


    public static final double TICKS_PER_MOTOR_REV = 537.7;
    public static final double FRONT_GEAR_RATIO = 64.0/72.0;
    public static final double BACK_GEAR_RATIO = 1/1;
    public static final double FRONT_TICKS_PER_DRIVE_REV = TICKS_PER_MOTOR_REV * FRONT_GEAR_RATIO;
    public static final double BACK_TICKS_PER_DRIVE_REV = TICKS_PER_MOTOR_REV * BACK_GEAR_RATIO;
    public static final double WHEEL_CIRCUMFERENCE = 4 * Math.PI;
    public static final double FRONT_TICKS_PER_INCH = FRONT_TICKS_PER_DRIVE_REV/WHEEL_CIRCUMFERENCE;
    public static final double BACK_TICKS_PER_INCH = BACK_TICKS_PER_DRIVE_REV/WHEEL_CIRCUMFERENCE;

    public static final double ARM_GEAR_RATIO = 16/80;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        init(hardwareMap, telemetry, true);
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry, boolean useCam){
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        arm = hardwareMap.dcMotor.get("arm");
        claw = hardwareMap.servo.get("claw");
        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        this.telemetry = telemetry;

        //parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        //imu.initialize(parameters);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        if(useCam) {
            WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

            int cameraMonitorViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewID);

            pipeLine = new Qual11588OpenCV(telemetry);
            camera.setPipeline(pipeLine);
            camera.openCameraDeviceAsync(this);
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        arm.setPower(0);
        claw.setPosition(0);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getAngle(){
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.normalizeDegrees(orientation.firstAngle);
    }

    @Override
    public void onOpened() {
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    @Override
    public void onError(int errorCode) {

    }
}