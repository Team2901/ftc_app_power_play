package org.firstinspires.ftc.teamcode.PowerPlay2901.Hardware;

import android.graphics.Camera;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.NewProgrammers.Y2023.Mecanum.ObjectDetectionPipeline;
import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.OpenCVPipelines.Qual11588OpenCV;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class RockBotHardware implements OpenCvCamera.AsyncCameraOpenListener {
    private ElapsedTime runtime = new ElapsedTime();

    public static final double ticksPerMotorRev = 134.4;
    public static final double podDriveRatio = 1/3;
    public static final double wheelDriveRatio = 25/12;
    public static final double ticksPerInch = (ticksPerMotorRev/wheelDriveRatio)/(Math.PI*2.835)/2;
    public static final double tickDiffPerDegree = (ticksPerMotorRev/podDriveRatio)/720;
    public static final double encoderTicksPerWheelRev = 8192;
    public static final double wheelCircumference = (3 * Math.PI);
    public static final double wheelRadius = 1.5;
    public static final double leftRightDistance = 12;
    public static final double midpointBackDistance = 12;
    public static final double encoderTicksPerInch = encoderTicksPerWheelRev/wheelCircumference;
    public static final double inchPerTick = wheelCircumference/encoderTicksPerWheelRev;

    public DcMotorEx leftOne;
    public DcMotorEx leftTwo;
    public DcMotorEx rightOne;
    public DcMotorEx rightTwo;
    public DcMotor liftOne;
    public DcMotor liftTwo;
    public DcMotor encoderLeft;
    public DcMotor encoderRight;

    public Servo passthrough;

    public BNO055IMU imu;

    public OpenCvCamera camera;
    public ObjectDetectionPipeline pipeLine;
    public void init(HardwareMap hardwareMap) {
        init(hardwareMap,null,false);
    }
    public void init(HardwareMap hardwareMap, Telemetry telemetry, boolean useCam) {
        leftOne = hardwareMap.get(DcMotorEx.class, "left 1");
        leftTwo = hardwareMap.get(DcMotorEx.class, "left 2");
        rightOne = hardwareMap.get(DcMotorEx.class, "right 1");
        rightTwo = hardwareMap.get(DcMotorEx.class, "right 2");
        liftOne = hardwareMap.get(DcMotor.class, "lift 1");
        liftTwo = hardwareMap.get(DcMotor.class, "lift 2");
        passthrough = hardwareMap.get(Servo.class, "passthrough");
        encoderLeft = hardwareMap.get(DcMotor.class, "left encoder");
        encoderRight = hardwareMap.get(DcMotor.class, "right encoder");

        leftTwo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightTwo.setDirection(DcMotorSimple.Direction.REVERSE);
        liftTwo.setDirection(DcMotorSimple.Direction.REVERSE);
        encoderRight.setDirection(DcMotorSimple.Direction.REVERSE);

        leftOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (useCam) {
            WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
            pipeLine = new ObjectDetectionPipeline(telemetry);
            int cameraMonitorViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewID);
            camera.setPipeline(pipeLine);
            camera.openCameraDeviceAsync(this);
        }

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public double getAngle() {
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
