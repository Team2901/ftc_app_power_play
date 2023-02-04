package org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class NotSoSmolBotHardware{
    //THIS CLASS IS JUST FOR TEST DONT USE
    //Ticks per motor rev for a gobilda yellowjacket 312
    public static final double TICKS_PER_MOTOR_REV = 537.7;
    public static final double DRIVE_GEAR_RATIO = 1;
    public static final double TICKS_PER_DRIVE_REV = TICKS_PER_MOTOR_REV * DRIVE_GEAR_RATIO;
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * 3.78;
    public static final double TICKS_PER_INCH = TICKS_PER_DRIVE_REV / WHEEL_CIRCUMFERENCE;
    public static final double OPEN_POSITION = 0.5;
    public static final double CLOSED_POSITION = 0.15;
    public static final double GROUND_ENCODER_VALUE = 200;
    public static final double LOW_POLE_ENCODER_VALUE = 550;
    public static final double MID_POLE_ENCODER_VALUE = 800;
    public static final double HIGH_POLE_ENCODER_VALUE = 1150;
    public static final double ARM_GEAR_RATIO = 40.0/16.0;

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;
    public DcMotorEx arm;
    public Servo claw;
    /*
    public OpenCvCamera camera;
    public Qual11588OpenCV pipeLine;
    public Telemetry telemetry;

     */
    public BNO055IMU imu;
    //public static Qual11588Hardware.allianceColor teamColor = Qual11588Hardware.allianceColor.BLUE;

    /*
    public void autoInit(HardwareMap hardwareMap, Telemetry telemetry, Qual11588Hardware.allianceColor team) {
        init(hardwareMap, telemetry,true, team, true);
    }
     */

    /*
    public void teleOpInit(HardwareMap hardwareMap, Telemetry telemetry, Boolean useCam) {
        init(hardwareMap, telemetry, useCam, null, false);
    }
     */

    public void init(HardwareMap hardwareMap, boolean encoderReset){
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        claw = hardwareMap.servo.get("claw");
        //teamColor = team;

        //this.telemetry = telemetry;

        /*
        if (useCam) {
            WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
            pipeLine = new Qual11588OpenCV(telemetry, this);
            int cameraMonitorViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewID);
            camera.setPipeline(pipeLine);
            camera.openCameraDeviceAsync(this);
        }
         */

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if(encoderReset){
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        arm.setPower(0);
        claw.setPosition(CLOSED_POSITION);
    }

    public double getAngle(){
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.normalizeDegrees(orientation.firstAngle);
    }

    public enum Height {
        GROUND,
        LOW,
        MEDIUM,
        HIGH
    }
}
