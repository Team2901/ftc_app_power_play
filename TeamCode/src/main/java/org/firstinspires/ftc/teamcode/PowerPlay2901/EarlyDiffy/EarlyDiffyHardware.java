package org.firstinspires.ftc.teamcode.PowerPlay2901.EarlyDiffy;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class EarlyDiffyHardware {
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
//    public DcMotor liftOne;
//    public DcMotor liftTwo;
//    public Servo clawOne;
//    public Servo clawTwo;
    //public Rev2mDistanceSensor clawSensor;

    public int currentLeftPosition = 0;
    public int currentRightPosition = 0;
    public int currentBackPosition = 0;

    private int oldLeftPosition = 0;
    private int oldRightPosition = 0;
    private int oldBackPosition = 0;

    /*public XYhVector START_POS = new XYhVector(0, 0, getAngle());
    public XYhVector pos = new XYhVector(START_POS);*/


    public BNO055IMU imu;

    public void init(HardwareMap hardwareMap) {
        leftOne = hardwareMap.get(DcMotorEx.class, "left 1");
        leftTwo = hardwareMap.get(DcMotorEx.class, "left 2");
        rightOne = hardwareMap.get(DcMotorEx.class, "right 1");
        rightTwo = hardwareMap.get(DcMotorEx.class, "right 2");
//        liftOne = hardwareMap.get(DcMotor.class, "lift 1");
//        liftTwo = hardwareMap.get(DcMotor.class, "lift 2");
//        clawOne = hardwareMap.get(Servo.class, "claw 1");
//        clawTwo = hardwareMap.get(Servo.class, "claw 2");
//        //clawSensor = hardwareMap.get(Rev2mDistanceSensor.class, "claw sensor");
//
//        clawOne.setPosition(0.12);
//        clawTwo.setPosition(0.18);;

        leftTwo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightTwo.setDirection(DcMotorSimple.Direction.REVERSE);
        //liftTwo.setDirection(DcMotorSimple.Direction.REVERSE);

        leftOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        liftOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        liftTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

    /*public void odometry(){
        oldRightPosition = currentRightPosition;
        oldLeftPosition = currentLeftPosition;
        oldBackPosition = currentBackPosition;

        currentRightPosition = rightOne.getCurrentPosition(); CHANGE NAME :))
        currentLeftPosition = leftOne.getCurrentPosition();
        currentBackPosition = rightTwo.getCurrentPosition();

        int dn1 = currentRightPosition - oldRightPosition;
        int dn2 = currentLeftPosition - oldLeftPosition;
        int dn3 = currentBackPosition - oldBackPosition;

        double dtheta = ((dn2 - dn1) / leftRightDistance) * inchPerTick;
        double dx = ((dn1 + dn2) / 2) * inchPerTick;
        double dy = (dn3 - (dn2 - dn1) * midpointBackDistance / leftRightDistance) * inchPerTick;

        double theta = pos.h + (dtheta / 2.0);
        pos.x += dx * Math.cos(theta) - dy * Math.sin(theta);
        pos.y += dx * Math.sin(theta) + dy * Math.cos(theta);
        pos.h += dtheta;
    }*/

    public double getAngle() {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return AngleUnit.normalizeDegrees(orientation.firstAngle);
    }
}
