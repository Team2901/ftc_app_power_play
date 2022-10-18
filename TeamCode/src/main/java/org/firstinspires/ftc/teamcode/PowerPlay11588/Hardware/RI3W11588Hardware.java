package org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RI3W11588Hardware {
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor arm;
    public Servo claw;

    public static final double TICKS_PER_MOTOR_REV = 537.7;
    public static final double FRONT_GEAR_RATIO = 64/72;
    public static final double FRONT_TICKS_PER_DRIVE_REV = TICKS_PER_MOTOR_REV * FRONT_GEAR_RATIO;
    public static final double WHEEL_CIRCUMFERENCE = 4 * Math.PI;
    public static final double FRONT_TICKS_PER_INCH = FRONT_TICKS_PER_DRIVE_REV * WHEEL_CIRCUMFERENCE;
    public static final double BACK_TICKS_PER_INCH = TICKS_PER_MOTOR_REV/WHEEL_CIRCUMFERENCE;

    public void init(HardwareMap hardwareMap){
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        arm = hardwareMap.dcMotor.get("arm");
        claw = hardwareMap.servo.get("claw");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

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
}