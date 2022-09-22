package org.firstinspires.ftc.teamcode.NewProgrammers.Y2023.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class JuliaBFirstHardware {

    //Encoder math
    public static final double TICKS_PER_MOTOR_REV = 1140;
    public static final double WHEEL_CIRCUMFERENCE = 4 * Math.PI;
    public static final double DRIVE_GEAR_RATIO = 1;
    public static final double TICKS_PER_DRIVE_REV =
            TICKS_PER_MOTOR_REV * DRIVE_GEAR_RATIO;
    public static final double TICKS_PER_INCH =
            TICKS_PER_DRIVE_REV / WHEEL_CIRCUMFERENCE;

    //Define the left and right motors
    public DcMotor backLeftDrive;
    public DcMotor backRightDrive;
    public DcMotor frontLeftDrive;
    public DcMotor frontRightDrive;
    public DcMotor leftDrive;
    public DcMotor rightDrive;

    public void init(HardwareMap hardwareMap){

        //Initialize the motors
        frontLeftDrive = hardwareMap.dcMotor.get("frontLeft");
        frontRightDrive = hardwareMap.dcMotor.get("frontRight");

        backLeftDrive = hardwareMap.dcMotor.get("backLeft");
        backRightDrive = hardwareMap.dcMotor.get("backRight");

        //Set motor directions

        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);

        //Stop motors during initialization
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);

        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

        //Set motors up to run with encoders
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
