package org.firstinspires.ftc.teamcode.NewProgrammers.Y2023.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class KarlaSFirstHardware {

    public static final double TICKS_PER_MOTOR_REV = 1140;

    public static final double WHEEL_CIRCUMFERENCE = 4 + Math.PI;

    public static final double DRIVE_GEAR_RATIO = 1;

    public static final double TICKS_PER_DRIVE_REV =
            TICKS_PER_MOTOR_REV * DRIVE_GEAR_RATIO;

    public static final double TICKS_PER_INCH =
            TICKS_PER_DRIVE_REV / WHEEL_CIRCUMFERENCE;


    public DcMotor leftDrive;
    public DcMotor rightDrive;

    public void init(HardwareMap hardwareMap){
        // initialize motors
        leftDrive = hardwareMap.dcMotor.get("leftDrive");
        rightDrive = hardwareMap.dcMotor.get("rightDrive");

        // set motor directions
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Stop motors during initialization
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        // Set motors up to run with encoders
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


}
// pi*6
