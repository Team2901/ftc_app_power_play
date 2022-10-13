package org.firstinspires.ftc.teamcode.Outreach.Hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

public class OutreachBotOneHardware {
    public static final double MID_SERVO = 0.5;
    public static final double ARM_UP_POWER = 1;
    public static final double ARM_DOWN_POWER = -0.5;
    public static final double MIN_SAFE_CLAW_OFFSET = 0;
    public static final double MAX_SAFE_CLAW_OFFSET = .4;
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public Servo claw = null;
    private HardwareMap hwMap = null;
    public List<String> failedHardware = new ArrayList<>();

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor = hwMap.dcMotor.get("left_drive");
        rightMotor = hwMap.dcMotor.get("right_drive");
        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        claw = hwMap.servo.get("left_hand");

        claw.setPosition(0);
    }

}
