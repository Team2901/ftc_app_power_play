package org.firstinspires.ftc.teamcode.Shared.Hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.AnalogInput;

import java.util.ArrayList;
import java.util.List;


/**
 * Created by Kearneyg20428 on 2/7/2017.
 */

public class ClawbotHardware {

    public static final double MID_SERVO = 0.5;
    public static final double ARM_UP_POWER = 1;
    public static final double ARM_DOWN_POWER = -0.5;
    public static final double MIN_SAFE_CLAW_OFFSET = 0;
    public static final double MAX_SAFE_CLAW_OFFSET = .4;
    private final ElapsedTime period = new ElapsedTime();
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public DcMotor armMotor = null;
    public Servo claw = null;
    private HardwareMap hwMap = null;
    public AnalogInput potentiometer;
    public static final double ARM_DOWN_VOLTAGE = 3.3;
    public static final double ARM_UP_VOLTAGE = 1.35;
    public static final double DANCE_ARM_DOWN_VOLTAGE = 3;
    public static final double DANCE_ARM_UP_VOLTAGE = 1.35;
    public static final double ARM_DOWN_SAFE_MOVEMENT_VOLTAGE = 3;
    public List<String> failedHardware = new ArrayList<>();

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor = hwMap.dcMotor.get("left_drive");
        rightMotor = hwMap.dcMotor.get("right_drive");
        armMotor = hwMap.dcMotor.get("left_arm");
        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        armMotor.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        claw = hwMap.servo.get("left_hand");

        claw.setPosition(0);

        //Set up the potentiometer
        potentiometer = getAnalogInput(hwMap, "potentiometer");
    }

    public void waitForTick(long periodMs) {

        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();

    }

    public AnalogInput getAnalogInput(HardwareMap hwMap, String name){
        try{
            return hwMap.analogInput.get(name);
        } catch(Exception e){
            failedHardware.add(name);
            //return new MockAnalogInput();
            return null;
        }
    }

}
