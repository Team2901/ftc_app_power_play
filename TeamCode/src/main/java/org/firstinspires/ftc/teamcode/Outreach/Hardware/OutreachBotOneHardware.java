package org.firstinspires.ftc.teamcode.Outreach.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

public class OutreachBotOneHardware {
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public Servo claw = null;
    private HardwareMap hardwareMap = null;
    public List<String> failedHardware = new ArrayList<>();
    public enum ClawState { OPEN, CLOSED }
    public final double CLOSED_POSITON = 0;
    public final double OPEN_POSITON = 0.75;
    public ClawState currentClawState = ClawState.CLOSED;

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;

        //Refactored it to have variable syntax and called them drive instead of motor
        // Define and Initialize Motors
        leftDrive = hardwareMap.dcMotor.get("leftDrive");
        rightDrive = hardwareMap.dcMotor.get("rightDrive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        claw = hardwareMap.servo.get("claw");

        claw.setPosition(0);
    }

}
