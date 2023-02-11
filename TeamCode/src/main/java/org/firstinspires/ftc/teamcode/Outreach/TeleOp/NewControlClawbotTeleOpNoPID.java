package org.firstinspires.ftc.teamcode.Outreach.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Outreach.Hardware.NewClawbotHardware;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

@TeleOp(name="OUTREACH New Control Clawbot Teleop", group="Outreach")
public class NewControlClawbotTeleOpNoPID extends OpMode {
    public static final int MINIMUM_LOW_ARM_VOLTAGE = 2;
    public static final double MAXIMUM_MEDIUM_ARM_VOLTAGE = 3.2;
    public static final double MINIMUM_HIGH_ARM_VOLTAGE = 1.3;
    private static NewClawbotHardware.ArmState armState = NewClawbotHardware.ArmState.GROUND;
    public enum ClawState {OPEN, CLOSED}
    public enum Controller{PARTICIPANT, MASTER}
    public NewClawbotTeleOp.ClawState currentClawState = NewClawbotTeleOp.ClawState.CLOSED;
    public ImprovedGamepad gamepad;
    public double voltage;
    public ImprovedGamepad masterGamepad;
    public ImprovedGamepad gamepadInControl;
    public ElapsedTime gamepadTimer = new ElapsedTime();
    public boolean override = false;
    public double rightPower = 0;
    public double leftPower = 0;


    public boolean gamepadOverride = false;

    NewClawbotHardware robot = new NewClawbotHardware();
    @Override
    public void init() {
        gamepad = new ImprovedGamepad(gamepad1, gamepadTimer, "gamepad");
        masterGamepad = new ImprovedGamepad(gamepad2, gamepadTimer, "masterGamepad");
        gamepadInControl = null;
        robot.init(hardwareMap);
        telemetry();
    }

    @Override
    public void loop() {
        gamepad.update();
        masterGamepad.update();

        if (masterGamepad.areButtonsActive()) {
            override = true;
        } else {
            override = false;
        }

        if(masterGamepad.x.isInitialPress() && gamepadOverride){
            gamepadOverride = false;
        }else if(masterGamepad.x.isInitialPress() && !gamepadOverride){
            gamepadOverride = true;
        }

        if(gamepadOverride || masterGamepad.areButtonsActive()){
            gamepadInControl = masterGamepad;
        }else{
            gamepadInControl = gamepad;
        }

        rightPower = (gamepadInControl.left_stick_y.getValue() / 2) + (gamepadInControl.right_stick_y.getValue() / 2) - (gamepadInControl.right_stick_x.getValue() / 2) - (gamepadInControl.left_stick_x.getValue() / 2);
        leftPower = (gamepadInControl.left_stick_y.getValue() / 2) + (gamepadInControl.right_stick_y.getValue() / 2) + (gamepadInControl.left_stick_x.getValue() / 2) + (gamepadInControl.right_stick_x.getValue() / 2);

        robot.leftDrive.setPower(-leftPower);
        robot.rightDrive.setPower(-rightPower);

        switch(currentClawState){
            case OPEN:
                robot.claw.setPosition(NewClawbotHardware.CLAW_OPEN_POSITION);
                if(gamepadInControl.b.isInitialPress()){
                    currentClawState = NewClawbotTeleOp.ClawState.CLOSED;
                }
                break;
            case CLOSED:
                robot.claw.setPosition(NewClawbotHardware.CLAW_CLOSED_POSITION);
                if(gamepadInControl.b.isInitialPress()){
                    currentClawState = NewClawbotTeleOp.ClawState.OPEN;
                }
                break;
        }

        switch (armState) {
            case GROUND:
            case LOW:
                if (gamepadInControl.dpad_up.isInitialPress()) {
                    armState = NewClawbotHardware.ArmState.MEDIUM;
                }
                break;
            case MEDIUM:
                if (gamepadInControl.dpad_up.isInitialPress()) {
                    armState = NewClawbotHardware.ArmState.HIGH;
                } else if (gamepadInControl.dpad_down.isInitialPress()) {
                    armState = NewClawbotHardware.ArmState.LOW;
                }
                break;
            case HIGH:
                if (gamepadInControl.dpad_down.isInitialPress()) {
                    armState = NewClawbotHardware.ArmState.MEDIUM;
                }
        }

        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        double scaleFactor = 12/result;
        voltage = scaleFactor * robot.potentiometer.getVoltage();

        if ((gamepadInControl.right_stick_y.getValue() != 0 || gamepadInControl.left_stick_y.getValue() != 0 || gamepadInControl.left_stick_x.getValue() != 0 || gamepadInControl.right_stick_x.getValue() != 0) && (armState == NewClawbotHardware.ArmState.GROUND || armState == NewClawbotHardware.ArmState.LOW) ) {
            armState = NewClawbotHardware.ArmState.LOW;
        }

        if (armState == NewClawbotHardware.ArmState.MEDIUM && voltage > MINIMUM_LOW_ARM_VOLTAGE) {
            robot.arm.setPower(0.5);
        } else if (armState == NewClawbotHardware.ArmState.LOW && voltage < MAXIMUM_MEDIUM_ARM_VOLTAGE) {
            robot.arm.setPower(-0.2);
        } else if (armState == NewClawbotHardware.ArmState.HIGH && voltage > MINIMUM_HIGH_ARM_VOLTAGE) {
            robot.arm.setPower(0.3);
        } else {
            robot.arm.setPower(0);
        }

        if ((rightPower != 0 || leftPower != 0) && voltage > 2.7) {
            robot.arm.setPower(0.3);
        }

        telemetry();

    }

    public void telemetry() {
        telemetry.addData("Master Control Start", "Start + B");
        telemetry.addData("Player Control Start", "Start + A");
        telemetry.addData("Forward/Backward", "Left and Right game stick");
        telemetry.addData("Open/Close claw", "B");
        telemetry.addData("Move Arm Up", "D-pad up");
        telemetry.addData("Move Arm Down", "D-pad down");
        telemetry.addData("Master Gamepad Override", "X");
        telemetry.addData("Override", override);
        telemetry.update();
    }
}
