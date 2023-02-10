package org.firstinspires.ftc.teamcode.Outreach.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Outreach.Hardware.NewClawbotHardware;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

@TeleOp(name="New Clawbot Teleop No PID", group="Outreach")
public class NewClawBotTeleOpNoPID extends OpMode {
    public enum ClawState {OPEN, CLOSED}
    public enum Controller{PARTICIPANT, MASTER}
    public NewClawbotTeleOp.ClawState currentClawState = NewClawbotTeleOp.ClawState.CLOSED;
    public ImprovedGamepad gamepad;
    public double voltage;
    public ImprovedGamepad masterGamepad;
    public ImprovedGamepad gamepadInControl;
    public ElapsedTime gamepadTimer = new ElapsedTime();

    public boolean gamepadOverride = false;

    NewClawbotHardware robot = new NewClawbotHardware();
    @Override
    public void init() {
        gamepad = new ImprovedGamepad(gamepad1, gamepadTimer, "gamepad");
        masterGamepad = new ImprovedGamepad(gamepad2, gamepadTimer, "masterGamepad");
        gamepadInControl = null;
        robot.init(hardwareMap);

    }

    @Override
    public void loop() {
        gamepad.update();
        masterGamepad.update();



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

        robot.leftDrive.setPower(-gamepadInControl.left_stick_y.getValue());
        robot.rightDrive.setPower(-gamepadInControl.right_stick_y.getValue());

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

        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        double scaleFactor = 12/result;
        voltage = scaleFactor * robot.potentiometer.getVoltage();


        if (gamepadInControl.dpad_up.getValue() && voltage > 2) {
            robot.arm.setPower(0.5);
        } else if (gamepadInControl.dpad_down.getValue() && voltage < 3.2) {
            robot.arm.setPower(-0.2);
        } else {
            robot.arm.setPower(0);
        }

        if ((gamepadInControl.left_stick_y.getValue() != 0 || gamepadInControl.right_stick_y.getValue() != 0) && voltage > 2.7) {
            robot.arm.setPower(0.3);
        }

        telemetry.addData("Override", gamepadOverride);
        telemetry.addData("Gamepad In Control", gamepadInControl);
        telemetry.addData("Claw State", currentClawState);
        telemetry.addData("Voltage", voltage);
        telemetry.update();

    }
}
