package org.firstinspires.ftc.teamcode.Outreach.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Outreach.Hardware.NewClawbotHardware;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

@TeleOp(name="New Clawbot Teleop", group="Outreach")
public class NewClawbotTeleOp extends OpMode {
    public enum clawState{
        OPEN, CLOSED
    }
    public ImprovedGamepad gamepad;
    public ImprovedGamepad masterGamepad;
    public ElapsedTime gamepadTimer = new ElapsedTime();
    public clawState currentClawState;

    NewClawbotHardware robot = new NewClawbotHardware();
    @Override
    public void init() {
        gamepad = new ImprovedGamepad(gamepad1, gamepadTimer, "gamepad");
        masterGamepad = new ImprovedGamepad(gamepad2, gamepadTimer, "masterGamepad");
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        gamepad.update();
        masterGamepad.update();
        robot.leftDrive.setPower(gamepad.left_stick_y.getValue());
        robot.rightDrive.setPower(gamepad.right_stick_y.getValue());

    }
}