package org.firstinspires.ftc.teamcode.Outreach.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Outreach.Hardware.OutreachBotOneHardware;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

@TeleOp (name = "Outreach Joystick TeleOp", group = "Outreach")
public class OutreachBotStickTeleOp extends OpMode {
    OutreachBotOneHardware robot = new OutreachBotOneHardware();
    double participantLeftPower;
    double participantRightPower;
    double gmLeftPower;
    double gmRightPower;
    ImprovedGamepad participantGamepad;
    ImprovedGamepad gmGamepad;


    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        robot.leftDrive.setPower(gamepad1.left_stick_y);
        robot.rightDrive.setPower(gamepad1.right_stick_y);
    }
}
