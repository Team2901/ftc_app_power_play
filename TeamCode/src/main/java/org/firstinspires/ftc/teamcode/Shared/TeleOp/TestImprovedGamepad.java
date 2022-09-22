package org.firstinspires.ftc.teamcode.Shared.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

@Disabled
@TeleOp(name = "Improved Gamepad Test", group = "Test")
public class TestImprovedGamepad extends OpMode {

    public ElapsedTime timer = new ElapsedTime();
    public ImprovedGamepad improvedGamepad1;
    public ImprovedGamepad improvedGamepad2;

    @Override
    public void init() {
        this.improvedGamepad1 = new ImprovedGamepad(gamepad1, timer, "g1");
        this.improvedGamepad2 = new ImprovedGamepad(gamepad2, timer, "g2");
    }

    @Override
    public void loop() {
        improvedGamepad1.update();
        improvedGamepad2.update();

        telemetry.addData("Pressed", improvedGamepad1.left_stick_y.isPressed());
        telemetry.addData("Pressed Count", improvedGamepad1.left_stick.y.getPressedCounts());
        telemetry.addData("Pressed Time", improvedGamepad1.left_stick.y.getPressedElapseTime());

        telemetry.addData("Raw x", improvedGamepad1.left_stick.x.getRawValue());
        telemetry.addData("Raw y", improvedGamepad1.left_stick.y.getRawValue());
        telemetry.addData("x", improvedGamepad1.left_stick.x.getValue());
        telemetry.addData("y", improvedGamepad1.left_stick.y.getValue());

        telemetry.addData("Raw Radius", improvedGamepad1.left_stick.getRawValue());
        telemetry.addData("Radius", improvedGamepad1.left_stick.getValue());
        telemetry.addData("Raw Angle", improvedGamepad1.left_stick.getRawAngle());
        telemetry.addData("Angle", improvedGamepad1.left_stick.getAngel());

        telemetry.update();
    }
}
