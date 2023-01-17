package org.firstinspires.ftc.teamcode.PowerPlay2901.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PowerPlay2901.Hardware.RockBotHardware;

@TeleOp(name = "angles are dumb")
public class angleStupidity extends OpMode {
    RockBotHardware robot = new RockBotHardware();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("result angle", Math.toDegrees(Math.atan2(-x, y + .001)));
        telemetry.addData("robot angle", robot.getAngle());
    }
}
