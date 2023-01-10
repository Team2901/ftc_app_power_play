package org.firstinspires.ftc.teamcode.Outreach.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Outreach.Hardware.ClawbotHardware;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.Utility.CountDownTimer;

/**
 * Created by Kearneyg20428 on 2/7/2017.
 */
@TeleOp(name = "Clawbot", group = "Outreach")
public class ClawbotTeleOp extends OpMode {

    final double CLAW_SPEED = 0.05;
    final ClawbotHardware robot = new ClawbotHardware();
    double clawOffset = 0.0;

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.claw.setPosition(ClawbotHardware.MID_SERVO);
    }

    @Override
    public void loop() {
        boolean overrideActive = process(gamepad2);
        if (!overrideActive){
            process(gamepad1);
        }
        // Send telemetry message to signify robot running;
        telemetry.addData("claw", "Offset = %.2f", clawOffset);
        telemetry.addData("claw", "Absolute = %.2f", ClawbotHardware.MID_SERVO - clawOffset);
        telemetry.addData("arm pos", "%d", robot.armMotor.getCurrentPosition());
        telemetry.addData("arm speed", "%.2f", robot.armMotor.getPower());
        telemetry.addData("left joystick position", "%.2f", -gamepad1.left_stick_y);
        telemetry.update();
    }
    public boolean process(Gamepad gamepad){
        double left;
        double right;
        boolean inUse = false;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        if (gamepad.dpad_up) {
            inUse = true;
            power(.75, .75);
        } else if (gamepad.dpad_down) {
            inUse = true;
            power(-.75, -.75);
        } else if (gamepad.dpad_left) {
            inUse = true;
            power(-.75, .75);
        } else if (gamepad.dpad_right) {
            inUse = true;
            power(.75, -.75);
        } else {
            left = gamepad.left_stick_y;
            right = gamepad.right_stick_y;
            robot.leftMotor.setPower(left);
            robot.rightMotor.setPower(right);
        }
        // Use gamepad left & right Bumpers to open and close the claw
        if (gamepad.right_bumper) {
            inUse = true;
            clawOffset += CLAW_SPEED;
        } else if (gamepad.left_bumper) {
            inUse = true;
            clawOffset -= CLAW_SPEED;
        }

        // Do not let the servo position go too wide or the plastic gears will come unhinged.
        clawOffset = Range.clip(clawOffset, ClawbotHardware.MIN_SAFE_CLAW_OFFSET, ClawbotHardware.MAX_SAFE_CLAW_OFFSET);

        // Move both servos to new position.
        robot.claw.setPosition(ClawbotHardware.MID_SERVO - clawOffset);

        // Use gamepad buttons to move the arm up (Y) and down (A)
        if (gamepad.y) {
            inUse = true;
            robot.armMotor.setPower(ClawbotHardware.ARM_UP_POWER);
        } else if (gamepad.a) {
            inUse = true;
            robot.armMotor.setPower(ClawbotHardware.ARM_DOWN_POWER);
        } else {
            inUse = false;
            robot.armMotor.setPower(0.0);
        }
        return inUse;
    }
    public void power(double left, double right) {
        robot.leftMotor.setPower(left);
        robot.rightMotor.setPower(right);
    }
}
