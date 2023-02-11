package org.firstinspires.ftc.teamcode.Outreach.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Outreach.Hardware.NewClawbotHardware;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

@Disabled
@TeleOp(name="New Control Clawbot Teleop", group="Outreach")
public class NewControlClawbotTeleOp extends OpMode {
    public enum ClawState {OPEN, CLOSED}
    public enum Controller{PARTICIPANT, MASTER}
    public NewClawbotTeleOp.ClawState currentClawState = NewClawbotTeleOp.ClawState.CLOSED;
    public ImprovedGamepad gamepad;
    public ImprovedGamepad masterGamepad;
    public ImprovedGamepad gamepadInControl;
    public ElapsedTime gamepadTimer = new ElapsedTime();

    public boolean gamepadOverride = false;
    public double armAngle = 0.0;
    public double zeroDegreeVoltage = 1.347;
    public double nintyDegreeVoltage = 2.737;
    public double cosArm = 0.0;
    public double kCos = 0.1;
    public double pidArm = 0.0;

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

        robot.leftDrive.setPower((-gamepadInControl.right_stick_y.getValue() + -gamepadInControl.left_stick_y.getValue()) / 4 +
                (-gamepadInControl.right_stick_x.getValue()));
        robot.rightDrive.setPower((-gamepadInControl.right_stick_y.getValue() + -gamepadInControl.left_stick_y.getValue()) / 4 +
                (gamepadInControl.left_stick_x.getValue()));


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

        armAngle = (90/(nintyDegreeVoltage - zeroDegreeVoltage) * robot.potentiometer.getVoltage() - zeroDegreeVoltage);
        cosArm = Math.cos(Math.toRadians(armAngle));
        pidArm = cosArm * kCos;
        if(gamepadInControl.dpad_up.isPressed() || gamepadInControl.y.isPressed()){
            robot.arm.setPower(pidArm + .5);
        }else if(gamepadInControl.dpad_down.isPressed() || gamepadInControl.a.isPressed()){
            robot.arm.setPower(pidArm - .5);
        }else{
            robot.arm.setPower(pidArm);
        }


        telemetry.addData("Override", gamepadOverride);
        telemetry.addData("Gamepad In Control", gamepadInControl);
        telemetry.addData("Claw State", currentClawState);
        telemetry.update();

    }
}
