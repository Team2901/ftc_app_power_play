package org.firstinspires.ftc.teamcode.Outreach.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Outreach.Hardware.NewClawbotHardware;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

@TeleOp(name="New Clawbot Teleop", group="Outreach")
public class NewClawbotTeleOp extends OpMode {
    public enum ClawState {OPEN, CLOSED}
    public enum Controller{PARTICIPANT, MASTER}
    public ClawState currentClawState = ClawState.CLOSED;
    public ImprovedGamepad gamepad;
    public ImprovedGamepad masterGamepad;
    public ImprovedGamepad gamepadInControl;
    public ElapsedTime gamepadTimer = new ElapsedTime();

    public boolean gamepadOverride = false;
    public double armAngle = 0.0;
    public double zeroDegreeVoltage = 2.891;
    public double nintyDegreeVoltage = 1.347;
    public double cosArm = 0.0;
    public double kCos = 0.25;
    public double pidArm = 0.0;

    NewClawbotHardware robot = new NewClawbotHardware();
    @Override
    public void init() {
        gamepad = new ImprovedGamepad(gamepad1, gamepadTimer, "gamepad");
        masterGamepad = new ImprovedGamepad(gamepad2, gamepadTimer, "masterGamepad");
        gamepadInControl = null;
        robot.init(hardwareMap);
        telemetry.addData("Override", gamepadOverride);
        telemetry.addData("Gamepad In Control", gamepadInControl);
        telemetry.addData("Claw State", currentClawState);
        telemetry.addData("Arm Angle", armAngle);
        telemetry.addData("Arm Power", robot.arm.getPower());
        telemetry.addData("Arm Voltage", robot.potentiometer.getVoltage());
        telemetry.update();
    }

    @Override
    public void loop() {
        gamepad.update();
        masterGamepad.update();

        if (masterGamepad.x.isInitialPress() && gamepadOverride) {
            gamepadOverride = false;
        } else if (masterGamepad.x.isInitialPress() && !gamepadOverride) {
            gamepadOverride = true;
        }

        if (gamepadOverride || masterGamepad.areButtonsActive()) {
            gamepadInControl = masterGamepad;
        } else {
            gamepadInControl = gamepad;
        }

        robot.leftDrive.setPower(-gamepadInControl.left_stick_y.getValue());
        robot.rightDrive.setPower(-gamepadInControl.right_stick_y.getValue());

        /*
        if(gamepadInControl.b.isInitialPress()){
            switch (currentClawState){
                case OPEN:
                    currentClawState = c
            }
        }

         */

        switch (currentClawState) {
            case OPEN:
                robot.claw.setPosition(NewClawbotHardware.CLAW_OPEN_POSITION);
                if (gamepadInControl.b.isInitialPress()) {
                    currentClawState = ClawState.CLOSED;
                }
                break;
            case CLOSED:
                robot.claw.setPosition(NewClawbotHardware.CLAW_CLOSED_POSITION);
                if (gamepadInControl.b.isInitialPress()) {
                    currentClawState = ClawState.OPEN;
                }
                break;
        }

        /*
        if(gamepadInControl.right_bumper.isInitialPress()){
            kCos += .01;
        }else if(gamepadInControl.left_bumper.isInitialPress()){
            kCos -= .01;
        }
         */

        armAngle = (90 / (nintyDegreeVoltage - zeroDegreeVoltage) * (robot.potentiometer.getVoltage() - zeroDegreeVoltage));
        cosArm = Math.cos(Math.toRadians(armAngle));
        pidArm = cosArm * kCos;

        if (armAngle > 90) {
            robot.arm.setPower(-.2);
        }else if(armAngle < -20 && (robot.leftDrive.getPower() != 0 || robot.rightDrive.getPower() != 0)){
            robot.arm.setPower(.2);
        }else if(gamepadInControl.dpad_up.isPressed() || gamepadInControl.y.isPressed()){
            robot.arm.setPower(pidArm + .4);
        }else if(gamepadInControl.dpad_down.isPressed() || gamepadInControl.a.isPressed()){
            robot.arm.setPower(pidArm - .6);
        }else{
            robot.arm.setPower(pidArm);
        }



        //robot.arm.setPower(pidArm);

        telemetry.addData("Override", gamepadOverride);
        telemetry.addData("Gamepad In Control", gamepadInControl);
        telemetry.addData("Claw State", currentClawState);
        telemetry.addData("K Cos", kCos);
        telemetry.addData("Arm Angle", armAngle);
        telemetry.addData("Arm Power", robot.arm.getPower());
        telemetry.addData("Arm Voltage", robot.potentiometer.getVoltage());
        telemetry.update();

    }
}