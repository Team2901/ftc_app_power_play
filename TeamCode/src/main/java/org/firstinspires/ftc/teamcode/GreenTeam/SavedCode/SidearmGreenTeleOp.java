package org.firstinspires.ftc.teamcode.GreenTeam.SavedCode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Sidearm Green TeleOp", group = "Green")
public class SidearmGreenTeleOp extends OpMode {
    SidearmGreenHardware robot = new SidearmGreenHardware();
    ElapsedTime runtime = new ElapsedTime();
    double spinnerSpeed = .9;
    int armTarget = 0;
    double pArm = 0;
    double iArm = 0;
    double dArm = 0;
    double kp = 0.7;
    double ki = 0.5;
    double kd = 0.05;
    int intakeSpot = 1875;
    boolean partyMode = false;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        double frontLeftPower = -gamepad1.left_stick_y;
        double frontRightPower = -gamepad1.left_stick_y;
        double backLeftPower = -gamepad1.left_stick_y;
        double backRightPower = -gamepad1.left_stick_y;

        frontLeftPower += gamepad1.left_stick_x;
        frontRightPower -= gamepad1.left_stick_x;
        backLeftPower -= gamepad1.left_stick_x;
        backRightPower += gamepad1.left_stick_x;

        robot.frontLeft.setPower(frontLeftPower + gamepad1.right_stick_x);
        robot.frontRight.setPower(frontRightPower - gamepad1.right_stick_x);
        robot.backLeft.setPower(backLeftPower + gamepad1.right_stick_x);
        robot.backRight.setPower(backRightPower - gamepad1.right_stick_x);

        robot.spinner.setPower(spinnerSpeed*gamepad2.left_stick_x);

        if(gamepad1.dpad_right || gamepad1.left_bumper){
            armTarget = 450;
        }
        if(gamepad1.dpad_up){
            armTarget = 900;
        }
        if(gamepad1.dpad_down || gamepad1.right_bumper){
            armTarget = intakeSpot;
        }
        if(gamepad1.dpad_left){
            armTarget = 1250;
        }
        if(gamepad2.x || gamepad1.x || (armTarget == intakeSpot && robot.backPlate.isPressed())){
            armTarget = intakeSpot-125;
        }
        if(gamepad1.y){
            armTarget = intakeSpot-225;
        }
        if(gamepad2.b || gamepad1.b){
            armTarget = 10;
        }

        if(gamepad1.start || gamepad2.start){
            partyMode = true;
        }
        if(gamepad1.back || gamepad2.back){
            partyMode = false;
        }

        if(gamepad2.dpad_right){
            robot.cap.setPosition(0);
        }
        if(gamepad2.dpad_up){
            robot.cap.setPosition(.35);
        }
        if(gamepad2.dpad_left){
            robot.cap.setPosition(.5);
        }
        if(gamepad2.dpad_down){
            robot.cap.setPosition(.7);
        }

        if(robot.backPlate.isPressed() && !partyMode){
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else if (!partyMode){
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
        } else {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
        }

        if(gamepad2.y){
            intakeSpot -= 1;
        }
        if(gamepad2.a){
            intakeSpot += 1;
        }

        robot.arm.setPower(armPower() + gamepad2.right_stick_y/3);

        robot.intakeMotor.setPower((gamepad1.right_trigger+gamepad2.right_trigger)
                -(gamepad1.left_trigger+gamepad2.left_trigger)/3);

        telemetry.addData("Arm Target", armTarget);
        telemetry.addData("Arm Position", robot.arm.getCurrentPosition());
        telemetry.addData("Intake Spot", intakeSpot);
        telemetry.addData("Backplate Pressed", robot.backPlate.isPressed());
    }

    public double armPower(){
        double error = (armTarget-robot.arm.getCurrentPosition());
        double secs = runtime.seconds();
        runtime.reset();
        dArm = (error - pArm) / secs;
        iArm = iArm + (error * secs);
        pArm = error;
        double total = (kp*pArm + ki*iArm + kd*dArm)/100;
        if(total > 1){
            iArm = 0;
            total = 1;
        }
        if(total < -1){
            iArm = 0;
            total = -1;
        }
        return total;
    }
}
