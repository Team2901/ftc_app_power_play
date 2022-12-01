package org.firstinspires.ftc.teamcode.PowerPlay2901.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PowerPlay2901.Hardware.EarlyDiffyHardware;

@TeleOp(name = "Early Swerve PID Tuner")
public class EarlyDiffyPIDTuner extends OpMode {
    EarlyDiffyHardware robot = new EarlyDiffyHardware();

    private ElapsedTime runtime = new ElapsedTime();
    double targetAngle = 0;
    double podAngle = 0;
    boolean joystickControl = false;
    double pAngle = 0;
    double iAngle = 0;
    double dAngle = 0;
    double kp = 1.2;
    double ki = 0;
    double kd = 0;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        podAngle = (robot.leftOne.getCurrentPosition() - robot.leftTwo.getCurrentPosition())/8.95;
        //podAngle = AngleUnit.normalizeDegrees(encoderDifference);

        if(gamepad2.dpad_up || gamepad1.dpad_up){
            targetAngle = 0;
        }
        if(gamepad2.dpad_down || gamepad1.dpad_down){
            targetAngle = 180;
        }
        if(gamepad2.dpad_left || gamepad1.dpad_left){
            targetAngle = -90;
        }
        if(gamepad2.dpad_right || gamepad1.dpad_right){
            targetAngle = 90;
        }

        if(gamepad1.start){
            joystickControl = true;
        }
        if(gamepad1.back){
            joystickControl = false;
        }
        if(joystickControl){
            targetAngle = Math.toDegrees(Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y+.001));
        } else {
            targetAngle += gamepad1.left_stick_x * 2;
        }

        double pidOutput = pidPower();
        robot.leftOne.setPower((gamepad1.right_stick_y+pidOutput));
        robot.leftTwo.setPower((gamepad1.right_stick_y-pidOutput));

        if(gamepad1.y){
            kp += .01;
        } else if(gamepad1.a){
            kp -= .01;
        }
        telemetry.addData("P Gain", kp);
        if(gamepad1.x){
            ki += .01;
        } else if(gamepad1.b){
            ki -= .01;
        }
        telemetry.addData("I Gain", ki);
        if(gamepad1.right_bumper){
            kd += .01;
        } else if(gamepad1.left_bumper){
            kd -= .01;
        }
        telemetry.addData("D Gain", kd);
        telemetry.addData("Target Position", targetAngle);
        telemetry.addData("Position", podAngle);
        telemetry.addData("Power", pidOutput);
        telemetry.update();
    }

    public double pidPower(){
        double error = AngleUnit.normalizeDegrees(targetAngle - podAngle);
        telemetry.addData("Error", error);
        double secs = runtime.seconds();
        runtime.reset();
        dAngle = (error - pAngle) / secs;
        iAngle = iAngle + (error * secs);
        pAngle = error;
        double total = (kp* pAngle + ki* iAngle + kd* dAngle)/100;
        if(total > 1){
            iAngle = 0;
            total = 1;
        }
        if(total < -1){
            iAngle = 0;
            total = -1;
        }
        return total;
    }
}
