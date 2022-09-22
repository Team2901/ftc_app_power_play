package org.firstinspires.ftc.teamcode.Shared.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "PID Tuner", group = "Test")
public class PIDTunerTeleOp extends OpMode {
    public DcMotorEx tuneMotor;

    private ElapsedTime runtime = new ElapsedTime();
    double targetPosition = 0;
    double pArm = 0;
    double iArm = 0;
    double dArm = 0;
    double kp = 0.5;
    double ki = 0;
    double kd = 0;

    @Override
    public void init() {
        tuneMotor = hardwareMap.get(DcMotorEx.class, "tune motor");
        tuneMotor.setPower(0);
    }

    @Override
    public void loop() {
        if(gamepad2.dpad_up || gamepad1.dpad_right){
            targetPosition = 500;
        }
        if(gamepad2.dpad_down || gamepad1.dpad_up){
            targetPosition = 0;
        }
        if(gamepad2.dpad_left || gamepad1.dpad_down){
            targetPosition = -200;
        }
        if(gamepad2.dpad_right || gamepad1.dpad_left){
            targetPosition = 200;
        }
        tuneMotor.setPower(pidPower());
        targetPosition += gamepad1.right_stick_y*10;
        if(gamepad1.right_bumper){
            targetPosition = gamepad1.left_stick_x*500 + 700;
        }

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
        if(gamepad1.start){
            kd += .01;
        } else if(gamepad1.back){
            kd -= .01;
        }
        telemetry.addData("D Gain", kd);
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Position", tuneMotor.getCurrentPosition());
    }

    public double pidPower(){
        double error = (targetPosition-tuneMotor.getCurrentPosition());
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
