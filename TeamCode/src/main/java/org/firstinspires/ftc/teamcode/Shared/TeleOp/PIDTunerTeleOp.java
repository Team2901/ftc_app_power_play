package org.firstinspires.ftc.teamcode.Shared.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "PID Tuner", group = "Test")
public class PIDTunerTeleOp extends OpMode {
    public DcMotorEx tuneMotor;

    private ElapsedTime runtime = new ElapsedTime();
    double targetPosition = 0;
    double lastTarget = 0;
    double pArm = 0;
    double iArm = 0;
    double dArm = 0;
    double cosArm = 0;
    double kp = 0.5;
    double ki = 0;
    double kd = 0;
    double kCos = 0;
    double armAngle = 0;
    double iArmMax = .5;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void init() {
        tuneMotor = hardwareMap.get(DcMotorEx.class, "tune motor");
        tuneMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tuneMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tuneMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //tuneMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        tuneMotor.setPower(0);
    }

    @Override
    public void loop() {
        try {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
        } catch (RobotCoreException e) {
            e.printStackTrace();
        }

        if(gamepad2.dpad_up || gamepad1.dpad_right){
            targetPosition = 800;
        }
        if(gamepad2.dpad_down || gamepad1.dpad_up){
            targetPosition = 0;
        }
        if(gamepad2.dpad_left || gamepad1.dpad_down){
            targetPosition = 200;
        }
        if(gamepad2.dpad_right || gamepad1.dpad_left){
            targetPosition = 500;
        }

        if(gamepad1.left_trigger > .5){
            tuneMotor.setPower(feedForwardPower());
        }else{
            tuneMotor.setPower(pidPower());
        }

        targetPosition += gamepad1.right_stick_y*10;


        if(gamepad1.right_trigger > .5){
            tuneMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }


        if(gamepad1.y && !previousGamepad1.y){
            kp += .01;
        } else if(gamepad1.a && !previousGamepad1.a){
            kp -= .01;
        }
        telemetry.addData("P Gain", kp);
        if(gamepad1.x && !previousGamepad1.x){
            ki += .01;
        } else if(gamepad1.b && !previousGamepad1.b){
            ki -= .01;
        }
        telemetry.addData("I Gain", ki);
        if(gamepad1.start && !previousGamepad1.start){
            kd += .01;
        } else if(gamepad1.back && !previousGamepad1.back){
            kd -= .01;
        }
        telemetry.addData("D Gain", kd);
        if(gamepad1.right_bumper && !previousGamepad1.right_bumper){
            kCos += .01;
        }else if(gamepad1.left_bumper && !previousGamepad1.left_bumper){
            kCos -= .01;
        }
        telemetry.addData("Cos Gain", kCos);
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Arm Angle:", armAngle);
        telemetry.addData("Motor Power", tuneMotor.getPower());
        telemetry.addData("Total", pidPower());
        telemetry.addData("pArm", pArm);
        telemetry.addData("P Value", pArm * kp);
        telemetry.addData("iArm", iArm);
        telemetry.addData("I Value", iArm * ki);
        telemetry.addData("dArm", dArm);
        telemetry.addData("D Value", dArm * kd);
        telemetry.addData("cosArm", cosArm);
        telemetry.addData("Cos Value", cosArm * kCos);
        telemetry.addData("Position", tuneMotor.getCurrentPosition());
    }

    public double feedForwardPower(){
        /*
        Find the angle based on measuring the encoder value at 0 and 90
        plug it into desmos how it is modeled using a linear regression
        0 degrees = 425 90 degrees = 1300
         */
        armAngle = (90.0/(1200.0 - 400.0)) * (tuneMotor.getCurrentPosition() - 400.0);
        cosArm = Math.cos(Math.toRadians(armAngle));
        double ffTotal = (kCos * cosArm);
        return ffTotal;
    }

    public double pidPower(){
        double error = (targetPosition-tuneMotor.getCurrentPosition());
        double secs = runtime.seconds();
        runtime.reset();
        dArm = (error - pArm) / secs;
        iArm = iArm + (error * secs);
        pArm = error;
        armAngle = 0.102856 * tuneMotor.getCurrentPosition() - 43.6276;
        cosArm = Math.cos(Math.toRadians(armAngle));
        double total = (kp*pArm + ki*iArm + kd*dArm)/100 + (kCos * cosArm);
        if(total > .75){
            //iArm = 0;
            total = .75;
        }
        if(total < 0){
            //iArm = 0;
            total = 0;
        }

        if(targetPosition != lastTarget){
            iArm = 0;
        }

        if(iArm > iArmMax){
            iArm = iArmMax;
        }else if(iArm < -iArmMax){
            iArm = -iArmMax;
        }
        lastTarget = targetPosition;
        return total;
    }
}
