package org.firstinspires.ftc.teamcode.Shared.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Voltage PID Tuner", group = "Test")
public class VoltagePIDTuner extends OpMode {
    public DcMotorEx tuneMotor;
    public AnalogInput potentiometer;

    private ElapsedTime runtime = new ElapsedTime();
    double cosArm = 0;
    double kCos = 0;
    double armAngle = 0;
    public double zeroDegreeVoltage = 2.737;
    public double nintyDegreeVoltage = 1.347;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void init() {
        tuneMotor = hardwareMap.get(DcMotorEx.class, "tune motor");
        potentiometer = hardwareMap.analogInput.get("potentiometer");
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
        if(gamepad1.right_bumper && !previousGamepad1.right_bumper){
            kCos += .01;
        }else if(gamepad1.left_bumper && !previousGamepad1.left_bumper){
            kCos -= .01;
        }

        tuneMotor.setPower(feedForwardPower());
        telemetry.addData("Cos Gain", kCos);
        telemetry.addData("Arm Angle:", armAngle);
        telemetry.addData("Motor Power", tuneMotor.getPower());
        telemetry.addData("Total", feedForwardPower());
        telemetry.addData("cosArm", cosArm);
        telemetry.addData("Cos Value", cosArm * kCos);
        telemetry.update();
    }

    public double feedForwardPower(){
        /*
        Find the angle based on measuring the encoder value at 0 and 90
        plug it into desmos how it is modeled using a linear regression
        0 degrees = 425 90 degrees = 1300
         */
        armAngle = (90/(nintyDegreeVoltage - zeroDegreeVoltage) * (potentiometer.getVoltage() - zeroDegreeVoltage));
        cosArm = Math.cos(Math.toRadians(armAngle));
        double ffTotal = (kCos * cosArm);
        return ffTotal;
    }
}
