package org.firstinspires.ftc.teamcode.NewProgrammers.Y2023;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Jenna L FirstTeleOp")
public class JennaLFirstTeleOp extends OpMode {
    public DcMotor leftDrive;
    public DcMotor rightDrive;

    Servo servo;
    TouchSensor touchSensor;
    DistanceSensor distanceSensor;

    @Override
    public void init() {
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        servo = hardwareMap.get(Servo.class, "servo");
        touchSensor= hardwareMap.get(TouchSensor.class, "touch sensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance sensor");
    }

    @Override
    public void loop() {
        leftDrive.setPower(-gamepad1.left_stick_y);
        rightDrive.setPower(-gamepad1.right_stick_y);

        if(gamepad1.a){
            servo.setPosition(1);
        } else if(gamepad1.y){
            servo.setPosition(0);
        } else if(gamepad1.b){
            servo.setPosition(.5);
        }


        telemetry.addData("Serve Position", servo.getPosition());
        telemetry.addData("Is Touching",touchSensor.isPressed());
        telemetry.addData("Distance", distanceSensor.getDistance(DistanceUnit.MM));

    }
}
