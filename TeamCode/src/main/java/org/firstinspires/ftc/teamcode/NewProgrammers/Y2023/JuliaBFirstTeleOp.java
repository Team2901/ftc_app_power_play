package org.firstinspires.ftc.teamcode.NewProgrammers.Y2023;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name= "JuliaBFirstTeleOp", group= "Shared")
public class JuliaBFirstTeleOp extends OpMode {

    /*public DcMotor backLeftDrive;
    public DcMotor backRightDrive;
    public DcMotor frontLeftDrive;
    public DcMotor frontRightDrive;*/

    //Defining the Servo
    public Servo Servo;
    //Defining touch sensor
   // public TouchSensor touchSensor;


    @Override
    public void init() {
        /*backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");*/

        Servo = hardwareMap.get(Servo.class, "servo");


        /*backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);*/
    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            Servo.setPosition(0);
        }
        if(gamepad1.b){
            Servo.setPosition(1);
        }
        /*else if (gamepad1.y){
            Servo.setPosition(1);
        }
        backLeftDrive.setPower(gamepad1.left_stick_y);
        backRightDrive.setPower(gamepad1.right_stick_y);

        frontLeftDrive.setPower(gamepad1.right_stick_y);
        frontRightDrive.setPower(gamepad1.right_stick_y);

        telemetry.addData("Is touching", touchSensor.isPressed());
        telemetry.update();
    }*/
    }
}
