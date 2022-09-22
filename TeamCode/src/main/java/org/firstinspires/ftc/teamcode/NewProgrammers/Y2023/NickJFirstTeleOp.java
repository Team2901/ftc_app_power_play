package org.firstinspires.ftc.teamcode.NewProgrammers.Y2023;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Nick Jerden First TeleOp", group = "Shared")
public class NickJFirstTeleOp<servo> extends OpMode {

    public DcMotor leftDrive;
    public DcMotor rightDrive;

    servo servo;
    TouchSensor touchSensor;
    DistanceSensor distanceSensor;

    @Override
    public void init() {
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        touchSensor = hardwareMap.get(TouchSensor.class, "touch sensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance sensor");
    }

    //init is a start up method

    @Override
    public void loop() {

        double currentDistance = distanceSensor.getDistance(DistanceUnit.MM);
        boolean isCloseToWall;
        boolean drivingIntoWall = false;

        if (currentDistance < 500) {
            isCloseToWall = true;
        }
        else {
            isCloseToWall = false;
        }

        if (gamepad1.left_stick_y < 0 || gamepad1.right_stick_y < 0) {
            if (isCloseToWall == true)
            {
                drivingIntoWall = true;
            }
        }

        if (drivingIntoWall == false) {
            leftDrive.setPower(gamepad1.left_stick_y);
            rightDrive.setPower(gamepad1.right_stick_y);
        }
        else{
            leftDrive.setPower(0.2);
            rightDrive.setPower(0.2);
        }

        telemetry.addData("Is Touching", touchSensor.isPressed());
        telemetry.addData("Distance", distanceSensor.getDistance(DistanceUnit.MM));
    }



}