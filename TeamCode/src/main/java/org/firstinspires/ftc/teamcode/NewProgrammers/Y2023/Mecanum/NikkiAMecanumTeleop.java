package org.firstinspires.ftc.teamcode.NewProgrammers.Y2023.Mecanum;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class NikkiAMecanumTeleop extends OpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        //Takes in the left stick y value. Gamepad y values are reversed
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        //Takes the right stick x value for turning
        double rx = gamepad1.right_stick_x;

        /*
        y value, and then add the direction the wheel turns if you want to strafe right
        Add x if the wheel goes forwards, subtract if wheel goes backwards
        for rotating same thing but for turning right
         */
        frontLeft.setPower(y + x + rx);
        frontRight.setPower(y - x - rx);
        backLeft.setPower(y - x + rx);
        backRight.setPower(y + x - rx);
    }
}
