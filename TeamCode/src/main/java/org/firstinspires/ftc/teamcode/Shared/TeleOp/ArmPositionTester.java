package org.firstinspires.ftc.teamcode.Shared.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Arm Position Tester", group = "Test")
public class ArmPositionTester extends OpMode {
    DcMotorEx tuneMotor;

    @Override
    public void init() {
        tuneMotor = hardwareMap.get(DcMotorEx.class, "tune motor");
        //tuneMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tuneMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //tuneMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        tuneMotor.setPower(0);
        telemetry.addData("Motor Position", tuneMotor.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void loop() {
        if(gamepad1.right_trigger > .5){
            tuneMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            tuneMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        telemetry.addData("Motor Position", tuneMotor.getCurrentPosition());
        telemetry.update();
    }
}
