package org.firstinspires.ftc.teamcode.Shared.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Arm Position Tester", group = "Test")
public class ArmPositionTester extends OpMode {
    DcMotorEx tuneMotor;

    @Override
    public void init() {
        tuneMotor = hardwareMap.get(DcMotorEx.class, "tune motor");
        tuneMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tuneMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tuneMotor.setPower(0);
        telemetry.addData("Motor Position", tuneMotor.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("Motor Position", tuneMotor.getCurrentPosition());
        telemetry.update();
    }
}
