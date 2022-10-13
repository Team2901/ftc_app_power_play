package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Hardware Tester", group = "Test")
public class HardwareTesterTeleOp extends OpMode {

    public Servo clawOne;
    public Servo clawTwo;

    @Override
    public void init() {
        clawOne = hardwareMap.get(Servo.class, "claw 1");
        clawTwo = hardwareMap.get(Servo.class, "claw 2");
    }

    @Override
    public void loop() {
        clawOne.setPosition(0);
        clawTwo.setPosition(1);
    }
}
