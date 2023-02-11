package org.firstinspires.ftc.teamcode.Outreach.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Outreach.Hardware.NewClawbotHardware;

@Disabled
@TeleOp(name="Clawbot Voltage tester", group="Outreach")
public class ClawbotVoltageTester extends OpMode {

    NewClawbotHardware robot = new NewClawbotHardware();
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        telemetryStuff();
    }

    public void telemetryStuff() {
        telemetry.addData("Voltage", robot.potentiometer.getVoltage());
        telemetry.update();
    }
}
