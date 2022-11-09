package org.firstinspires.ftc.teamcode.PowerPlay11588.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.Qual11588Hardware;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

public class Qual11588TeleOp extends OpMode {
    Qual11588Hardware robot = new Qual11588Hardware();
    public ElapsedTime gamepadTimer = new ElapsedTime();
    public ImprovedGamepad impGamepad1;
    public ImprovedGamepad impGamepad2;
    double turningPower = 0;

    @Override
    public void init() {
        robot.init(hardwareMap);
        impGamepad1 = new ImprovedGamepad(gamepad1, gamepadTimer, "g1");
        impGamepad2 = new ImprovedGamepad(gamepad2, gamepadTimer, "g2");
    }

    @Override
    public void loop() {
        if(impGamepad1.right_trigger.getValue() > 0){
            turningPower = .3 * impGamepad1.right_trigger.getValue();
        }else if(impGamepad1.left_trigger.getValue() > 0){
            turningPower = -.3 * impGamepad1.left_trigger.getValue();
        }else{
            turningPower = impGamepad1.right_stick_x.getValue();
        }
        double y = -.5 * impGamepad1.left_stick_y.getValue();
        double x = .5 * impGamepad1.left_stick_x.getValue();
        double rx = turningPower;

        robot.frontLeft.setPower(y + x + rx);
        robot.frontRight.setPower(y - x - rx);
        robot.backLeft.setPower(y - x + rx);
        robot.backRight.setPower(y + x - rx);
    }
}
