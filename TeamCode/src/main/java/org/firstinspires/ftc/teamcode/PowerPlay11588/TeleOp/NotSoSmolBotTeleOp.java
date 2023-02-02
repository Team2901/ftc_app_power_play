package org.firstinspires.ftc.teamcode.PowerPlay11588.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.NotSoSmolBotHardware;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

@TeleOp (name = "NotSoSmolBot TeleOp", group = "11588")
public class NotSoSmolBotTeleOp extends OpMode {
    NotSoSmolBotHardware robot = new NotSoSmolBotHardware();

    ElapsedTime gamepadTimer = new ElapsedTime();
    public ImprovedGamepad impGamepad1;
    public ImprovedGamepad impGamepad2;
    double turningPower = 0;
    @Override
    public void init() {
        robot.init(hardwareMap, false);
        impGamepad1 = new ImprovedGamepad(gamepad1, gamepadTimer, "g1");
        impGamepad2 = new ImprovedGamepad(gamepad2, gamepadTimer, "g2");
    }

    @Override
    public void loop() {
        impGamepad1.update();
        impGamepad2.update();
        if(impGamepad1.right_trigger.getValue() > 0){
            turningPower = .3 * impGamepad1.right_trigger.getValue();
        }else if(impGamepad1.left_trigger.getValue() > 0){
            turningPower = -.3 * impGamepad1.left_trigger.getValue();
        }else{
            turningPower = .75 * impGamepad1.right_stick_x.getValue();
            //turningPower = impGamepad1.right_stick_x.getValue();
        }
        double y = .75 * impGamepad1.left_stick_y.getValue();
        double x = .75 * impGamepad1.left_stick_x.getValue();
        //double y = impGamepad1.left_stick_y.getValue();
        //double x = impGamepad1.left_stick_x.getValue();
        double rx = turningPower;

        robot.frontLeft.setPower(y + x + rx);
        robot.frontRight.setPower(y - x - rx);
        robot.backLeft.setPower(y - x + rx);
        robot.backRight.setPower(y + x - rx);

        telemetry.addData("Front Left Power", robot.frontLeft.getPower());
        telemetry.addData("Front Right Power", robot.frontRight.getPower());
        telemetry.addData("Back Left Power", robot.backLeft.getPower());
        telemetry.addData("Back Right Power", robot.backRight.getPower());
        telemetry.addData("Front Left Position", robot.frontLeft.getCurrentPosition());
        telemetry.addData("Front Right Position", robot.frontRight.getCurrentPosition());
        telemetry.addData("Back Left Position", robot.backLeft.getCurrentPosition());
        telemetry.addData("Back Right Position", robot.backRight.getCurrentPosition());
        telemetry.addData("Robot Angle", robot.getAngle());
        telemetry.update();
    }
}
