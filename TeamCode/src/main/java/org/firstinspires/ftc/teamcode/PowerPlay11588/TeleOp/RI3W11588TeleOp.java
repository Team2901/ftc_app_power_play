package org.firstinspires.ftc.teamcode.PowerPlay11588.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.OpenCVPipelines.RI3W11588OpenCV;
import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.RI3W11588Hardware;

@TeleOp(name = "3 Week 11588")
public class RI3W11588TeleOp extends OpMode {
    RI3W11588Hardware robot = new RI3W11588Hardware();

    double triggerValue = 0;
    enum ClawPosition{Open, Closed}
    ClawPosition currentClawPosition = ClawPosition.Open;

    //I can't decide which one to use, imp gamepad or checking ourselves
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void init() {
        robot.init(this.hardwareMap);
    }

    @Override
    public void loop() {
        try {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
        } catch (RobotCoreException e) {
            e.printStackTrace();
        }

        //I fixed turning I think, definately test
        if(gamepad1.right_trigger > 0){
            triggerValue = gamepad1.right_trigger;
        }else if(gamepad1.left_trigger > 0){
            triggerValue = -gamepad1.left_trigger;
        }else{
            triggerValue = 0;
        }
        double y = -.5 * gamepad1.left_stick_y;
        double x = .5 * gamepad1.left_stick_x;
        double rx = triggerValue;

        robot.frontLeft.setPower(y + x + rx);
        robot.frontRight.setPower(y - x - rx);
        robot.backLeft.setPower(y - x + rx);
        robot.backRight.setPower(y + x - rx);

        robot.arm.setPower(.3 * gamepad1.right_stick_y);

        telemetry.addData("frp: ", "%f", y - x - rx);
        telemetry.addData("fr: ", "%d", robot.frontRight.getCurrentPosition());
        telemetry.addData("brp: ", "%f", y + x - rx);
        telemetry.addData("br: ", "%d",  robot.backRight.getCurrentPosition());

        /*
        if(currentGamepad1.b && !previousGamepad1.b) {
            robot.claw.setPosition(1);
        }
        if(currentGamepad1.a && !previousGamepad1.a) {
            robot.claw.setPosition(.5);
        }
        if(currentGamepad1.y && !previousGamepad1.y) {
            robot.claw.setPosition(0);
        }
        */
        switch (currentClawPosition){
            case Open:
                if(currentGamepad1.b && !previousGamepad1.b){
                    robot.claw.setPosition(0);
                    currentClawPosition = ClawPosition.Closed;
                }
                break;
            case Closed:
                if(currentGamepad1.b && !previousGamepad1.b){
                    robot.claw.setPosition(.5);
                    currentClawPosition = ClawPosition.Open;
                }
        }
    }
}
