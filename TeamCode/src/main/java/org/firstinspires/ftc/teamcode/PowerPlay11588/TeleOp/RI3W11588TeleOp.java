package org.firstinspires.ftc.teamcode.PowerPlay11588.TeleOp;

import static org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.RI3W11588Hardware.FRONT_GEAR_RATIO;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.RI3W11588Hardware;

@TeleOp(name = "3 Week 11588", group = "11588")
public class RI3W11588TeleOp extends OpMode {
    RI3W11588Hardware robot = new RI3W11588Hardware();

    double turningPower = 0;
    enum ClawPosition{Open, Closed}
    ClawPosition currentClawPosition = ClawPosition.Closed;

    //I can't decide which one to use, imp gamepad or checking ourselves
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    int armTarget = 100;
    int lastTarget = armTarget;
    double total = 0.0;
    double pArm = 0.0;
    double iArm = 0.0;
    double dArm = 0.0;
    double kp = 0.5;
    double ki = 0.5;
    double kd = 0.01;
    double iArmMax = .25;

    enum Height{
        INTAKE,
        GROUND,
        LOW,
        MEDIUM,
        HIGH
    }

    ElapsedTime pidTimer = new ElapsedTime();

    @Override
    public void init() {
        robot.init(this.hardwareMap, telemetry);
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
            turningPower = 0.3 * gamepad1.right_trigger;
        }else if(gamepad1.left_trigger > 0){
            turningPower = -.3 * gamepad1.left_trigger;
        }else{
            turningPower = gamepad1.right_stick_x;
        }
        double y = -.5 * gamepad1.left_stick_y;
        double x = .5 * gamepad1.left_stick_x;
        //double rx = gamepad1.right_stick_x;
        double rx = turningPower;


        robot.frontLeft.setPower(y + x + rx);
        robot.frontRight.setPower(y - x - rx);
        robot.backLeft.setPower((y - x + rx)*FRONT_GEAR_RATIO);
        robot.backRight.setPower((y + x - rx)*FRONT_GEAR_RATIO);

        if(gamepad1.dpad_left){
            armTarget = 100;
        }
        if(gamepad1.dpad_down){
            armTarget = 600;
        }
        if(gamepad1.dpad_right){
            armTarget = 950;
        }
        if(gamepad1.dpad_up){
            armTarget = 1000;
        }
        if(currentGamepad1.a && !previousGamepad1.a){
            armTarget = armTarget - 10;
        }
        if(currentGamepad1.y && !previousGamepad1.y){
            armTarget = armTarget + 10;
        }

        robot.arm.setPower(armPower());
        //armPowerer();

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
        if(currentGamepad1.x && !currentGamepad1.x){
            robot.claw.setPosition(0);
            currentClawPosition = ClawPosition.Closed;
        }

        telemetry.addData("Front Left Position", robot.frontLeft.getCurrentPosition());
        telemetry.addData("Front Right Position", robot.frontRight.getCurrentPosition());
        telemetry.addData("Back Left Position", robot.backLeft.getCurrentPosition());
        telemetry.addData("Back Right Position", robot.backRight.getCurrentPosition());
        telemetry.addData("Arm Position", robot.arm.getCurrentPosition());
        telemetry.addData("Arm Target", armTarget);
        telemetry.addData("P Arm", pArm);
        telemetry.addData("I Arm", iArm);
        telemetry.addData("D Arm", dArm);
        telemetry.addData("Proportional Stuff", pArm * kp);
        telemetry.addData("Integral Stuff", iArm * ki);
        telemetry.addData("Derivative Stuff", dArm * kd);
        telemetry.addData("Pid Total", total);
        telemetry.addData("Claw State", currentClawPosition);
//        telemetry.addData("Blue", robot.pipeLine.blueAmountAverage);
//        telemetry.addData("Green", robot.pipeLine.greenAmountAverage);
//        telemetry.addData("red", robot.pipeLine.redAmountAverage);
        telemetry.update();

        /*
        telemetry.addData("frp: ", "%f", y - x - rx);
        telemetry.addData("fr: ", "%d", robot.frontRight.getCurrentPosition());
        telemetry.addData("brp: ", "%f", y + x - rx);
        telemetry.addData("br: ", "%d",  robot.backRight.getCurrentPosition());
         */

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
    }

    public double armPower(){
        double error = armTarget - robot.arm.getCurrentPosition();
        dArm = (error - pArm) / pidTimer.seconds();
        iArm = iArm + (error * pidTimer.seconds());
        pArm = error;

        total = ((kp * pArm) + (ki * iArm) + (kd * dArm))/100;

        if(total > .5) {
            total = .5;
        }

        if(armTarget != lastTarget){
            iArm = 0;
        }
        if(iArm > iArmMax){
            iArm = iArmMax;
        }else if(iArm < -iArmMax){
            iArm = -iArmMax;
        }

        lastTarget = armTarget;

        pidTimer.reset();
        return total;
    }
}
