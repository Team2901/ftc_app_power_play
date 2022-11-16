package org.firstinspires.ftc.teamcode.PowerPlay11588.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.Qual11588Hardware;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

public class Qual11588TeleOp extends OpMode {
    Qual11588Hardware robot = new Qual11588Hardware();
    public ElapsedTime gamepadTimer = new ElapsedTime();
    public enum ClawPosition{Open, Closed};
    ClawPosition currentClawPosition = ClawPosition.Closed;
    public ImprovedGamepad impGamepad1;
    public ImprovedGamepad impGamepad2;
    double turningPower = 0;

    //All the variables that are needed for pid
    ElapsedTime PIDTimer = new ElapsedTime();
    int armTarget = 50;
    int lastTarget = armTarget;
    int modifier = 0;
    double error = 0;
    double total = 0;
    double kp = 0;
    double ki = 0;
    double kd = 0;
    double pArm = 0;
    double iArm = 0;
    double dArm = 0;
    double iArmMax = .25;


    @Override
    public void init() {
        robot.init(hardwareMap);
        impGamepad1 = new ImprovedGamepad(gamepad1, gamepadTimer, "g1");
        impGamepad2 = new ImprovedGamepad(gamepad2, gamepadTimer, "g2");
    }

    @Override
    public void loop() {
        if(impGamepad1.dpad_left.isInitialPress()){
            //Sets the armTarget to ground/intake
            armTarget = 50;
        }else if(impGamepad1.dpad_down.isInitialPress()){
            //Sets the armTarget to the low pole
            armTarget = 100;
        }else if(impGamepad1.dpad_right.isInitialPress()){
            //Sets the armTarget to the mid pole
            armTarget = 150;
        }else if(impGamepad1.dpad_up.isInitialPress()){
            //Sets the armTarget to the high pole
            armTarget = 200;
        }
        /*Allows for the armTarget to be changed for the duration of the TeleOp rather than resetting
        when you change height*/
        if(impGamepad1.y.isInitialPress()){
            modifier += 5;
        }
        if(impGamepad1.a.isInitialPress()){
            modifier -= 5;
        }
        armTarget += modifier;
        robot.arm.setPower(armPower(armTarget));

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

        switch (currentClawPosition){
            case Open:
                robot.claw.setPosition(.5);
                if(impGamepad1.b.isInitialPress()){
                    currentClawPosition = ClawPosition.Closed;
                }
                break;
            case Closed:
                robot.claw.setPosition(0);
                if(impGamepad1.b.isInitialPress()){
                    currentClawPosition = ClawPosition.Open;
                }
        }
    }

    public double armPower(int target){
        error = target - robot.arm.getCurrentPosition();
        dArm = (error - pArm) / PIDTimer.seconds();
        iArm = iArm + (error * PIDTimer.seconds());
        pArm = error;
        total = ((pArm * kp) + (iArm * ki) + (dArm * kd))/100;
        PIDTimer.reset();

        if(armTarget != lastTarget){
            iArm = 0;
        }

        if(iArm > iArmMax){
            iArm = iArmMax;
        }else if(iArm < -iArmMax){
            iArm = -iArmMax;
        }
        lastTarget = armTarget;

        return total;
    }
}
