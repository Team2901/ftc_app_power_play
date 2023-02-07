package org.firstinspires.ftc.teamcode.PowerPlay11588.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.Qual11588Hardware;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;
@TeleOp(name = "Qual 11588", group = "11588")
public class Qual11588TeleOp extends OpMode {
    Qual11588Hardware robot = new Qual11588Hardware();
    public ElapsedTime gamepadTimer = new ElapsedTime();
    public enum ClawPosition{Open, Closed}
    ClawPosition currentClawPosition = ClawPosition.Closed;
    public enum Height{GROUND, STACK, LOW, MID, HIGH}
    Height currentArmHeight = Height.GROUND;
    public ImprovedGamepad impGamepad1;
    public ImprovedGamepad impGamepad2;
    double turningPower = 0;

    //All the variables that are needed for pid
    ElapsedTime PIDTimer = new ElapsedTime();
    int armTarget = 200;

    //Making different variables for each target height
    Height lastArmHeight = currentArmHeight;
    int groundArmPosition = 200;
    int stackArmPosition = 350;
    int lowArmPosition = 550;
    int midArmPosition = 800;
    int highArmPosition = 1150;
    int zeroAngleTicks = lowArmPosition - 150;

    double error = 0.0;
    double total = 0.0;
    double kp = 0.9;
    double ki = 0.0;
    double kd = 0.0;
    double kCos = 0.3;
    double pArm = 0.0;
    double iArm = 0.0;
    double dArm = 0.0;
    double cosArm = 0.0;
    double iArmMax = .25;
    double armAngle = 0;

    @Override
    public void init() {
        robot.teleOpInit(hardwareMap, telemetry, false);
        impGamepad1 = new ImprovedGamepad(gamepad1, gamepadTimer, "g1");
        impGamepad2 = new ImprovedGamepad(gamepad2, gamepadTimer, "g2");
    }

    @Override
    public void loop() {
        impGamepad1.update();
        impGamepad2.update();
        //Drive Base Code
        if (impGamepad1.right_trigger.getValue() > 0) {
            turningPower = .3 * impGamepad1.right_trigger.getValue();
        } else if (impGamepad1.left_trigger.getValue() > 0) {
            turningPower = -.3 * impGamepad1.left_trigger.getValue();
        } else {
            turningPower = .75 * impGamepad1.right_stick_x.getValue();
        }
        double y = .75 * Math.cbrt(impGamepad1.left_stick_y.getValue());
        double x = .75 * Math.cbrt(impGamepad1.left_stick_x.getValue());
        double rx = turningPower;

        robot.frontLeft.setPower(y + x + rx);
        robot.frontRight.setPower(y - x - rx);
        robot.backLeft.setPower(y - x + rx);
        robot.backRight.setPower(y + x - rx);

        //Claw Stuff
        switch (currentClawPosition) {
            case Open:
                robot.claw.setPosition(Qual11588Hardware.OPEN_POSITION);
                if (impGamepad1.b.isInitialPress()) {
                    currentClawPosition = ClawPosition.Closed;
                }
                break;
            case Closed:
                robot.claw.setPosition(robot.CLOSED_POSITION);
                if (impGamepad1.b.isInitialPress()) {
                    currentClawPosition = ClawPosition.Open;
                }
        }

        //Arm Stuff
        //Reset relative to low position
        if (impGamepad1.x.isInitialPress()) {
            lowArmPosition = robot.arm.getCurrentPosition();
            groundArmPosition = lowArmPosition - 350;
            stackArmPosition = lowArmPosition - 200;
            midArmPosition = lowArmPosition + 250;
            highArmPosition = lowArmPosition + 600;
            zeroAngleTicks = lowArmPosition - 75;
        }

        /*
        I'm really gonna miss you guys. I'm really sorry that it had to end like this
        I'm always here to help if you need me. Please don't forget me. I have spent
        4 years on this team doing everything for this team. I really wish I could stay
        until the end of the year but I just can't do it anymore. You guys know how to
        reach me if you need it. I'm sorry
         */

        if (impGamepad1.dpad_left.isInitialPress()) {
            //Sets the armTarget to ground/intake
            armTarget = groundArmPosition;
            currentArmHeight = Height.GROUND;
        } else if (impGamepad1.dpad_down.isInitialPress()) {
            //Sets the armTarget to the low pole
            armTarget = lowArmPosition;
            currentArmHeight = Height.LOW;
        } else if (impGamepad1.dpad_right.isInitialPress()) {
            //Sets the armTarget to the mid pole
            armTarget = midArmPosition;
            currentArmHeight = Height.MID;
        } else if (impGamepad1.dpad_up.isInitialPress()) {
            //Sets the armTarget to the high pole
            armTarget = highArmPosition;
            currentArmHeight = Height.HIGH;
        } else if(impGamepad1.right_bumper.isInitialPress()){
            stackArmPosition += 35;
            armTarget = stackArmPosition;
            currentArmHeight = Height.STACK;
        }else if(impGamepad1.left_bumper.isInitialPress()){
            stackArmPosition -= 35;
            armTarget = stackArmPosition;
            currentArmHeight = Height.STACK;
        }
        /*
        else if ((impGamepad1.right_bumper.isInitialPress() || impGamepad1.left_bumper.isInitialPress()) && currentArmHeight != Height.STACK) {

            If a stack button is pressed and we aren't already at a stack height
            sets armTarget to stack height

            armTarget = stackArmPosition;
            currentArmHeight = Height.STACK;
        } else if (impGamepad1.right_bumper.isInitialPress()) {
            //Increases stack position up one cone
            stackArmPosition += 35;
        } else if (impGamepad1.left_bumper.isInitialPress()) {
            //Increases stack position down one cone
            stackArmPosition -= 35;

        }
        */

        /*Allows for the armTarget to be changed for the duration of the TeleOp rather than resetting
        when you change height*/
        if (impGamepad1.y.isInitialPress()) {
            switch (currentArmHeight) {
                case GROUND:
                    groundArmPosition += 10;
                    break;
                case STACK:
                    stackArmPosition += 10;
                    break;
                case LOW:
                    lowArmPosition += 10;
                    break;
                case MID:
                    midArmPosition += 10;
                    break;
                case HIGH:
                    highArmPosition += 10;
                    break;
            }
        }
        if (impGamepad1.a.isInitialPress()) {
            switch (currentArmHeight) {
                case GROUND:
                    groundArmPosition -= 10;
                    break;
                case STACK:
                    stackArmPosition -= 10;
                    break;
                case LOW:
                    lowArmPosition -= 10;
                    break;
                case MID:
                    midArmPosition -= 10;
                    break;
                case HIGH:
                    highArmPosition -= 10;
                    break;
            }
        }

        switch(currentArmHeight){
            case GROUND:
                armTarget = groundArmPosition;
                break;
            case STACK:
                armTarget = stackArmPosition;
                break;
            case LOW:
                armTarget = lowArmPosition;
                break;
            case MID:
                armTarget = midArmPosition;
                break;
            case HIGH:
                armTarget = highArmPosition;
                break;
        }

        robot.arm.setPower(armPower(armTarget));

        /*
        I don't think there is really a reason for this, it will immediately
        try to go to the target position above where its pressed so annoying
        if(impGamepad1.back.isInitialPress()){
            robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
         */
        telemetryStuff();
    }

    public double armPower(int target){
        error = target - robot.arm.getCurrentPosition();
        dArm = (error - pArm) / PIDTimer.seconds();
        iArm = iArm + (error * PIDTimer.seconds());
        pArm = error;
        armAngle = recalculateAngle();
        cosArm = Math.cos(Math.toRadians(armAngle));
        total = ((pArm * kp) + (iArm * ki) + (dArm * kd))/100 + (cosArm * kCos);
        PIDTimer.reset();

        if(currentArmHeight != lastArmHeight){
            iArm = 0;
        }

        if(iArm > iArmMax){
            iArm = iArmMax;
        }else if(iArm < -iArmMax){
            iArm = -iArmMax;
        }

        if(total > .5){
            total = .5;
        }
        if(recalculateAngle() > 60 && total < -.3){
            total = -.3;
        }else if(total < .005 && recalculateAngle() < 60){
            total = .005;
        }
        lastArmHeight = currentArmHeight;

        return total;
    }

    public double recalculateAngle(){
        //Placeholder variables that will be deleted
        double rightAngleDiff = 800;
        double slope = 90/((zeroAngleTicks + rightAngleDiff) - zeroAngleTicks);
        double newAngle = slope * (robot.arm.getCurrentPosition() - zeroAngleTicks);
        return newAngle;
    }

    public void telemetryStuff(){
        telemetry.addData("Front Left Position", robot.frontLeft.getCurrentPosition());
        telemetry.addData("Front Right Position", robot.frontRight.getCurrentPosition());
        telemetry.addData("Back Left Position", robot.backLeft.getCurrentPosition());
        telemetry.addData("Back Right Position", robot.backRight.getCurrentPosition());
        telemetry.addData("Claw Position", robot.claw.getPosition());
        telemetry.addData("Claw State", currentClawPosition);
        telemetry.addData("Arm Target", armTarget);
        telemetry.addData("Arm Position", robot.arm.getCurrentPosition());
        telemetry.addData("Arm Angle", recalculateAngle());
        telemetry.addData("Current Target Height", currentArmHeight);
        telemetry.addData("Ground Position", groundArmPosition);
        telemetry.addData("Stack Position", stackArmPosition);
        telemetry.addData("Low Position", lowArmPosition);
        telemetry.addData("Medium Position", midArmPosition);
        telemetry.addData("High Position", highArmPosition);
        telemetry.addData("PID Total", total);
        telemetry.addData("P Arm", pArm);
        telemetry.addData("I Arm", iArm);
        telemetry.addData("D Arm", dArm);
        telemetry.addData("Cos Arm", cosArm);
        telemetry.addData("Proportional Stuff", pArm * kp);
        telemetry.addData("Integral Stuff", iArm * ki);
        telemetry.addData("Derivative Stuff", dArm * kd);
        telemetry.addData("Cos Stuff", cosArm * kCos);
        telemetry.update();
    }
}
