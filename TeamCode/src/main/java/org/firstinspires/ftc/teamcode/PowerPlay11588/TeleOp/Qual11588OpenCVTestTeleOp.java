package org.firstinspires.ftc.teamcode.PowerPlay11588.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.Qual11588Hardware;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;
@TeleOp(name = "OCV Qual 11588", group = "11588")
public class Qual11588OpenCVTestTeleOp extends OpMode {
    Qual11588Hardware robot = new Qual11588Hardware();
    public ElapsedTime gamepadTimer = new ElapsedTime();
    public enum ClawPosition{Open, Closed};
    Qual11588TeleOp.ClawPosition currentClawPosition = Qual11588TeleOp.ClawPosition.Closed;
    public ImprovedGamepad impGamepad1;
    public ImprovedGamepad impGamepad2;
    double turningPower = 0;

    //All the variables that are needed for pid
    ElapsedTime PIDTimer = new ElapsedTime();
    int armTarget = 200;
    int realArmTarget = armTarget;
    int lastTarget = armTarget;
    int modifier = 0;
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
    int firstRedFrame = -1;
    int firstGreenFrame =-1;
    int firstBlueFrame = -1;


    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, true);
        impGamepad1 = new ImprovedGamepad(gamepad1, gamepadTimer, "g1");
        impGamepad2 = new ImprovedGamepad(gamepad2, gamepadTimer, "g2");
    }

    @Override
    public void loop() {
        impGamepad1.update();
        impGamepad2.update();
        if(impGamepad1.dpad_left.isInitialPress()){
            //Sets the armTarget to ground/intake
            armTarget = 200;
        }else if(impGamepad1.dpad_down.isInitialPress()){
            //Sets the armTarget to the low pole
            armTarget = 475;
        }else if(impGamepad1.dpad_right.isInitialPress()){
            //Sets the armTarget to the mid pole
            armTarget = 775;
        }else if(impGamepad1.dpad_up.isInitialPress()){
            //Sets the armTarget to the high pole
            armTarget = 1000;
        }
        /*Allows for the armTarget to be changed for the duration of the TeleOp rather than resetting
        when you change height*/
        if(impGamepad1.y.isInitialPress()){
            modifier += 5;
        }
        if(impGamepad1.a.isInitialPress()){
            modifier -= 5;
        }

        realArmTarget = armTarget + modifier;
        robot.arm.setPower(armPower(realArmTarget));

        if(impGamepad1.right_trigger.getValue() > 0){
            turningPower = .3 * impGamepad1.right_trigger.getValue();
        }else if(impGamepad1.left_trigger.getValue() > 0){
            turningPower = -.3 * impGamepad1.left_trigger.getValue();
        }else{
            turningPower = impGamepad1.right_stick_x.getValue();
        }
        double y = .75 * impGamepad1.left_stick_y.getValue();
        double x = .75 * impGamepad1.left_stick_x.getValue();
        double rx = turningPower;

        robot.frontLeft.setPower(y + x + rx);
        robot.frontRight.setPower(y - x - rx);
        robot.backLeft.setPower(y - x + rx);
        robot.backRight.setPower(y + x - rx);

        switch (currentClawPosition){
            case Open:
                robot.claw.setPosition(Qual11588Hardware.OPEN_POSITION);
                if(impGamepad1.b.isInitialPress()){
                    currentClawPosition = Qual11588TeleOp.ClawPosition.Closed;
                }
                break;
            case Closed:
                robot.claw.setPosition(robot.CLOSED_POSITION);
                if(impGamepad1.b.isInitialPress()){
                    currentClawPosition = Qual11588TeleOp.ClawPosition.Open;
                }
        }
        if (firstRedFrame == -1 && robot.pipeLine.redAmount > 0) {
            firstRedFrame = robot.pipeLine.framesProceeded;
        } else if (firstGreenFrame == -1 && robot.pipeLine.greenAmount > 0) {
            firstGreenFrame = robot.pipeLine.framesProceeded;
        } else if (firstBlueFrame == -1 && robot.pipeLine.blueAmount > 0) {
            firstBlueFrame = robot.pipeLine.framesProceeded;
        }
        telemetryStuff();
    }

    public double armPower(int target){

        error = target - robot.arm.getCurrentPosition();
        dArm = (error - pArm) / PIDTimer.seconds();
        iArm = iArm + (error * PIDTimer.seconds());
        pArm = error;
        armAngle = 0.102856 * robot.arm.getCurrentPosition() - 43.6276;
        cosArm = Math.cos(Math.toRadians(armAngle));
        total = ((pArm * kp) + (iArm * ki) + (dArm * kd))/100 + (cosArm * kCos);
        PIDTimer.reset();

        if(armTarget != lastTarget){
            iArm = 0;
        }

        if(iArm > iArmMax){
            iArm = iArmMax;
        }else if(iArm < -iArmMax){
            iArm = -iArmMax;
        }

        if(total > .6){
            total = .6;
        }
        if(total < .01){
            total = .01;
        }
        lastTarget = armTarget;

        return total;
    }

    public void telemetryStuff(){
        telemetry.addData("Blue amount", robot.pipeLine.blueAmount);
        telemetry.addData("Green amount", robot.pipeLine.greenAmount);
        telemetry.addData("red amount", robot.pipeLine.redAmount);
        telemetry.addData("First red frame", firstRedFrame);
        telemetry.addData("First green frame", firstGreenFrame);
        telemetry.addData("First blue frame", firstBlueFrame);
        telemetry.update();
    }
}

