package org.firstinspires.ftc.teamcode.PowerPlay2901.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PowerPlay2901.Hardware.RockBotHardware;

public class RockBotTeleop extends OpMode {
    RockBotHardware robot = new RockBotHardware();
    private ElapsedTime runtime = new ElapsedTime();

    double leftPodPower = 0;
    double rightPodPower = 0;
    public double leftTurnPower = 0;
    public double rightTurnPower = 0;
    double moveAngle;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        double forwardPower = gamepad1.left_stick_y;
        double sidePower = gamepad1.left_stick_x;
        double turnPower = gamepad1.right_stick_x;

        if(gamepad1.right_bumper){
            leftPodPower = turnPower-forwardPower;
            rightPodPower = -turnPower-forwardPower;
            leftTurnPower = leftPodTurn(0);
            rightTurnPower = rightPodTurn(0);
        } else {
            moveAngle = Math.toDegrees(Math.atan2(sidePower, -forwardPower+.001));
            //moveAngle = AngleUnit.normalizeDegrees(moveAngle+robot.getAngle()); //uncomment this for field oriented
            leftPodPower = Math.sqrt(forwardPower*forwardPower+sidePower*sidePower)+(turnPower*Math.cos(Math.toRadians(moveAngle)));
            rightPodPower = Math.sqrt(forwardPower*forwardPower+sidePower*sidePower)-(turnPower*Math.cos(Math.toRadians(moveAngle)));
            leftTurnPower = leftPodTurn(moveAngle-(45*turnPower*Math.sin(Math.toRadians(moveAngle))));
            rightTurnPower = rightPodTurn(moveAngle+(45*turnPower*Math.sin(Math.toRadians(moveAngle))));
        }

        int speedMod = 3;
        if(gamepad1.left_bumper){
            speedMod = 1;
        }

        if(gamepad2.dpad_up){
            //passthrough extends
        } else if(gamepad2.dpad_down){
            //passthrough retracts
        }

        robot.liftOne.setPower(gamepad2.right_stick_y/4+.05);
        robot.liftTwo.setPower(gamepad2.right_stick_y/4+.05);

        robot.leftOne.setVelocity((leftPodPower/speedMod+leftTurnPower)*2500);
        robot.leftTwo.setVelocity((leftPodPower/speedMod-leftTurnPower)*2500);
        robot.rightOne.setVelocity((rightPodPower/speedMod+rightTurnPower)*2500);
        robot.rightTwo.setVelocity((rightPodPower/speedMod-rightTurnPower)*2500);

        telemetry.addData("Left Pod Angle", leftPodAngle);
        telemetry.addData("Right Pod Angle", rightPodAngle);
        telemetry.addData("Lift Position", robot.liftOne.getCurrentPosition());
        telemetry.addData("joy angle", moveAngle);
        telemetry.update();
    }

    double kp = 1.2;
    double ki = 0;
    double kd = 0;

    private ElapsedTime runtimePodLeft = new ElapsedTime();
    double leftPodAngle = 0;
    double pAngleLeft = 0;
    double iAngleLeft = 0;
    double dAngleLeft = 0;

    public double leftPodTurn(double angle){
        leftPodAngle = (robot.leftOne.getCurrentPosition() - robot.leftTwo.getCurrentPosition())/8.95;
        double error = AngleUnit.normalizeDegrees(angle - leftPodAngle);
        if(!gamepad1.start && (error >= 90 || error <= -90)){
            error = AngleUnit.normalizeDegrees(error-180);
            leftPodPower = -leftPodPower;
        }
        double secs = runtimePodLeft.seconds();
        runtime.reset();
        dAngleLeft = (error - pAngleLeft) / secs;
        iAngleLeft = iAngleLeft + (error * secs);
        pAngleLeft = error;
        double total = (kp* pAngleLeft + ki* iAngleLeft + kd* dAngleLeft)/100;
        if(total > 1){
            iAngleLeft = 0;
            total = 1;
        }
        if(total < -1){
            iAngleLeft = 0;
            total = -1;
        }
        return total;
    }

    private ElapsedTime runtimePodRight = new ElapsedTime();
    double rightPodAngle = 0;
    double pAngleRight = 0;
    double iAngleRight = 0;
    double dAngleRight = 0;

    public double rightPodTurn(double angle){
        rightPodAngle = (robot.rightOne.getCurrentPosition() - robot.rightTwo.getCurrentPosition())/8.95;
        double error = AngleUnit.normalizeDegrees(angle - rightPodAngle);
        if(!gamepad1.start && (error >= 90 || error <= -90)){
            error = AngleUnit.normalizeDegrees(error-180);
            rightPodPower = -rightPodPower;
        }
        double secs = runtimePodRight.seconds();
        runtime.reset();
        dAngleRight = (error - pAngleRight) / secs;
        iAngleRight = iAngleRight + (error * secs);
        pAngleRight = error;
        double total = (kp* pAngleRight + ki* iAngleRight + kd* dAngleRight)/100;
        if(total > 1){
            iAngleRight = 0;
            total = 1;
        }
        if(total < -1){
            iAngleRight = 0;
            total = -1;
        }
        return total;
    }
}
