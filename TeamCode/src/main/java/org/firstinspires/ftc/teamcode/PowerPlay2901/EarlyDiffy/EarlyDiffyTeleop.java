package org.firstinspires.ftc.teamcode.PowerPlay2901.EarlyDiffy;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

@TeleOp(name = "Early Diffy Teleop")
public class EarlyDiffyTeleop extends OpMode {
    EarlyDiffyHardware robot = new EarlyDiffyHardware();
    private ElapsedTime runtime = new ElapsedTime();
    ImprovedGamepad impGamepad1;

    double leftPodPower = 0;
    double rightPodPower = 0;
    double leftTurnPower = 0;
    double rightTurnPower = 0;

    @Override
    public void init() {
        robot.init(hardwareMap);
        impGamepad1 = new ImprovedGamepad(this.gamepad1, this.runtime, "GP1");
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
            double moveAngle = Math.toDegrees(Math.atan2(sidePower, -forwardPower+.001));
            leftPodPower = Math.sqrt(forwardPower*forwardPower+sidePower*sidePower)+(turnPower*Math.cos(Math.toRadians(moveAngle)));
            rightPodPower = Math.sqrt(forwardPower*forwardPower+sidePower*sidePower)-(turnPower*Math.cos(Math.toRadians(moveAngle)));
            leftTurnPower = leftPodTurn(moveAngle-(45*turnPower*Math.sin(Math.toRadians(moveAngle))));
            rightTurnPower = rightPodTurn(moveAngle+(45*turnPower*Math.sin(Math.toRadians(moveAngle))));
        }

        int speedMod = 3;
        if(gamepad1.left_bumper){
            speedMod = 1;
        }

        robot.leftOne.setPower(leftPodPower/speedMod+leftTurnPower);
        robot.leftTwo.setPower(leftPodPower/speedMod-leftTurnPower);
        robot.rightOne.setPower(rightPodPower/speedMod+rightTurnPower);
        robot.rightTwo.setPower(rightPodPower/speedMod-rightTurnPower);
    }

    double kp = 1.2;
    double ki = 0;
    double kd = 0;

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
        double secs = runtime.seconds();
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
        double secs = runtime.seconds();
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
