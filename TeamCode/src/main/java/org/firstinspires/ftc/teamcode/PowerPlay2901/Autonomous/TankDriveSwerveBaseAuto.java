package org.firstinspires.ftc.teamcode.PowerPlay2901.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PowerPlay2901.Hardware.RockBotHardware;

public class TankDriveSwerveBaseAuto extends LinearOpMode {
    RockBotHardware robot = new RockBotHardware();
    private ElapsedTime runtime = new ElapsedTime();

    double leftPodPower = 0;
    double rightPodPower = 0;
    double leftTurnPower = 0;
    double rightTurnPower = 0;

    @Override
    public void runOpMode() throws InterruptedException {}

    public void moveInches(double inches){
        leftTurnPower = leftPodTurn(0);
        rightTurnPower = rightPodTurn(0);

        while(true) {
            leftTurnPower = leftPodTurn(0);
            rightTurnPower = rightPodTurn(0);

            robot.leftOne.setVelocity((leftPodPower / 2 + leftTurnPower) * 2500);
            robot.leftTwo.setVelocity((leftPodPower / 2 - leftTurnPower) * 2500);
            robot.rightOne.setVelocity((rightPodPower / 2 + rightTurnPower) * 2500);
            robot.rightTwo.setVelocity((rightPodPower / 2 - rightTurnPower) * 2500);
        }
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
