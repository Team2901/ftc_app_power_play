package org.firstinspires.ftc.teamcode.PowerPlay2901.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PowerPlay2901.EarlyDiffy.EarlyDiffyHardware;

public class EarlyDiffyBaseAuto extends LinearOpMode {
    EarlyDiffyHardware robot = new EarlyDiffyHardware();

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void moveInches(double inches){

    }

    public void turnByAngle(double angle){
        ElapsedTime runtimeTurn = new ElapsedTime();
        double kp = 1;
        double ki = 0;
        double kd = 0;
        double pAngle = 0;
        double iAngle = 0;
        double dAngle = 0;
        while(true){
            double error = AngleUnit.normalizeDegrees(angle - leftPodAngle);
            double secs = runtimeTurn.seconds();
            runtimeTurn.reset();
            dAngle = (error - pAngle) / secs;
            iAngle = iAngle + (error * secs);
            pAngle = error;
            double total = (kp* pAngle + ki* iAngle + kd* dAngle)/100;
            double leftPodTurn = leftPodTurn(0);
            double rightPodTurn = rightPodTurn(0);
            robot.leftOne.setVelocity((total+leftPodTurn)*250*5);
            robot.leftTwo.setVelocity((total-leftPodTurn)*250*5);
            robot.rightOne.setVelocity((-total+rightPodTurn)*250*5);
            robot.rightTwo.setVelocity((-total-rightPodTurn)*250*5);
            if(dAngle<5&&dAngle>-5&&error<5&&error>-5){
                return;
            }
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
        double secs = runtimePodLeft.seconds();
        runtimePodLeft.reset();
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
        double secs = runtimePodRight.seconds();
        runtimePodRight.reset();
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
