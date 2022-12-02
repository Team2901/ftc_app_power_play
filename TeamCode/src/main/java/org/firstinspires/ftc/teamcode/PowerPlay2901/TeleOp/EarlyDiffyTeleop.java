package org.firstinspires.ftc.teamcode.PowerPlay2901.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PowerPlay2901.Hardware.EarlyDiffyHardware;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

@Disabled
@TeleOp(name = "Early Diffy Teleop")
public class EarlyDiffyTeleop extends OpMode {
    EarlyDiffyHardware robot = new EarlyDiffyHardware();
    private ElapsedTime runtime = new ElapsedTime();
    ImprovedGamepad impGamepad1;

    double leftPodPower = 0;
    double rightPodPower = 0;
    public double leftTurnPower = 0;
    public double rightTurnPower = 0;
    double moveAngle;

    @Override
    public void init() {
        robot.init(hardwareMap);
        impGamepad1 = new ImprovedGamepad(this.gamepad1, this.runtime, "GP1");
    }

    @Override
    public void loop() {
        impGamepad1.update();
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

//        if(gamepad1.a||gamepad2.a){
//            robot.clawOne.setPosition(0.12);
////            robot.clawTwo.setPosition(0.14);
//        } else if(gamepad1.x||gamepad2.x/*robot.clawSensor.getDistance(DistanceUnit.INCH)<1.5*/){
//            robot.clawOne.setPosition(0);
////            robot.clawTwo.setPosition(0.25);
//        }

//        if(gamepad1.dpad_left && robot.clawOne.getPosition() > 0){
//            robot.clawOne.setPosition(robot.clawOne.getPosition() - 0.01);
//        } else if(gamepad1.dpad_right && robot.clawOne.getPosition() < 1){
//            robot.clawOne.setPosition(robot.clawOne.getPosition() + 0.01);
//        } else if(gamepad1.dpad_up && robot.clawTwo.getPosition() < 1){
//            robot.clawTwo.setPosition(robot.clawTwo.getPosition() + 0.01);
//        } else if(gamepad1.dpad_down && robot.clawTwo.getPosition() > 0){
//            robot.clawTwo.setPosition(robot.clawTwo.getPosition() - 0.01);
//        }

//        if(gamepad2.dpad_up){
//            robot.clawOne.setPosition(0.08);
//            robot.clawTwo.setPosition(0.14);
//        } else if(gamepad2.dpad_down){
//            robot.clawOne.setPosition(0.0);
//            robot.clawTwo.setPosition(0.25);
//        }

        double liftPower = gamepad2.left_stick_y;

//        robot.liftOne.setPower(liftPower);
//        robot.liftTwo.setPower(liftPower);

        int speedMod = 3;
        if(gamepad1.left_bumper){
            speedMod = 1;
        }

        robot.leftOne.setVelocity((leftPodPower/speedMod+leftTurnPower)*2500);
        robot.leftTwo.setVelocity((leftPodPower/speedMod-leftTurnPower)*2500);
        robot.rightOne.setVelocity((rightPodPower/speedMod+rightTurnPower)*2500);
        robot.rightTwo.setVelocity((rightPodPower/speedMod-rightTurnPower)*2500);

        telemetry.addData("Left Pod Angle", leftPodAngle);
        telemetry.addData("Right Pod Angle", rightPodAngle);
        //telemetry.addData("Lift Position", robot.liftOne.getCurrentPosition());
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

    double klp = .5;
    double kli = 0;
    double kld = 0;

    private ElapsedTime runtimeLift = new ElapsedTime();
    double liftP = 0;
    double liftI = 0;
    double liftD = 0;

    /*public double liftPower(double target){
        double error = target-robot.liftOne.getCurrentPosition();
        double secs = runtimeLift.seconds();
        runtime.reset();
        liftD = (error - liftP) / secs;
        liftI = liftI + (error * secs);
        liftP = error;
        double total = (klp* pAngleRight + kli* iAngleRight + kld* dAngleRight)/100;
        if(total > 1){
            liftI = 0;
            total = 1;
        }
        if(total < -1){
            liftI = 0;
            total = -1;
        }
        return total;
    }*/
}
