package org.firstinspires.ftc.teamcode.Outreach.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Outreach.Hardware.DDRDance;
import org.firstinspires.ftc.teamcode.Outreach.Hardware.DanceObserver;
import org.firstinspires.ftc.teamcode.Outreach.Hardware.OutreachBotOneHardware;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.DDRGamepad;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

import java.util.ArrayList;

@TeleOp(name = "New DDR Outreach", group = "Outreach")
public class NewDDRDanceTeleOp extends OpMode {
    OutreachBotOneHardware robot = new OutreachBotOneHardware();
    ElapsedTime timer = new ElapsedTime();
    static DDRGamepad participantGamepad;
    ImprovedGamepad masterGamepad;
    double speedBoostFactor = 1;
    boolean spining = false;
    static ArrayList<DDRDance> dances = new ArrayList<DDRDance>();
    static DDRDance testDance;
    boolean overide = false;
    private double leftSpeed = 0;
    private double rightSpeed = 0;
    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Master override", "X");
        telemetry.update();
        participantGamepad = new DDRGamepad(this.gamepad1, this.timer, "GP1");
        masterGamepad = new ImprovedGamepad(this.gamepad2, this.timer, "GP2");
        initDances();
    }

    @Override
    public void loop() {
        participantGamepad.update();
        masterGamepad.update();
        if (masterGamepad.areButtonsActive()) {
            robot.rightDrive.setPower(masterGamepad.left_stick_y.getValue());
            robot.leftDrive.setPower(masterGamepad.right_stick_y.getValue());
        }

        if (masterGamepad.x.isInitialPress()) {
            overide = !overide;
        }

        if (overide) {
            return;
        }

        updateDances();
        leftSpeed = 0;
        rightSpeed = 0;

        if (participantGamepad.upArrow.getValue()) {
            leftSpeed += .5 * speedBoostFactor;
            rightSpeed += .5 * speedBoostFactor;
        } else if (participantGamepad.downArrow.getValue()) {
            leftSpeed += -.5 * speedBoostFactor;
            rightSpeed += -.5 * speedBoostFactor;
        }

        if (participantGamepad.leftArrow.getValue()) {
            if (rightSpeed == 0 && rightSpeed == 0) {
                leftSpeed = -.4;
                rightSpeed = .4;
            } else if (leftSpeed > 0){
                rightSpeed = .65;
            } else {
                rightSpeed = -.65;
            }
        } else if (participantGamepad.rightArrow.getValue()) {
            if (rightSpeed == 0 && rightSpeed == 0) {
                rightSpeed = -.4;
                leftSpeed = .4;
            } else if (rightSpeed > 0) {
                leftSpeed = .65;
            } else {
                leftSpeed = -.65;
            }
        }
        switch (robot.currentClawState) {
            case OPEN:
                if (participantGamepad.topRightArrow.isInitialPress() || participantGamepad.topLeftArrow.isInitialPress() || masterGamepad.b.isInitialPress()) {
                    robot.currentClawState = OutreachBotOneHardware.ClawState.CLOSED;
                }
                robot.claw.setPosition(robot.OPEN_POSITON);
                break;
            case CLOSED:
                if (participantGamepad.topRightArrow.isInitialPress() || participantGamepad.topLeftArrow.isInitialPress() || masterGamepad.b.isInitialPress()) {
                    robot.currentClawState = OutreachBotOneHardware.ClawState.OPEN;
                }
                robot.claw.setPosition(robot.CLOSED_POSITON);
                break;
        }
        robot.rightDrive.setPower(rightSpeed);
        robot.leftDrive.setPower(leftSpeed);


    }

    public void initDances() {
        ArrayList testDanceMoves = new ArrayList<DDRDance.DanceMoves>();
        testDanceMoves.add(DDRDance.DanceMoves.LEFT);
        testDanceMoves.add(DDRDance.DanceMoves.RIGHT);
        testDanceMoves.add(DDRDance.DanceMoves.UP);
        testDance = new DDRDance(testDanceMoves, participantGamepad, new SpeedDanceObserver());
        dances.add(testDance);
    }

    public void updateDances() {
        testDance.update();
    }

    private class SpeedDanceObserver implements DanceObserver
    {
        @Override
        public void onCompleted() {
            speedBoostFactor = 1.5;
        }

        @Override
        public void onSuccess() {
            //none
        }
    }

    private class SuperSpeedDanceObserver implements DanceObserver
    {
        @Override
        public void onCompleted() {
            speedBoostFactor = 2;
        }

        @Override
        public void onSuccess() {
            //none
        }
    }

    private class SpinDanceObserver implements DanceObserver
    {
        @Override
        public void onCompleted() {
            spining = true;

        }

        @Override
        public void onSuccess() {
            //none
        }
    }
}
