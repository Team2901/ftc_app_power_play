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
//    static ArrayList<DDRDance> dances = new ArrayList<DDRDance>();
//    static DDRDance testDance;
    boolean overide = false;
    private final double MOTOR_POWER = 0.5;
    @Override
    public void init() {
        robot.init(hardwareMap);
        participantGamepad = new DDRGamepad(this.gamepad1, this.timer, "GP1");
        masterGamepad = new ImprovedGamepad(this.gamepad2, this.timer, "GP2");
    }

    @Override
    public void loop() {

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

//        updateDances();

        if (participantGamepad.upArrow.getValue()) {
            robot.leftDrive.setPower(MOTOR_POWER);
            robot.rightDrive.setPower(MOTOR_POWER);
        }
        if (participantGamepad.downArrow.getValue()) {
            robot.leftDrive.setPower(-MOTOR_POWER);
            robot.rightDrive.setPower(-MOTOR_POWER);
        }

        if (participantGamepad.leftArrow.getValue()) {
            robot.leftDrive.setPower(MOTOR_POWER);
        }

        if (participantGamepad.rightArrow.getValue()) {
            robot.rightDrive.setPower(MOTOR_POWER);
        }

        switch (robot.currentClawState) {
            case OPEN:
                if (participantGamepad.a.isInitialPress() || masterGamepad.b.isInitialPress()) {
                    robot.currentClawState = OutreachBotOneHardware.ClawState.CLOSED;
                }
                robot.claw.setPosition(robot.OPEN_POSITON);
                break;
            case CLOSED:
                if (participantGamepad.a.isInitialPress() || masterGamepad.b.isInitialPress()) {
                    robot.currentClawState = OutreachBotOneHardware.ClawState.OPEN;
                }
                robot.claw.setPosition(robot.CLOSED_POSITON);
                break;
        }
    }

//    public void initDances() {
//        ArrayList testDanceMoves = new ArrayList<DDRDance.DanceMoves>();
//        testDanceMoves.add(DDRDance.DanceMoves.X);
//        testDanceMoves.add(DDRDance.DanceMoves.LEFT);
//        testDanceMoves.add(DDRDance.DanceMoves.RIGHT);
//        testDanceMoves.add(DDRDance.DanceMoves.O);
//        testDance = new DDRDance(testDanceMoves, participantGamepad,new BarDanceObserver());
//        dances.add(testDance);
//    }
//
//    public void updateDances() {
//        testDance.update();
//    }
//
//    private class BarDanceObserver implements DanceObserver
//    {
//        @Override
//        public void onCompleted() {
//            telemetry.addData("Test dance complete", true);
//            telemetry.update();
//        }
//    }
}
