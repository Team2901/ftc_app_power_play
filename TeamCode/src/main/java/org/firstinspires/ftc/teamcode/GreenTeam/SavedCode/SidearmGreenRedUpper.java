package org.firstinspires.ftc.teamcode.GreenTeam.SavedCode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GreenTeam.SavedCode.SidearmGreenBaseAuto;
import org.firstinspires.ftc.teamcode.GreenTeam.SavedCode.SidearmGreenHardware;

@Autonomous (name = "Sidearm Red Upper", group = "Green")
public class SidearmGreenRedUpper extends SidearmGreenBaseAuto {
    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this.hardwareMap, true);
        robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

        waitForStart();
        while(runtime.seconds()<2){}
        SidearmGreenHardware.DuckPosition position = robot.winner;
        telemetry.addData("Duck Position", position);
        telemetry.update();
        strafeXY(24, 20);
        Height armHeight = Height.SHELF_HIGH;
        if(position == SidearmGreenHardware.DuckPosition.LEFT){
            armHeight = Height.SHELF;
        } else if(position == SidearmGreenHardware.DuckPosition.MIDDLE){
            armHeight = Height.SHELF_MID;
        }
        moveArm(armHeight);
        strafeXY(0, 10);
        robot.intakeMotor.setPower(-.4);
        robot.safeWait(1000);
        robot.intakeMotor.setPower(0);
        strafeXY(0, -10);
        robot.arm.setPower(0);
        strafeXY(-78, 0);
    }
}
