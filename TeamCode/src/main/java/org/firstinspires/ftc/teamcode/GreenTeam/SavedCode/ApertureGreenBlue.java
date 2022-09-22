package org.firstinspires.ftc.teamcode.GreenTeam.SavedCode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Sidearm Green Blue Simp", group = "Green")
public class ApertureGreenBlue extends SidearmGreenBaseAuto {
    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this.hardwareMap, true);
        robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

        waitForStart();
        runtime.reset();
        while(runtime.seconds()<2){}
        SidearmGreenHardware.DuckPosition position = robot.winner;
        telemetry.addData("Duck Position", position);
        telemetry.update();
        strafeXY(0, 10);
        turnByAngle(180);
        strafeXY(34, 6);
        spinDuck(3000, SidearmGreenBaseAuto.Side.BLUE);
        strafeXY(-6, 0);
        turnByAngle(-90);
        SidearmGreenBaseAuto.Height armHeight = SidearmGreenBaseAuto.Height.SHELF_HIGH;
        if(position == SidearmGreenHardware.DuckPosition.LEFT){
            armHeight = SidearmGreenBaseAuto.Height.SHELF;
        } else if(position == SidearmGreenHardware.DuckPosition.MIDDLE){
            armHeight = SidearmGreenBaseAuto.Height.SHELF_MID;
        }
        moveArm(armHeight);
        strafeXY(-36, 38);
        robot.intakeMotor.setPower(-.4);
        robot.safeWait(1000);
        robot.intakeMotor.setPower(0);
        strafeXY(0, -10);
        robot.arm.setPower(0);
        strafeXY(20, -40);
    }
}
