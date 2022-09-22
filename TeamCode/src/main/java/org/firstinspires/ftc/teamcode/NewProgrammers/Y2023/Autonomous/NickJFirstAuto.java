package org.firstinspires.ftc.teamcode.NewProgrammers.Y2023.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robocol.TelemetryMessage;

import org.firstinspires.ftc.teamcode.NewProgrammers.Y2023.Hardware.NickJFIrstHardware;


@Autonomous(name = "Nick Jerden First Auto")
public class NickJFirstAuto extends LinearOpMode {

    public NickJFIrstHardware robot = new NickJFIrstHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this.hardwareMap);
        waitForStart();
        //nothing under here will run into start is hit
        turnLeft(8);
        moveInches(1);
        turnLeft(8);
        moveInches(1);
        turnLeft(8);
        moveInches(1);
        turnLeft(8);
    }

    public void turnLeft(double inches) {

        /*
         * Very Similar to the moveInches() method, the right drive behaves just like the normal
         * moveInches() method, but the left drive is reserved. All math is essentially the same
         * but the left drive target position is set equal to the current ticks minus how many ticks
         *  we want to move and the motor's power is negative
         */

        int ticks = (int) (inches * robot.TICKS_PER_INCH);

        robot.leftDrive.setTargetPosition((int) (robot.leftDrive.getCurrentPosition() - ticks));
        robot.rightDrive.setTargetPosition((int) (robot.rightDrive.getCurrentPosition() + ticks));

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftDrive.setPower(-.5);
        robot.rightDrive.setPower(.5);

        while (opModeIsActive() && (robot.leftDrive.isBusy() && robot.rightDrive.isBusy()
        )
        )
        {
            telemetry.addData("Current Left Position", robot.leftDrive.getCurrentPosition());
            telemetry.addData("Current Right Position", robot.rightDrive.getCurrentPosition());
            telemetry.update();
        }



        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void moveInches(double inches)
    {
        int ticks = (int) (inches * robot.TICKS_PER_INCH);

        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + ticks);
        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + ticks);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftDrive.setPower(.5);
        robot.rightDrive.setPower(.5);

        while (opModeIsActive() && (robot.leftDrive.isBusy() && robot.rightDrive.isBusy()
            )
        )
        {
            telemetry.addData("Current Left Position", robot.leftDrive.getCurrentPosition());
            telemetry.addData("Current Right Position", robot.rightDrive.getCurrentPosition());
            telemetry.update();
        }



            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
