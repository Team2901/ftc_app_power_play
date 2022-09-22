package org.firstinspires.ftc.teamcode.NewProgrammers.Y2023.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NewProgrammers.Y2023.Hardware.KarlaSFirstHardware;

@Autonomous(name = "KarlaSFirstAuto")
public class KarlaSFirstAuto extends LinearOpMode {

    public KarlaSFirstHardware robot = new KarlaSFirstHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this.hardwareMap);
        waitForStart();
        // move, turn, move, turn, move, turn, move, turn
        moveSeconds(2);
        moveLeft(0.75);
        moveSeconds(2);
        moveLeft(0.75);
        moveSeconds(2);
        moveLeft(0.75);
        moveSeconds(2);
        moveLeft(0.75);
        //

    }

    private void moveSeconds(double seconds) {
        moveSeconds(seconds, 0.5,  0.5);
    }
    private void moveLeft(double seconds) {
        moveSeconds(seconds, 0, 1);
    }


    public void moveSeconds(double seconds, double leftPower, double rightMotorPower) {

        ElapsedTime stopwatch = new ElapsedTime();


        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.leftDrive.setPower(leftPower);
        robot.rightDrive.setPower(rightMotorPower);

        stopwatch.reset();
        double timeoutTime = 2.5;

        // Display the motors current positions while they are moving
        while (opModeIsActive() &&
                (stopwatch.seconds() < seconds)) {
            telemetry.addData("current Left Position", robot.leftDrive.getCurrentPosition());
            telemetry.addData("current Right Position", robot.rightDrive.getCurrentPosition());
            telemetry.update();
        }

        // Stop the motors and revert the motors to run using encoders
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void moveInches(double inches) {
        // Calculate the target position
        int ticks = (int) (inches * robot.TICKS_PER_INCH);

        // Set the target position for each motor and start moving
        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + ticks);
        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + ticks);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        robot.leftDrive.setPower(.5);
        robot.rightDrive.setPower(.5);

        // Display the motors current positions while they are moving
        while (opModeIsActive() &&
                (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {
            telemetry.addData("current Left Position", robot.leftDrive.getCurrentPosition());
            telemetry.addData("current Right Position", robot.rightDrive.getCurrentPosition());
            telemetry.update();
        }

        // Stop the motors and revert the motors to run using encoders
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



}

