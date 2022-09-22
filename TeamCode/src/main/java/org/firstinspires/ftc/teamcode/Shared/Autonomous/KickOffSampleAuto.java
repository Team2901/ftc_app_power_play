package org.firstinspires.ftc.teamcode.Shared.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Shared.Hardware.KickOffSampleHardware;

import java.util.List;

@Disabled
@Autonomous(name = "Kick Off Sample", group = "Shared")
public class KickOffSampleAuto extends LinearOpMode {
    public KickOffSampleHardware robot = new KickOffSampleHardware();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(this.hardwareMap);

        robot.initTfod(this.hardwareMap);

        robot.activateTfod();

        waitForStart();

        // Move forwards 6 inches
        moveInches(6);

        //waveServoHand();

        // Turn the robot 90 degrees (counter-clockwise)
        turnByAngle(90);

        // Continually call findSkyStone until the opmode ends
        while (opModeIsActive()) {
            Float centerPercentDifference = findSkyStone();
        }

        robot.shutdownTfod();
    }

    public void moveInches(double inches) {
        // Calculate the target position
        int ticks = (int) (inches * robot.TICKS_PER_INCH);

        // Set the target position for each motor
        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + ticks);
        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + ticks);

        // Set the motors to run to a target position
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start the motors (at half speed)
        robot.leftDrive.setPower(.5);
        robot.rightDrive.setPower(.5);

        // Display the motors current positions while they are moving
        // Note: Always check opModeIsActive() when using while loops
        while (opModeIsActive() &&
                (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {
            telemetry.addData("Current Left Position", robot.leftDrive.getCurrentPosition());
            telemetry.addData("Current Right Position", robot.rightDrive.getCurrentPosition());

            telemetry.update();
        }

        // Stop the motors
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        // Reset the motors to run using encoders
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*public void waveServoHand() {
        for (int i = 0; i < 4; i++) {
            //robot.armServo.setPosition(.25);
            sleep(1000);
            //robot.armServo.setPosition(.75);
            sleep(1000);
        }
    }*/

    public void turnByAngle(double turnAngle) {

        // Get robot's start angle
        double startAngle = robot.getAngle();

        // Calculate the robot's end angle (Normalize to be between -180 and 180)
        double targetAngle = AngleUnit.normalizeDegrees(startAngle + turnAngle);

        // Determine which direction the robot needs to turn (counter-clockwise or clockwise)
        int turnDirection;
        if (turnAngle >= 0) {
            // Turn counter-clockwise
            turnDirection = 1;
        } else {
            // Turn clockwise
            turnDirection = -1;
        }

        // Set the wheels to turn the correct direction
        robot.leftDrive.setPower(-0.25 * turnDirection);
        robot.rightDrive.setPower(0.25 * turnDirection);

        double currentAngle = robot.getAngle();

        if (turnDirection == 1) {
            // If turning counter-clockwise: update currentAngle until it is at or greater than the targetAngle
            while (AngleUnit.normalizeDegrees(currentAngle - targetAngle) < 0 && opModeIsActive()) {

                telemetry.addData("Current Angle", currentAngle);
                telemetry.addData("Target Angle", targetAngle);
                telemetry.update();

                currentAngle = robot.getAngle();
            }
        } else {
            //  If turning clockwise: update currentAngle until it is at or less than the targetAngle
            while (AngleUnit.normalizeDegrees(currentAngle - targetAngle) > 0 && opModeIsActive()) {

                telemetry.addData("Current Angle", currentAngle);
                telemetry.addData("Target Angle", targetAngle);
                telemetry.update();

                currentAngle = robot.getAngle();
            }
        }

        // Stop the motors once we hit the targetAngle
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }

    public Float findSkyStone() {

        // Get the list of all recognitions from TensorFlow. Use getUpdatedRecognitions() if you only want NEW ones
        List<Recognition> recognitions;
        if (robot.tfod != null) {
            recognitions = robot.tfod.getRecognitions();
        } else {
            recognitions = null;
        }

        Float centerPercentDifference = null;

        if (recognitions != null) {

            telemetry.addData("# Object Detected", recognitions.size());

            // Look at each recognition
            for (Recognition recognition : recognitions) {

                // Ignore non-skystone recognitions
                if (!recognition.getLabel().equals(robot.LABEL_SECOND_ELEMENT)) {
                    continue;
                }

                // get the center x pixel of the recognition
                float centerSkyStone = (recognition.getRight() + recognition.getLeft()) / 2;

                // get the center x pixel of the image
                int centerFrame = recognition.getImageWidth() / 2;

                // Calculate how many pixels the recognition is off from the center of the image
                float centerDifference = centerSkyStone - centerFrame;

                // Calculate what percentage (-100 to 100) the recognition is off from the center of the image
                centerPercentDifference = (centerDifference / centerFrame) * 100;

                telemetry.addData("Center", centerSkyStone);
                telemetry.addData("Difference", centerDifference);
                telemetry.addData("Percent Difference", centerPercentDifference);
            }
        }

        telemetry.update();

        // if centerDifference/centerPercentDifference is...
        // Null:     no recognitions were found
        // Negative: the recognition is to the left side of the image
        // Zero:     the recognition is in the dead center of the image
        // Positive: the recognition is to the left side of the image
        return centerPercentDifference;
    }
}


