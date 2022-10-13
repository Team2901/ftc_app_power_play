package org.firstinspires.ftc.teamcode.Outreach.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Shared.Gamepad.DDRGamepad;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.Shared.Hardware.ClawbotHardware;
import org.firstinspires.ftc.teamcode.Utility.CountDownTimer;

@TeleOp(name = "DDR Outreach Bot One Teleop")
public class DDROutreachBotOneTeleOp extends OpMode {
    final double CLAW_SPEED = 0.05;
    final ClawbotHardware robot = new ClawbotHardware();
    CountDownTimer countDownTimer = new CountDownTimer(ElapsedTime.Resolution.MILLISECONDS);
    double clawOffset = 0.0;
    boolean isLastClawPressed = false;
    boolean isClawOpen = false;
    boolean override = false;
    int difficultyMode = 1;
    String[] difficultyNames = {"Beginner", "Intermediate", "Lawsuit"};
    DDRGamepad participantGP;
    ImprovedGamepad gameMasterGP;
    ElapsedTime timer = new ElapsedTime();
    int konamiCodeProgress = 0;
    int beginnerKonamiCodeProgress = 0;
    int cardinalCodeProgress = 0;
    int swiftStepProgress = 0;
    int grapeVineProgress = 0;
    int hopScotchProgress = 0;
    boolean isActive = false;

    int armTarget = 0;

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.claw.setPosition(ClawbotHardware.MID_SERVO);

        participantGP = new DDRGamepad(this.gamepad1, this.timer, "GP1");
        gameMasterGP = new ImprovedGamepad(this.gamepad2, this.timer, "GP2");
    }

    @Override
    public void loop() {
        participantGP.update();
        gameMasterGP.update();

        double participantLeftPower;
        double participantRightPower;
        double participantArmPower;

        // gm = game master
        double gmLeftPower = 0;
        double gmRightPower = 0;
        double gmArmPower = 0;

        /*if(robot.potentiometer.getVoltage() < ClawbotHardware.MIN_ARM_VOLTAGE){
            robot.armMotor.setPower(0.3);
        } else if(robot.potentiometer.getVoltage() > ClawbotHardware.MAX_ARM_VOLTAGE){
            robot.armMotor.setPower(-0.3);
        }*/

        //another different ujkufcomment to prove a point

        // Moves robot forward using the left joystick
        if (gameMasterGP.b.getValue()) {
            countDownTimer.setTargetTime(10000);
        }

        if (gameMasterGP.left_bumper.isInitialPress() && difficultyMode > 0) {
            difficultyMode--;
        }

        if (gameMasterGP.right_bumper.isInitialPress() && difficultyMode < 2) {
            difficultyMode++;
        }

        if (gameMasterGP.a.isInitialPress() && !gameMasterGP.start.getValue()) {
            override = !override;
        }

        if (gameMasterGP.x.isInitialPress()) {
            isActive = !isActive;
        }

        gmRightPower = gameMasterGP.left_stick_y.getValue();
        gmLeftPower = gameMasterGP.left_stick_y.getValue();

        // Turns the robot using the left joystick

        gmRightPower -= gameMasterGP.left_stick_x.getValue();
        gmLeftPower += gameMasterGP.left_stick_x.getValue();

        double maxPower = Math.max(Math.abs(gmLeftPower), Math.abs(gmRightPower));

        // Adjusts speeds for the arcs.
        if (maxPower > 1) {
            gmRightPower /= maxPower;
            gmLeftPower /= maxPower;
        }

        // DDR pad left moves the arm down, DDR pad right moves the arm up, else, it stays in place.
        gmArmPower = gameMasterGP.right_stick_y.getValue() * .5;

        if (this.participantGP.rightArrow.getValue() && this.participantGP.leftArrow.getValue() && difficultyMode > 0) {
            participantLeftPower = -0.75;
            participantRightPower = -0.75;

        } else if (this.participantGP.rightArrow.getValue() && this.participantGP.leftArrow.getValue() && difficultyMode == 0) {
            participantLeftPower = 0;
            participantRightPower = 0;
        }
        //Topleft + Up = arc counterclockwise
        //Left power = 0.75, Right power = 1
        else if (this.participantGP.leftArrow.getValue() && this.participantGP.upArrow.getValue()) {
            participantLeftPower = 1;
            participantRightPower = .75;

        } //Top right + Up = arc clockwise
        //Left power = 1, Right power = 0.75
        else if (this.participantGP.rightArrow.getValue() && this.participantGP.upArrow.getValue()) {
            participantLeftPower = .75;
            participantRightPower = 1;

        } //Top left = counterclockwise
        //Left power = -0.75, Right power = 0.75
        else if (this.participantGP.leftArrow.getValue()) {
            participantLeftPower = 0.75;
            participantRightPower = -0.75;

        } //Top right = clockwise
        //Left power = 0.75, Right power = -0.75
        else if (this.participantGP.rightArrow.getValue()) {
            participantLeftPower = -0.75;
            participantRightPower = 0.75;
        } //Up = straight
        //Left and right motors same power 0.75
        else if (this.participantGP.upArrow.getValue()) {
            participantLeftPower = 0.75;
            participantRightPower = 0.75;
        } else {
            participantLeftPower = 0;
            participantRightPower = 0;
        }

        // DDR pad left moves the arm down, DDR pad right moves the arm up, else, it stays in place.
        if (this.participantGP.topLeftArrow.getValue() && robot.potentiometer.getVoltage() < ClawbotHardware.ARM_DOWN_VOLTAGE || gameMasterGP.dpad_up.getValue()) {
            armTarget = 25;
        } else if (this.participantGP.topRightArrow.getValue() && robot.potentiometer.getVoltage() > ClawbotHardware.ARM_UP_VOLTAGE || gameMasterGP.dpad_down.getValue()) {
            armTarget = 280/3;
        } else {
            participantArmPower = 0;
        }


        // Checks to see if it is the initial press of DDR pad down
        if (this.participantGP.downArrow.isInitialPress() && !override && !this.participantGP.startButton.getValue()) {
            isClawOpen = !isClawOpen;
        } else if (this.gameMasterGP.y.isInitialPress()) {
            isClawOpen = !isClawOpen;
        }


        final boolean participantInput = participantGP.areButtonsActive();

        // If there is any power set to the left or right motors, checks and moves the arm up if it is under limit.
        if (participantLeftPower != 0 || participantRightPower != 0 || gmLeftPower != 0 || gmRightPower != 0) {
            if (robot.potentiometer.getVoltage() > ClawbotHardware.ARM_DOWN_SAFE_MOVEMENT_VOLTAGE) {
                participantArmPower = 0.5;
                gmArmPower = 0.5;
            }
        }
        // If the user is pressing a button and the override is turned off then
        // the participant can use the robot.  Otherwise, the game master has complete control.


        telemetryDDRGraphic();
        telemetry.addData("Arm Power:", robot.armMotor.getPower());
        telemetry.addData("Override", override);
        telemetry.addData("Is Active", isActive);
        telemetry.addData("Mode", difficultyNames[difficultyMode]);
        telemetry.addData("Participant Input", participantInput);
        telemetry.addData("Potentiometer", robot.potentiometer.getVoltage());
        telemetry.addData("Konami Code Progress", konamiCodeProgress);
        telemetry.addData("Beginner Konami Code Progress", beginnerKonamiCodeProgress);
        telemetry.update();


    }

    public void power(double left, double right) {
        robot.leftMotor.setPower(-left);
        robot.rightMotor.setPower(-right);
    }

    public void telemetryDDRGraphic() {
        String lineOne = "";
        if (participantGP.topLeftArrow.getValue()) {
            lineOne += "x|";
        } else {
            lineOne += "  |";
        }
        if (participantGP.upArrow.getValue()) {
            lineOne += "^|";
        } else {
            lineOne += "  |";
        }
        if (participantGP.topRightArrow.getValue()) {
            lineOne += "o";
        }

        String lineTwo = "";
        if (participantGP.leftArrow.getValue()) {
            lineTwo += "<|  |";
        } else {
            lineTwo += "  |  |";
        }
        if (participantGP.rightArrow.getValue()) {
            lineTwo += ">";
        }

        String lineThree;
        if (participantGP.downArrow.getValue()) {
            lineThree = "  |*|  ";
        } else {
            lineThree = "  |  |  ";
        }

        telemetry.addLine(lineOne);
        telemetry.addLine(lineTwo);
        telemetry.addLine(lineThree);
    }
}
