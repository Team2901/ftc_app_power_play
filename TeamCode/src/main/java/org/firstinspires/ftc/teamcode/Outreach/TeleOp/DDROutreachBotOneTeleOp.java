package org.firstinspires.ftc.teamcode.Outreach.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Outreach.Hardware.OutreachBotOneHardware;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.BooleanButton;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.DDRGamepad;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.Utility.CountDownTimer;

@TeleOp(name = "DDR Outreach Bot One Teleop", group = "Outreach")
public class DDROutreachBotOneTeleOp extends OpMode {
    final static int WAITING = 0;
    final static int PWRUP_UP1 = 1;
    final static int PWRUP_UP2 = 2;
    final static int PWRUP_UP3 = 3;
    final static int PWRUP_FINISHED = 4;
    final static int PWRUP_TAPTIME = 1000;
    OutreachBotOneHardware robot = new OutreachBotOneHardware();
    CountDownTimer countDownTimer = new CountDownTimer(ElapsedTime.Resolution.MILLISECONDS);
    CountDownTimer boostTimer = new CountDownTimer(ElapsedTime.Resolution.MILLISECONDS);
    CountDownTimer tapCountDownTimer = new CountDownTimer(ElapsedTime.Resolution.MILLISECONDS);
    /*
    CountDownTimer maxBoostTimer = new CountdownTimer(ElapsedTime.Resolution.MILLISECONDS);
     */
    ElapsedTime timer = new ElapsedTime();
    boolean isClawOpen = false;
    boolean override = false;
    int difficultyMode = 1;
    String[] difficultyNames = {"Beginner", "Intermediate", "Lawsuit"};
    DDRGamepad participantGP;
    ImprovedGamepad gameMasterGP;

    int konamiCodeProgress = 0;
    int beginnerKonamiCodeProgress = 0;
    int cardinalCodeProgress = 0;
    int swiftStepProgress = 0;
    int grapeVineProgress = 0;
    int hopScotchProgress = 0;

    int powerUpProgress = 0;
    double boostPower = 1;
    boolean isBoost = false;

    boolean isActive = false;

    double participantLeftPower = 0;
    double participantRightPower = 0;

    double gmLeftPower = 0;
    double gmRightPower = 0;

    @Override
    public void init() {
        robot.init(hardwareMap);

        participantGP = new DDRGamepad(this.gamepad1, this.timer, "GP1");
        gameMasterGP = new ImprovedGamepad(this.gamepad2, this.timer, "GP2");
    }

    @Override
    public void loop() {
        participantGP.update();
        gameMasterGP.update();

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

        if (this.participantGP.leftArrow.getValue() && this.participantGP.upArrow.getValue()) {
            participantLeftPower = 0.75;
            participantRightPower = 1;

        } //Top right + Up = arc clockwise
        //Left power = 1, Right power = 0.75
        else if (this.participantGP.rightArrow.getValue() && this.participantGP.upArrow.getValue()) {
            participantLeftPower = 1;
            participantRightPower = 0.75;

        } //Top left = counterclockwise
        //Left power = -0.75, Right power = 0.75
        else if (this.participantGP.leftArrow.getValue()) {
            participantLeftPower = -0.75;
            participantRightPower = 0.75;

        } //Top right = clockwise
        //Left power = 0.75, Right power = -0.75
        else if (this.participantGP.rightArrow.getValue()) {
            participantLeftPower = 0.75;
            participantRightPower = -0.75;
        } //Up = straight
        //Left and right motors same power 0.75
        else if (this.participantGP.upArrow.getValue()) {
            participantLeftPower = 0.75;
            participantRightPower = 0.75;
        }
        else if (this.participantGP.downArrow.getValue()){
            participantLeftPower = -0.75;
            participantRightPower = -0.75;
        }
        else {
            participantLeftPower = 0;
            participantRightPower = 0;
        }

        if(this.participantGP.topRightArrow.getValue()){
            isClawOpen = true;
        }else if(this.participantGP.topLeftArrow.getValue()){
            isClawOpen = false;
        }else if(this.gameMasterGP.y.isInitialPress()){
            isClawOpen = !isClawOpen;
        }

        isPowerUpComplete();
        if(boostTimer.getRemainingTime() == 0){
            boostPower = 1;
            isBoost = false;
        }

        /*
        isCardinalCodeComplete();
        if(boostTimer.getRemainingTime() == 0){
            boostPower = 1;
         */

        final boolean participantInput = participantGP.areButtonsActive();

        if (isClawOpen) {
            robot.claw.setPosition(0.75);
        } else {
            robot.claw.setPosition(0);
        }
        if (participantInput && !override) {
            if (difficultyMode == 0) {
                participantLeftPower /= 3;
                participantRightPower /= 3;
            } else if (difficultyMode == 1) {
                participantLeftPower *= 2.0 / 3;
                participantRightPower *= 2.0 / 3;
            }
            participantLeftPower *= boostPower;
            participantRightPower *= boostPower;
            // Sets power to motors
            power(participantLeftPower, participantRightPower);
        } else {
            power(gmLeftPower, gmRightPower);
        }

        telemetryDDRGraphic();
        telemetry.addData("Override", override);
        telemetry.addData("Is Active", isActive);
        telemetry.addData("Mode", difficultyNames[difficultyMode]);
        telemetry.addData("Participant Input", participantInput);
        telemetry.addData("Konami Code Progress", konamiCodeProgress);
        telemetry.addData("Beginner Konami Code Progress", beginnerKonamiCodeProgress);
        telemetry.addData("Power Up Progress", powerUpProgress);
        telemetry.addData("Boosting", isBoost);
        telemetry.addData("Left Power", robot.leftDrive.getPower());
        telemetry.addData("Right Power", robot.rightDrive.getPower());
        telemetry.update();
    }

    public void power(double left, double right) {
        robot.leftDrive.setPower(-left);
        robot.rightDrive.setPower(-right);
    }

    public void isPowerUpComplete() {
        if (!participantGP.areButtonsInitialPress()) {
            return;
        }
        switch (powerUpProgress) {
            case WAITING: {
                if (this.participantGP.upArrow.isInitialPress()) {
                    powerUpProgress = PWRUP_UP1;
                    tapCountDownTimer.setTargetTime(PWRUP_TAPTIME);
                } else {
                    powerUpProgress = WAITING;
                }
                break;
            }
            case PWRUP_UP1: {
                if (this.participantGP.upArrow.isInitialPress() && tapCountDownTimer.hasRemainingTime()) {
                    powerUpProgress = PWRUP_UP2;
                    tapCountDownTimer.setTargetTime(PWRUP_TAPTIME);
                } else {
                    powerUpProgress = WAITING;
                }
                break;
            }
            case PWRUP_UP2: {
                if (this.participantGP.upArrow.isInitialPress() && tapCountDownTimer.hasRemainingTime()) {
                    powerUpProgress = PWRUP_UP3;
                    tapCountDownTimer.setTargetTime(PWRUP_TAPTIME);
                } else {
                    powerUpProgress = WAITING;
                }
                break;
            }
            case PWRUP_UP3: {
                if (this.participantGP.rightArrow.isInitialPress() && tapCountDownTimer.hasRemainingTime()) {
                    powerUpProgress = PWRUP_FINISHED;
                    tapCountDownTimer.setTargetTime(PWRUP_TAPTIME);
                } else {
                    powerUpProgress = WAITING;
                }
                break;
            }
            case PWRUP_FINISHED: {
                boostPower = 2;
                boostTimer.setTargetTime(15000);
                isBoost = true;
                powerUpProgress = WAITING;
            }
        }


        /*
        if (cardinalCodeProgressProgress == 0) {
            cardinalCodeProgressProgress = this.participantGP.upArrow.isInitialPress() ? 1:0;
        } else if (cardinalCodeProgressProgress == 1) {
            cardinalCodeProgressProgress = this.participantGP.downArrow.isInitialPress() ? 2:0;
        } else if (cardinalCodeProgressProgress == 2) {
            cardinalCodeProgressProgress = this.participantGP.leftArrow.isInitialPress() ? 3:0;
        } else if (cardinalCodeProgressProgress == 3) {
            cardinalCodeProgressProgress = this.participantGP.rightArrow.isInitialPress() ? 4:0;
        } else if (cardinalCodeProgressProgress == 4) {
            cardinalCodeProgressProgress = this.participantGP.topLeftArrow.isInitialPress() ? 5:0;
        }
        if (cardinalCodeProgressProgress == 5) {
            boostPower = 1.25; ???
            maxBoostTimer.setTargetTime(15000);
            cardinalCodeProgressProgress = 0;

         */

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
