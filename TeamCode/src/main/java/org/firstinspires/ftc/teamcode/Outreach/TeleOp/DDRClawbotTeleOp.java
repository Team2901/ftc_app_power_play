package org.firstinspires.ftc.teamcode.Outreach.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Shared.Gamepad.DDRGamepad;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.Outreach.Hardware.ClawbotHardware;
import org.firstinspires.ftc.teamcode.Utility.CountDownTimer;

/**
 * Created by Kearneyg20428 on 2/7/2017.
 */

@TeleOp(name = "DDR Clawbot", group = "Outreach")
public class DDRClawbotTeleOp extends OpMode {

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
        isKonamiCodeComplete();
        isBeginnerKonamiCodeComplete();
        isCardinalCodeComplete();
        isSwiftStepComplete();
        isGrapeVineComplete();
        isHopScotchComplete();

        boolean isDancing = danceRoutine(isActive);

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
        if (!isDancing) {
            //If isClawOpen is true, opens the claw, otherwise it closes the claw
            if (isClawOpen) {
                robot.claw.setPosition(.5);
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
                // Sets power to motors
                power(participantLeftPower, participantRightPower);
                armPower(armTarget);
            } else {
                power(gmLeftPower, gmRightPower);
                armPower(armTarget);
            }
        }

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

    public void safeWait(int milliseconds) {
        timer.reset();
        if (timer.milliseconds() < milliseconds) {
        }
    }

    //only run dance routine if timer is going
    public boolean danceRoutine(boolean active) {
        if (active && countDownTimer.hasRemainingTime()) {

            if (robot.potentiometer.getVoltage() > robot.DANCE_ARM_UP_VOLTAGE && countDownTimer.getRemainingTime() > 7000) {
                //move claw all of the way up
                robot.armMotor.setPower(ClawbotHardware.ARM_UP_POWER);
            } else if (robot.potentiometer.getVoltage() < robot.DANCE_ARM_DOWN_VOLTAGE && countDownTimer.getRemainingTime() < 3000) {
                //move claw all of the way up
                robot.armMotor.setPower(ClawbotHardware.ARM_DOWN_POWER);
            } else {
                robot.armMotor.setPower(0);
            }

            if (countDownTimer.getRemainingTime() < 7000 && countDownTimer.getRemainingTime() > 6750) {
                robot.claw.setPosition(robot.MID_SERVO - robot.MAX_SAFE_CLAW_OFFSET);
            } else if (countDownTimer.getRemainingTime() > 6500) {
                robot.claw.setPosition(robot.MID_SERVO - robot.MIN_SAFE_CLAW_OFFSET);
            } else if (countDownTimer.getRemainingTime() > 6250) {
                robot.claw.setPosition(robot.MID_SERVO - robot.MAX_SAFE_CLAW_OFFSET);
            } else if (countDownTimer.getRemainingTime() > 6000) {
                robot.claw.setPosition(robot.MID_SERVO - robot.MIN_SAFE_CLAW_OFFSET);
            } else if (countDownTimer.getRemainingTime() > 5750) {
                robot.claw.setPosition(robot.MID_SERVO - robot.MAX_SAFE_CLAW_OFFSET);
            } else if (countDownTimer.getRemainingTime() > 5500) {
                robot.claw.setPosition(robot.MID_SERVO - robot.MIN_SAFE_CLAW_OFFSET);
            }

            if (countDownTimer.getRemainingTime() < 5500 && countDownTimer.getRemainingTime() > 3000) {
                power(1, -1);
            } else {
                power(0, 0);
            }

            return true;
        }
        return false;
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

    public void isSwiftStepComplete() {
        if (!participantGP.areButtonsInitialPress()) {
            if (swiftStepProgress == 4) {
                swiftStepDance(isActive);
                swiftStepProgress = 0;
            }
            return;
        }
        if (swiftStepProgress == 0) {
            swiftStepProgress = this.participantGP.topRightArrow.isInitialPress() ? 1 : 0;
        } else if (swiftStepProgress == 1) {
            swiftStepProgress = this.participantGP.downArrow.isInitialPress() ? 2 : 0;
        } else if (swiftStepProgress == 2) {
            swiftStepProgress = this.participantGP.upArrow.isInitialPress() ? 3 : 0;
        } else if (swiftStepProgress == 3) {
            swiftStepProgress = this.participantGP.topLeftArrow.isInitialPress() ? 4 : 0;
        } else if (swiftStepProgress == 4) {
            swiftStepProgress = 0;
        }
    }

    public void isCardinalCodeComplete() {
        if (!participantGP.areButtonsInitialPress()) {
            if (cardinalCodeProgress == 4) {
                cardinalDirectionsDance(isActive);
                cardinalCodeProgress = 0;
            }
            return;
        }
        if (cardinalCodeProgress == 0) {
            cardinalCodeProgress = this.participantGP.upArrow.isInitialPress() ? 1 : 0;
        } else if (cardinalCodeProgress == 1) {
            cardinalCodeProgress = this.participantGP.rightArrow.isInitialPress() ? 2 : 0;
        } else if (cardinalCodeProgress == 2) {
            cardinalCodeProgress = this.participantGP.downArrow.isInitialPress() ? 3 : 0;
        } else if (cardinalCodeProgress == 3) {
            cardinalCodeProgress = this.participantGP.leftArrow.isInitialPress() ? 4 : 0;
        } else if (cardinalCodeProgress == 4) {
            cardinalCodeProgress = 0;

        }
    }

    public void isBeginnerKonamiCodeComplete() {
        if (!participantGP.areButtonsInitialPress()) {
            if (beginnerKonamiCodeProgress == 5) {
                beginnerKonamiDance(isActive);
                beginnerKonamiCodeProgress = 0;
            }
            return;
        }
        if (beginnerKonamiCodeProgress == 0) {
            beginnerKonamiCodeProgress = this.participantGP.upArrow.isInitialPress() ? 1 : 0;
        } else if (beginnerKonamiCodeProgress == 1) {
            beginnerKonamiCodeProgress = this.participantGP.downArrow.isInitialPress() ? 2 : 0;
        } else if (beginnerKonamiCodeProgress == 2) {
            beginnerKonamiCodeProgress = this.participantGP.leftArrow.isInitialPress() ? 3 : 0;
        } else if (beginnerKonamiCodeProgress == 3) {
            beginnerKonamiCodeProgress = this.participantGP.rightArrow.isInitialPress() ? 4 : 0;
        } else if (beginnerKonamiCodeProgress == 4) {
            if (this.participantGP.topLeftArrow.isInitialPress()) {
                beginnerKonamiCodeProgress = 5;
            } else {
                beginnerKonamiCodeProgress = 0;
            }
        } else if (beginnerKonamiCodeProgress == 5) {
            beginnerKonamiCodeProgress = 0;
        }
    }

    public void isGrapeVineComplete() {
        if (!participantGP.areButtonsInitialPress()) {
            if (grapeVineProgress == 5) {
                grapeVineDance(isActive);
                grapeVineProgress = 0;
            }
            return;
        } else if (grapeVineProgress == 0) {
            grapeVineProgress = this.participantGP.leftArrow.isInitialPress() ? 1 : 0;
        } else if (grapeVineProgress == 1) {
            grapeVineProgress = this.participantGP.upArrow.isInitialPress() ? 2 : 0;
        } else if (grapeVineProgress == 2) {
            grapeVineProgress = this.participantGP.downArrow.isInitialPress() ? 3 : 0;
        } else if (grapeVineProgress == 3) {
            grapeVineProgress = this.participantGP.topLeftArrow.isInitialPress() ? 4 : 0;
        } else if (grapeVineProgress == 4) {
            grapeVineProgress = this.participantGP.topRightArrow.isInitialPress() ? 5 : 0;
        } else if (grapeVineProgress == 5) {
            grapeVineProgress = 0;
        }
    }

    public void isHopScotchComplete() {
        if (!participantGP.areButtonsInitialPress()) {
            if (hopScotchProgress == 5) {
                hopScotchDance(isActive);
                hopScotchProgress = 0;
            }
            return;
        } else if (hopScotchProgress == 0) {
            hopScotchProgress = this.participantGP.upArrow.isInitialPress() ? 1 : 0;
        } else if (hopScotchProgress == 1) {
            hopScotchProgress = this.participantGP.topRightArrow.isInitialPress() ? 2 : 0;
        } else if (hopScotchProgress == 2) {
            if (this.participantGP.leftArrow.isPressed() && this.participantGP.rightArrow.isPressed()) {
                hopScotchProgress = 3;
            } else if (!this.participantGP.leftArrow.isPressed() && !this.participantGP.rightArrow.isPressed()) {
                hopScotchProgress = 0;
            }
        } else if (hopScotchProgress == 3) {
            hopScotchProgress = this.participantGP.downArrow.isInitialPress() ? 4 : 0;
        } else if (hopScotchProgress == 4) {
            if (this.participantGP.topLeftArrow.isPressed() && this.participantGP.topRightArrow.isPressed()) {
                hopScotchProgress = 5;
            } else if (!this.participantGP.topLeftArrow.isPressed() && !this.participantGP.topRightArrow.isPressed()) {
                hopScotchProgress = 0;
            }
        } else if (hopScotchProgress == 5) {
            hopScotchProgress = 0;
        }
    }

    public void isKonamiCodeComplete() {
        if (!participantGP.areButtonsInitialPress()) {
            //If no buttons are being pressed, dont check anything
            return;
        }
        if (konamiCodeProgress == 0) {
            konamiCodeProgress = this.participantGP.upArrow.isInitialPress() ? 1 : 0;
        } else if (konamiCodeProgress == 1) {
            konamiCodeProgress = this.participantGP.upArrow.isInitialPress() ? 2 : 0;
        } else if (konamiCodeProgress == 2) {
            konamiCodeProgress = this.participantGP.downArrow.isInitialPress() ? 3 : 0;
        } else if (konamiCodeProgress == 3) {
            konamiCodeProgress = this.participantGP.downArrow.isInitialPress() ? 4 : 0;
        } else if (konamiCodeProgress == 4) {
            konamiCodeProgress = this.participantGP.leftArrow.isInitialPress() ? 5 : 0;
        } else if (konamiCodeProgress == 5) {
            konamiCodeProgress = this.participantGP.rightArrow.isInitialPress() ? 6 : 0;
        } else if (konamiCodeProgress == 6) {
            konamiCodeProgress = this.participantGP.leftArrow.isInitialPress() ? 7 : 0;
        } else if (konamiCodeProgress == 7) {
            konamiCodeProgress = this.participantGP.rightArrow.isInitialPress() ? 8 : 0;
        } else if (konamiCodeProgress == 8) {
            konamiCodeProgress = this.participantGP.topLeftArrow.isInitialPress() ? 9 : 0;
        } else if (konamiCodeProgress == 9) {
            konamiCodeProgress = this.participantGP.topRightArrow.isInitialPress() ? 10 : 0;
        } else if (konamiCodeProgress == 10) {
            if (this.participantGP.leftArrow.isPressed() && this.participantGP.rightArrow.isPressed()) {
                konamiCodeProgress = 11;
                countDownTimer.setTargetTime(10000);
            } else if (!this.participantGP.leftArrow.isPressed() && !this.participantGP.rightArrow.isPressed()) {
                konamiCodeProgress = 0;
            }
        } else if (konamiCodeProgress == 11) {
            konamiCodeProgress = 0;
        }
    }
    public boolean swiftStepDance(boolean active) {
        if (active) {
            //Put dance code here
            robot.rightMotor.setPower(.75);
            robot.leftMotor.setPower(.75);
            safeWait(1000);
            robot.rightMotor.setPower(.75);
            robot.leftMotor.setPower(0);
            safeWait(1000);
            robot.rightMotor.setPower(.75);
            robot.leftMotor.setPower(.75);
            safeWait(1000);
            robot.rightMotor.setPower(.75);
            robot.leftMotor.setPower(0);
            safeWait(1000);
            robot.rightMotor.setPower(.75);
            robot.leftMotor.setPower(.75);
            safeWait(1000);
            robot.rightMotor.setPower(.75);
            robot.leftMotor.setPower(0);
            safeWait(1000);
            robot.rightMotor.setPower(.75);
            robot.leftMotor.setPower(.75);
            safeWait(1000);
            robot.rightMotor.setPower(.75);
            robot.leftMotor.setPower(0);
            safeWait(1000);
            robot.rightMotor.setPower(0);
            robot.leftMotor.setPower(0);
            return true;
        }
        return false;
    }

    public boolean cardinalDirectionsDance(boolean active) {
        if (active) {
            robot.rightMotor.setPower(.75);
            robot.leftMotor.setPower(.25);
            safeWait(10000);
            robot.rightMotor.setPower(0);
            robot.leftMotor.setPower(0);
            return true;
        }
        return false;
    }

    public boolean beginnerKonamiDance(boolean active) {
        if (active) {
            //Put dance code here
            robot.rightMotor.setPower(.75);
            robot.leftMotor.setPower(.25);
            safeWait(2000);
            robot.rightMotor.setPower(.25);
            robot.leftMotor.setPower(.75);
            safeWait(4000);
            robot.rightMotor.setPower(.75);
            robot.leftMotor.setPower(.25);
            safeWait(2000);
            return true;
        }
        return false;
    }

    public boolean grapeVineDance(boolean active) {
        if (active) {
            //Put dance code here
            robot.rightMotor.setPower(-.5);
            robot.leftMotor.setPower(-.5);
            robot.claw.setPosition(robot.MID_SERVO - robot.MAX_SAFE_CLAW_OFFSET);
            safeWait(2000);
            robot.claw.setPosition(robot.MID_SERVO - robot.MIN_SAFE_CLAW_OFFSET);
            safeWait(2000);

            return true;
        }
        return false;
    }

    public boolean hopScotchDance(boolean active) {
        if (active) {
            //Put dance code here
            //arm.setPower(.15);
            if(robot.potentiometer.getVoltage() > robot.DANCE_ARM_DOWN_VOLTAGE){
                robot.armMotor.setPower(ClawbotHardware.ARM_DOWN_POWER);
            }
            robot.claw.setPosition(robot.MID_SERVO - robot.MIN_SAFE_CLAW_OFFSET);
            robot.rightMotor.setPower(.75);
            robot.leftMotor.setPower(.75);
            safeWait(500);
            //arm.setPower(.075);
            if(robot.potentiometer.getVoltage() > robot.DANCE_ARM_UP_VOLTAGE){
                robot.armMotor.setPower(ClawbotHardware.ARM_UP_POWER);
            }
            robot.claw.setPosition(robot.MID_SERVO - robot.MAX_SAFE_CLAW_OFFSET);
            robot.rightMotor.setPower(.75);
            robot.leftMotor.setPower(.25);
            safeWait(500);
            //arm.setPower(.15);
            if(robot.potentiometer.getVoltage() > robot.DANCE_ARM_DOWN_VOLTAGE){
                robot.armMotor.setPower(ClawbotHardware.ARM_DOWN_POWER);
            }
            robot.claw.setPosition(robot.MID_SERVO - robot.MIN_SAFE_CLAW_OFFSET);
            robot.rightMotor.setPower(.75);
            robot.leftMotor.setPower(.75);
            safeWait(500);
            //arm.setPower(.075);
            if(robot.potentiometer.getVoltage() > robot.DANCE_ARM_UP_VOLTAGE){
                robot.armMotor.setPower(ClawbotHardware.ARM_UP_POWER);
            }
            robot.claw.setPosition(robot.MID_SERVO - robot.MAX_SAFE_CLAW_OFFSET);
            robot.rightMotor.setPower(-.25);
            robot.leftMotor.setPower(.75);
            safeWait(500);
            //arm.setPower(.15);
            if(robot.potentiometer.getVoltage() > robot.DANCE_ARM_DOWN_VOLTAGE){
                robot.armMotor.setPower(ClawbotHardware.ARM_DOWN_POWER);
            }
            robot.claw.setPosition(robot.MID_SERVO - robot.MIN_SAFE_CLAW_OFFSET);
            robot.rightMotor.setPower(.75);
            robot.leftMotor.setPower(.75);
            safeWait(500);
            //arm.setPower(.075);
            if(robot.potentiometer.getVoltage() > robot.DANCE_ARM_UP_VOLTAGE){
                robot.armMotor.setPower(ClawbotHardware.ARM_UP_POWER);
            }
            robot.claw.setPosition(robot.MID_SERVO - robot.MAX_SAFE_CLAW_OFFSET);
            robot.rightMotor.setPower(.75);
            robot.leftMotor.setPower(.25);
            safeWait(500);
            //arm.setPower(.15);
            if(robot.potentiometer.getVoltage() > robot.DANCE_ARM_DOWN_VOLTAGE){
                robot.armMotor.setPower(ClawbotHardware.ARM_DOWN_POWER);
            }
            robot.claw.setPosition(robot.MID_SERVO - robot.MIN_SAFE_CLAW_OFFSET);
            robot.rightMotor.setPower(.75);
            robot.leftMotor.setPower(.75);
            safeWait(500);
            //arm.setPower(.075);
            if(robot.potentiometer.getVoltage() > robot.DANCE_ARM_UP_VOLTAGE){
                robot.armMotor.setPower(ClawbotHardware.ARM_UP_POWER);
            }
            robot.claw.setPosition(robot.MID_SERVO - robot.MAX_SAFE_CLAW_OFFSET);
            robot.rightMotor.setPower(-.25);
            robot.leftMotor.setPower(.75);
            safeWait(500);
            //arm.setPower(.15);
            if(robot.potentiometer.getVoltage() > robot.DANCE_ARM_DOWN_VOLTAGE){
                robot.armMotor.setPower(ClawbotHardware.ARM_DOWN_POWER);
            }
            robot.claw.setPosition(robot.MID_SERVO - robot.MIN_SAFE_CLAW_OFFSET);
            robot.rightMotor.setPower(.75);
            robot.leftMotor.setPower(.75);
            safeWait(500);
            //arm.setPower(.075);
            if(robot.potentiometer.getVoltage() > robot.DANCE_ARM_UP_VOLTAGE){
                robot.armMotor.setPower(ClawbotHardware.ARM_UP_POWER);
            }
            robot.claw.setPosition(robot.MID_SERVO - robot.MAX_SAFE_CLAW_OFFSET);
            robot.rightMotor.setPower(.75);
            robot.leftMotor.setPower(.25);
            safeWait(500);
            //arm.setPower(.15);
            if(robot.potentiometer.getVoltage() > robot.DANCE_ARM_DOWN_VOLTAGE){
                robot.armMotor.setPower(ClawbotHardware.ARM_DOWN_POWER);
            }
            robot.claw.setPosition(robot.MID_SERVO - robot.MIN_SAFE_CLAW_OFFSET);
            robot.rightMotor.setPower(.75);
            robot.leftMotor.setPower(.75);
            safeWait(500);
            //arm.setPower(.075);
            if(robot.potentiometer.getVoltage() > robot.DANCE_ARM_UP_VOLTAGE){
                robot.armMotor.setPower(ClawbotHardware.ARM_UP_POWER);
            }
            robot.claw.setPosition(robot.MID_SERVO - robot.MAX_SAFE_CLAW_OFFSET);
            robot.rightMotor.setPower(-.25);
            robot.leftMotor.setPower(.75);
            safeWait(500);
            //arm.setPower(.15);
            if(robot.potentiometer.getVoltage() > robot.DANCE_ARM_DOWN_VOLTAGE){
                robot.armMotor.setPower(ClawbotHardware.ARM_DOWN_POWER);
            }
            robot.claw.setPosition(robot.MID_SERVO - robot.MIN_SAFE_CLAW_OFFSET);
            robot.rightMotor.setPower(.75);
            robot.leftMotor.setPower(.75);
            safeWait(500);
            //arm.setPower(.075);
            if(robot.potentiometer.getVoltage() > robot.DANCE_ARM_UP_VOLTAGE){
                robot.armMotor.setPower(ClawbotHardware.ARM_UP_POWER);
            }
            robot.claw.setPosition(robot.MID_SERVO - robot.MAX_SAFE_CLAW_OFFSET);
            robot.rightMotor.setPower(.75);
            robot.leftMotor.setPower(.25);
            safeWait(500);
            //arm.setPower(.15);
            if(robot.potentiometer.getVoltage() > robot.DANCE_ARM_DOWN_VOLTAGE){
                robot.armMotor.setPower(ClawbotHardware.ARM_DOWN_POWER);
            }
            robot.claw.setPosition(robot.MID_SERVO - robot.MIN_SAFE_CLAW_OFFSET);
            robot.rightMotor.setPower(.75);
            robot.leftMotor.setPower(.75);
            safeWait(500);
            //arm.setPower(.075);
            if(robot.potentiometer.getVoltage() > robot.DANCE_ARM_UP_VOLTAGE){
                robot.armMotor.setPower(ClawbotHardware.ARM_UP_POWER);
            }
            robot.claw.setPosition(robot.MID_SERVO - robot.MAX_SAFE_CLAW_OFFSET);
            robot.rightMotor.setPower(-.25);
            robot.leftMotor.setPower(.75);
            safeWait(500);
            //arm.setPower(.15);
            if(robot.potentiometer.getVoltage() > robot.DANCE_ARM_DOWN_VOLTAGE){
                robot.armMotor.setPower(ClawbotHardware.ARM_DOWN_POWER);
            }
            robot.claw.setPosition(robot.MID_SERVO - robot.MIN_SAFE_CLAW_OFFSET);
            robot.rightMotor.setPower(.75);
            robot.leftMotor.setPower(.75);
            safeWait(500);
            //arm.setPower(.075);
            if(robot.potentiometer.getVoltage() > robot.DANCE_ARM_UP_VOLTAGE){
                robot.armMotor.setPower(ClawbotHardware.ARM_UP_POWER);
            }
            robot.claw.setPosition(robot.MID_SERVO - robot.MAX_SAFE_CLAW_OFFSET);
            robot.rightMotor.setPower(.75);
            robot.leftMotor.setPower(.25);
            safeWait(500);
            //arm.setPower(.15);
            if(robot.potentiometer.getVoltage() > robot.DANCE_ARM_DOWN_VOLTAGE){
                robot.armMotor.setPower(ClawbotHardware.ARM_DOWN_POWER);
            }
            robot.claw.setPosition(robot.MID_SERVO - robot.MIN_SAFE_CLAW_OFFSET);
            robot.rightMotor.setPower(.75);
            robot.leftMotor.setPower(.75);
            safeWait(500);
            //arm.setPower(.075);
            if(robot.potentiometer.getVoltage() > robot.DANCE_ARM_UP_VOLTAGE){
                robot.armMotor.setPower(ClawbotHardware.ARM_UP_POWER);
            }
            robot.claw.setPosition(robot.MID_SERVO - robot.MAX_SAFE_CLAW_OFFSET);
            robot.rightMotor.setPower(-.25);
            robot.leftMotor.setPower(.75);
            safeWait(500);
            robot.rightMotor.setPower(0);
            robot.leftMotor.setPower(0);
            return true;
        }
        return false;
    }

    public void armPower(int target){
        robot.armMotor.setPower((target - robot.armMotor.getCurrentPosition())/500);
    }
}
