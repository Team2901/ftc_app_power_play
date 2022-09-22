package org.firstinspires.ftc.teamcode.Shared.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Shared.Gamepad.DDRGamepad;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.Utility.CountDownTimer;

@TeleOp(name = "DDR Jeffrey Bezos", group = "Shared")
public class DDRJefferyBezosTeleOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor leftDrive;
    public DcMotor rightDrive;
    public Servo claw;
    public DcMotor arm;

    CountDownTimer countDownTimer = new CountDownTimer(ElapsedTime.Resolution.MILLISECONDS);
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
    boolean isActive = true;
    boolean isClawOpen = false;

    int armPosition = 0;
    double participantLeftPower = 0;
    double participantRightPower = 0;

    double gmLeftPower = 0;
    double gmRightPower = 0;

    @Override
    public void init() {
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");
        claw = hardwareMap.servo.get("claw");
        arm = hardwareMap.dcMotor.get("arm");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        participantGP = new DDRGamepad(this.gamepad1, this.timer, "GP1");
        gameMasterGP = new ImprovedGamepad(this.gamepad2, this.timer, "GP2");
    }

    @Override
    public void loop() {
        participantGP.update();
        gameMasterGP.update();

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

        gmLeftPower = gameMasterGP.left_stick_y.getValue();
        gmRightPower = gameMasterGP.left_stick_y.getValue();

        gmLeftPower += gameMasterGP.left_stick_x.getValue();
        gmRightPower -= gameMasterGP.left_stick_x.getValue();

        double maxPower = Math.max(Math.abs(gmLeftPower), Math.abs(gmRightPower));

        // Adjusts speeds for the arcs.
        if (maxPower > 1) {
            gmRightPower /= Math.abs(gmRightPower);
            gmLeftPower /= Math.abs(gmLeftPower);
        }

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
        } else {
            participantLeftPower = 0;
            participantRightPower = 0;
        }

        if(this.participantGP.topLeftArrow.isPressed() && !override && !this.participantGP.startButton.getValue()){
            armPosition = 280/3;
        }else if(this.gameMasterGP.left_bumper.isPressed()){
            armPosition = 280/3;
        }

        if(this.participantGP.topRightArrow.isPressed() && !override && !this.participantGP.startButton.getValue()){
            armPosition = 25;
        }else if(this.gameMasterGP.right_bumper.isPressed()){
            armPosition = 25;
        }

        if(this.participantGP.downArrow.isInitialPress() && !override && !this.participantGP.startButton.getValue()) {
            isClawOpen = !isClawOpen;
        } else if(this.gameMasterGP.y.isInitialPress()) {
            isClawOpen = !isClawOpen;
        }

        final boolean participantInput = participantGP.areButtonsActive();

        telemetryDDRGraphic();
        telemetry.addData("Override", override);
        telemetry.addData("Is Active", isActive);
        telemetry.addData("Mode", difficultyNames[difficultyMode]);
        telemetry.addData("Participant Input", participantInput);
        telemetry.addData("Swift Step Progress", swiftStepProgress);
        telemetry.addData("Cardinal Directions Progress", cardinalCodeProgress);
        telemetry.addData("Beginner Konami Code Progress", beginnerKonamiCodeProgress);
        telemetry.addData("Grape Vine Progress", grapeVineProgress);
        telemetry.addData("Hop Scotch Progress", hopScotchProgress);
        telemetry.addData("Konami Code Progress", konamiCodeProgress);
        telemetry.addData("Right Motor", rightDrive.getCurrentPosition());
        telemetry.addData("Left Motor", leftDrive.getCurrentPosition());
        telemetry.addData("arm position",  arm.getCurrentPosition());
        telemetry.update();

        if (gameMasterGP.b.getValue()) {
            countDownTimer.setTargetTime(10000);
        }
        /*isKonamiCodeComplete();
        isBeginnerKonamiCodeComplete();
        isCardinalCodeComplete();
        isSwiftStepComplete();
        isGrapeVineComplete();
        isHopScotchComplete();*/
        //boolean isDancing = danceRoutine(isActive);

        if (isClawOpen) {
            claw.setPosition(.1);
        } else {
            claw.setPosition(.5);
        }

        armPower(armPosition);

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
        } else {
            power(gmLeftPower, gmRightPower);
        }
    }

    public void safeWait(int milliseconds) {
        runtime.reset();
        if(runtime.milliseconds() < milliseconds) {
        }
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
            if(cardinalCodeProgress == 4){
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
            if(grapeVineProgress == 5){
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
            if(hopScotchProgress == 5){
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
            if(konamiCodeProgress == 11){
                konamiCodeDance(isActive);
                konamiCodeProgress = 0;
            }
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
            } else if (!this.participantGP.leftArrow.isPressed() && !this.participantGP.rightArrow.isPressed()) {
                konamiCodeProgress = 0;
            }
        } else if (konamiCodeProgress == 11) {
            konamiCodeProgress = 0;
        }
    }

    /*
    public boolean danceRoutine(boolean active){
        if(active && (swiftStepDance(active) || cardinalDirectionsDance(active) ||
                beginnerKonamiDance(active) || grapeVineDance(active) ||
                hopScotchDance(active) || konamiCodeDance(active))){
            return true;
        }else{
            return false;
        }
    }
     */

    public boolean swiftStepDance(boolean active) {
        if (active) {
            //Put dance code here
            rightDrive.setPower(.75);
            leftDrive.setPower(.75);
            safeWait(1000);
            rightDrive.setPower(.75);
            leftDrive.setPower(0);
            safeWait(1000);
            rightDrive.setPower(.75);
            leftDrive.setPower(.75);
            safeWait(1000);
            rightDrive.setPower(.75);
            leftDrive.setPower(0);
            safeWait(1000);
            rightDrive.setPower(.75);
            leftDrive.setPower(.75);
            safeWait(1000);
            rightDrive.setPower(.75);
            leftDrive.setPower(0);
            safeWait(1000);
            rightDrive.setPower(.75);
            leftDrive.setPower(.75);
            safeWait(1000);
            rightDrive.setPower(.75);
            leftDrive.setPower(0);
            safeWait(1000);
            rightDrive.setPower(0);
            leftDrive.setPower(0);
            return true;
        }
        return false;
    }

    public boolean cardinalDirectionsDance(boolean active) {
        if (active) {
            rightDrive.setPower(.75);
            leftDrive.setPower(.25);
            safeWait(10000);
            rightDrive.setPower(0);
            leftDrive.setPower(0);
            return true;
        }
        return false;
    }

    public boolean beginnerKonamiDance(boolean active) {
        if (active) {
            //Put dance code here
            rightDrive.setPower(.75);
            leftDrive.setPower(.25);
            safeWait(2000);
            rightDrive.setPower(.25);
            leftDrive.setPower(.75);
            safeWait(4000);
            rightDrive.setPower(.75);
            leftDrive.setPower(.25);
            safeWait(2000);
            return true;
        }
        return false;
    }

    public boolean grapeVineDance(boolean active) {
        if (active) {
            //Put dance code here
            rightDrive.setPower(-.5);
            leftDrive.setPower(-.5);
            claw.setPosition(.5);
            safeWait(2000);
            claw.setPosition(.1);
            safeWait(2000);

            return true;
        }
        return false;
    }

    public boolean hopScotchDance(boolean active) {
        if (active) {
            //Put dance code here
            //arm.setPower(.15);
            armPosition = 25;
            claw.setPosition(0.1);
            rightDrive.setPower(.75);
            leftDrive.setPower(.75);
            safeWait(500);
            //arm.setPower(.075);
            armPosition = 280/3;
            claw.setPosition(0.5);
            rightDrive.setPower(.75);
            leftDrive.setPower(.25);
            safeWait(500);
            //arm.setPower(.15);
            armPosition = 25;
            claw.setPosition(.1);
            rightDrive.setPower(.75);
            leftDrive.setPower(.75);
            safeWait(500);
            //arm.setPower(.075);
            armPosition = 280/3;
            claw.setPosition(.5);
            rightDrive.setPower(-.25);
            leftDrive.setPower(.75);
            safeWait(500);
            //arm.setPower(.15);
            armPosition = 25;
            claw.setPosition(0.1);
            rightDrive.setPower(.75);
            leftDrive.setPower(.75);
            safeWait(500);
            //arm.setPower(.075);
            armPosition = 280/3;
            claw.setPosition(0.5);
            rightDrive.setPower(.75);
            leftDrive.setPower(.25);
            safeWait(500);
            //arm.setPower(.15);
            armPosition = 25;
            claw.setPosition(.1);
            rightDrive.setPower(.75);
            leftDrive.setPower(.75);
            safeWait(500);
            //arm.setPower(.075);
            armPosition = 280/3;
            claw.setPosition(.5);
            rightDrive.setPower(-.25);
            leftDrive.setPower(.75);
            safeWait(500);
            //arm.setPower(.15);
            armPosition = 25;
            claw.setPosition(0.1);
            rightDrive.setPower(.75);
            leftDrive.setPower(.75);
            safeWait(500);
            //arm.setPower(.075);
            armPosition = 280/3;
            claw.setPosition(0.5);
            rightDrive.setPower(.75);
            leftDrive.setPower(.25);
            safeWait(500);
            //arm.setPower(.15);
            armPosition = 25;
            claw.setPosition(.1);
            rightDrive.setPower(.75);
            leftDrive.setPower(.75);
            safeWait(500);
            //arm.setPower(.075);
            armPosition = 280/3;
            claw.setPosition(.5);
            rightDrive.setPower(-.25);
            leftDrive.setPower(.75);
            safeWait(500);
            //arm.setPower(.15);
            armPosition = 25;
            claw.setPosition(0.1);
            rightDrive.setPower(.75);
            leftDrive.setPower(.75);
            safeWait(500);
            //arm.setPower(.075);
            armPosition = 280/3;
            claw.setPosition(0.5);
            rightDrive.setPower(.75);
            leftDrive.setPower(.25);
            safeWait(500);
            //arm.setPower(.15);
            armPosition = 25;
            claw.setPosition(.1);
            rightDrive.setPower(.75);
            leftDrive.setPower(.75);
            safeWait(500);
            //arm.setPower(.075);
            armPosition = 280/3;
            claw.setPosition(.5);
            rightDrive.setPower(-.25);
            leftDrive.setPower(.75);
            safeWait(500);
            //arm.setPower(.15);
            armPosition = 25;
            claw.setPosition(0.1);
            rightDrive.setPower(.75);
            leftDrive.setPower(.75);
            safeWait(500);
            //arm.setPower(.075);
            armPosition = 280/3;
            claw.setPosition(0.5);
            rightDrive.setPower(.75);
            leftDrive.setPower(.25);
            safeWait(500);
            //arm.setPower(.15);
            armPosition = 25;
            claw.setPosition(.1);
            rightDrive.setPower(.75);
            leftDrive.setPower(.75);
            safeWait(500);
            //arm.setPower(.075);
            armPosition = 280/3;
            claw.setPosition(.5);
            rightDrive.setPower(-.25);
            leftDrive.setPower(.75);
            safeWait(500);
            rightDrive.setPower(0);
            leftDrive.setPower(0);
            return true;
        }
        return false;
    }

    public boolean konamiCodeDance(boolean active) {
        if (active) {
            //Put dance code here
            claw.setPosition(.5);
            //arm.setPower(.075);
            armPosition = 280/3;
            rightDrive.setPower(.75);
            leftDrive.setPower(.25);
            safeWait(500);
            claw.setPosition(.1);
            safeWait(500);
            //arm.setPower(.15);
            armPosition = 25;
            safeWait(500);
            claw.setPosition(.5);
            safeWait(500);
            claw.setPosition(.1);
            safeWait(500);
            claw.setPosition(.5);
            //arm.setPower(.075);
            armPosition = 280/3;
            rightDrive.setPower(.25);
            leftDrive.setPower(.75);
            safeWait(500);
            claw.setPosition(.1);
            safeWait(500);
            //arm.setPower(.15);
            armPosition = 25;
            safeWait(500);
            claw.setPosition(.5);
            safeWait(500);
            claw.setPosition(.1);
            safeWait(500);
            rightDrive.setPower(0);
            leftDrive.setPower(0);
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

    public void power(double left, double right) {
        leftDrive.setPower(-left);
        rightDrive.setPower(-right);
    }

    public void armPower(int target){
        arm.setPower(((target) - arm.getCurrentPosition())/15);
    }
}
