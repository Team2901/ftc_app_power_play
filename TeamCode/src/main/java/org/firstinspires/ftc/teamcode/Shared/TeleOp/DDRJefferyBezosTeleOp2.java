package org.firstinspires.ftc.teamcode.Shared.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.DDRGamepad;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;
import org.firstinspires.ftc.teamcode.Utility.CountDownTimer;

@TeleOp(name = "DDR Jeffrey Bezos 2", group = "Shared")
public class DDRJefferyBezosTeleOp2 extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor leftDrive;
    public DcMotor rightDrive;
    public Servo claw;
    public DcMotor arm;

    CountDownTimer countDownTimerKonami = new CountDownTimer(ElapsedTime.Resolution.MILLISECONDS);
    CountDownTimer countDownTimerSwift = new CountDownTimer(ElapsedTime.Resolution.MILLISECONDS);
    CountDownTimer countDownTimerBeginner = new CountDownTimer(ElapsedTime.Resolution.MILLISECONDS);
    CountDownTimer countDownTimerHop = new CountDownTimer(ElapsedTime.Resolution.MILLISECONDS);
    CountDownTimer countDownTimerCardinal = new CountDownTimer(ElapsedTime.Resolution.MILLISECONDS);
    CountDownTimer countDownTimerGrape = new CountDownTimer(ElapsedTime.Resolution.MILLISECONDS);
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

    public BNO055IMU imu;

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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Initialize the imu with the parameters above
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
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

        gmLeftPower = -gameMasterGP.left_stick_y.getValue();
        gmRightPower = -gameMasterGP.left_stick_y.getValue();

        gmLeftPower -= gameMasterGP.left_stick_x.getValue();
        gmRightPower += gameMasterGP.left_stick_x.getValue();

        double maxPower = Math.max(Math.abs(gmLeftPower), Math.abs(gmRightPower));

        // Adjusts speeds for the arcs.
        if (maxPower > 1) {
            gmRightPower /= Math.abs(gmRightPower);
            gmLeftPower /= Math.abs(gmLeftPower);
        }

        if (this.participantGP.rightArrow.getValue() && this.participantGP.leftArrow.getValue() && difficultyMode == 0) {
            participantLeftPower = 0;
            participantRightPower = 0;
        }
        else if (this.participantGP.rightArrow.getValue() && this.participantGP.downArrow.getValue()) {
            participantLeftPower = 1;
            participantRightPower = 0.75;
        }
        else if (this.participantGP.leftArrow.getValue() && this.participantGP.downArrow.getValue()) {
            participantLeftPower = 0.75;
            participantRightPower = 1;
        }
        //Topleft + Up = arc counterclockwise
        //Left power = 0.75, Right power = 1
        else if (this.participantGP.leftArrow.getValue() && this.participantGP.upArrow.getValue()) {
            participantLeftPower = -0.75;
            participantRightPower = -1;

        } //Top right + Up = arc clockwise
        //Left power = 1, Right power = 0.75
        else if (this.participantGP.rightArrow.getValue() && this.participantGP.upArrow.getValue()) {
            participantLeftPower = -1;
            participantRightPower = -0.75;

        }
        else if (this.participantGP.downArrow.getValue() && difficultyMode > 0) {
            participantLeftPower = 1;
            participantRightPower = 1;

        }
        //Top left = counterclockwise
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
            participantLeftPower = -1;
            participantRightPower = -1;
        } else {
            participantLeftPower = 0;
            participantRightPower = 0;
        }


        if(this.participantGP.topLeftArrow.isInitialPress() && !override && !this.participantGP.startButton.getValue()) {
            isClawOpen = false;
        } else if(this.participantGP.topRightArrow.isInitialPress() && !override && !this.participantGP.startButton.getValue()) {
            isClawOpen = true;
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
        telemetry.addData("servo", claw.getPosition());
        telemetry.update();

        if (gameMasterGP.b.getValue()) {
            countDownTimerSwift.setTargetTime(20000);
        }
        isKonamiCodeComplete();

        isBeginnerKonamiCodeComplete();
        isCardinalCodeComplete();
        isSwiftStepComplete();
        isGrapeVineComplete();
        isHopScotchComplete();
        //boolean isDancing = danceRoutine(isActive);

        if (isClawOpen) {
            claw.setPosition(0.75);
        } else {
            claw.setPosition(0);
        }

//        armPower(armPosition);

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

        if(countDownTimerKonami.hasRemainingTime()){
            if(countDownTimerKonami.getRemainingTime() > 19000){
                leftDrive.setPower(1);
                rightDrive.setPower(1);
            } else if(countDownTimerKonami.getRemainingTime() > 17500){
                leftDrive.setPower(-0.25);
                rightDrive.setPower(0.75);
            } else if(countDownTimerKonami.getRemainingTime() > 16500){
                leftDrive.setPower(1);
                rightDrive.setPower(1);
            } else if(countDownTimerKonami.getRemainingTime() > 15000){
                leftDrive.setPower(-0.25);
                rightDrive.setPower(0.75);
            } else if(countDownTimerKonami.getRemainingTime() > 14000){
                leftDrive.setPower(1);
                rightDrive.setPower(1);
            } else if(countDownTimerKonami.getRemainingTime() > 12500){
                leftDrive.setPower(-0.25);
                rightDrive.setPower(0.75);
            } else if(countDownTimerKonami.getRemainingTime() > 11500){
                leftDrive.setPower(1);
                rightDrive.setPower(1);
            } else if(countDownTimerKonami.getRemainingTime() > 10000){
                leftDrive.setPower(-0.25);
                rightDrive.setPower(0.75);
            } else if(countDownTimerKonami.getRemainingTime() > 9000){
                leftDrive.setPower(1);
                rightDrive.setPower(1);
            }
        }

        if(countDownTimerSwift.hasRemainingTime()){
            if(countDownTimerSwift.getRemainingTime() > 19500){
                leftDrive.setPower(1);
                rightDrive.setPower(1);
            } else if(countDownTimerSwift.getRemainingTime() > 19000 && countDownTimerSwift.getRemainingTime() < 19500){
                turnByAngle(90);
            } else if(countDownTimerSwift.getRemainingTime() > 16500){
                leftDrive.setPower(1);
                rightDrive.setPower(1);
            } else if(countDownTimerSwift.getRemainingTime() > 16000 && countDownTimerSwift.getRemainingTime() < 16500){
                turnByAngle(90);
            } else if(countDownTimerSwift.getRemainingTime() > 13500){
                leftDrive.setPower(1);
                rightDrive.setPower(1);
            } else if(countDownTimerSwift.getRemainingTime() > 13000 && countDownTimerSwift.getRemainingTime() < 13500){
                if(!leftDrive.isBusy() && !rightDrive.isBusy()){
                    turnByAngle(90);
                }
            } else if(countDownTimerSwift.getRemainingTime() > 9500){
                leftDrive.setPower(1);
                rightDrive.setPower(1);
            }
        }

        if(countDownTimerBeginner.hasRemainingTime()){
            if(countDownTimerBeginner.getRemainingTime() > 7500){
                leftDrive.setPower(0.25);
                rightDrive.setPower(0.75);
            } else if(countDownTimerBeginner.getRemainingTime() > 2500){
                leftDrive.setPower(0.75);
                rightDrive.setPower(0.25);
            } else if(countDownTimerBeginner.getRemainingTime() > 0){
                leftDrive.setPower(0.25);
                rightDrive.setPower(0.75);
            }
        }

        if(countDownTimerCardinal.hasRemainingTime()){
            if(countDownTimerCardinal.getRemainingTime() > 1) {
                rightDrive.setPower(0.75);
                leftDrive.setPower(0.25);
            }
        }

        if(countDownTimerGrape.hasRemainingTime()){
            if(countDownTimerGrape.getRemainingTime() > 8000){
                rightDrive.setPower(-0.75);
                leftDrive.setPower(-0.25);
            } else if(countDownTimerGrape.getRemainingTime() > 6000){
                rightDrive.setPower(-0.25);
                leftDrive.setPower(-0.75);
            } else if(countDownTimerGrape.getRemainingTime() > 4000){
                rightDrive.setPower(-0.75);
                leftDrive.setPower(-0.25);
            } else if(countDownTimerGrape.getRemainingTime() > 2000){
                rightDrive.setPower(-0.25);
                leftDrive.setPower(-0.75);
            } else if(countDownTimerGrape.getRemainingTime() > 0){
                rightDrive.setPower(-0.75);
                leftDrive.setPower(-0.25);
            }
        }

        if(countDownTimerHop.hasRemainingTime()){
            if(countDownTimerHop.getRemainingTime() > 19000){
                leftDrive.setPower(1);
                rightDrive.setPower(1);
            } else if(countDownTimerHop.getRemainingTime() > 17500){
                leftDrive.setPower(-0.25);
                rightDrive.setPower(0.75);
            } else if(countDownTimerHop.getRemainingTime() > 16500){
                leftDrive.setPower(1);
                rightDrive.setPower(1);
            } else if(countDownTimerHop.getRemainingTime() > 15000){
                leftDrive.setPower(-0.25);
                rightDrive.setPower(0.75);
            } else if(countDownTimerHop.getRemainingTime() > 14000){
                leftDrive.setPower(1);
                rightDrive.setPower(1);
            } else if(countDownTimerHop.getRemainingTime() > 12500){
                leftDrive.setPower(-0.25);
                rightDrive.setPower(0.75);
            } else if(countDownTimerHop.getRemainingTime() > 11500){
                leftDrive.setPower(1);
                rightDrive.setPower(1);
            } else if(countDownTimerHop.getRemainingTime() > 10000){
                leftDrive.setPower(-0.25);
                rightDrive.setPower(0.75);
            } else if(countDownTimerHop.getRemainingTime() > 9000){
                leftDrive.setPower(1);
                rightDrive.setPower(1);
            }
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
            if(this.participantGP.topLeftArrow.isInitialPress()){
                swiftStepProgress = 4;
                countDownTimerSwift.setTargetTime(20000);
            }
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
            if(this.participantGP.leftArrow.isInitialPress()){
                cardinalCodeProgress = 4;
                countDownTimerCardinal.setTargetTime(7800);
            }
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
                countDownTimerBeginner.setTargetTime(10000);
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
            if(this.participantGP.topRightArrow.isInitialPress()){
                grapeVineProgress = 5;
                countDownTimerGrape.setTargetTime(10000);
            }
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
                countDownTimerHop.setTargetTime(20000);
            } else if (!this.participantGP.topLeftArrow.isPressed() && !this.participantGP.topRightArrow.isPressed()) {
                hopScotchProgress = 0;
            }
        } else if (hopScotchProgress == 5) {
            hopScotchProgress = 0;
        }
    }

    public void isKonamiCodeComplete() {
        if (!participantGP.areButtonsInitialPress()) {
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
                countDownTimerKonami.setTargetTime(20000);
            } else if (!this.participantGP.leftArrow.isPressed() && !this.participantGP.rightArrow.isPressed()) {
                konamiCodeProgress = 0;
            }
        }
        else if (konamiCodeProgress == 11) {
            konamiCodeProgress = 0;
        }
//        else if (konamiCodeProgress == 11) {
//            konamiCodeProgress = 0;
//        }
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
            turnByAngle(90);
            safeWait(1000);
            rightDrive.setPower(.75);
            leftDrive.setPower(.75);
            safeWait(1000);
            turnByAngle(90);
            safeWait(1000);
            rightDrive.setPower(.75);
            leftDrive.setPower(.75);
            safeWait(1000);
            turnByAngle(90);
            safeWait(1000);
            rightDrive.setPower(.75);
            leftDrive.setPower(.75);
            safeWait(1000);
            turnByAngle(90);
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
    public double getAngle() {
        // Get the Z-angle from the imu (in degrees) and return it as a value from -180 to 180
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.normalizeDegrees(orientation.firstAngle);
    }

    public void turnByAngle(double turnAngle) {

        // Get robot's start angle
        double startAngle = getAngle();

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
        leftDrive.setPower(-0.3 * turnDirection);
        rightDrive.setPower(0.3 * turnDirection);

        double currentAngle = getAngle();

        if (turnDirection == 1) {
            // If turning counter-clockwise: update currentAngle until it is at or greater than the targetAngle
            while (AngleUnit.normalizeDegrees(currentAngle - targetAngle) < 0) {

                telemetry.addData("Current Angle", currentAngle);
                telemetry.addData("Target Angle", targetAngle);
                telemetry.update();

                currentAngle = getAngle();
            }
        } else {
            //  If turning clockwise: update currentAngle until it is at or less than the targetAngle
            while (AngleUnit.normalizeDegrees(currentAngle - targetAngle) > 0) {

                telemetry.addData("Current Angle", currentAngle);
                telemetry.addData("Target Angle", targetAngle);
                telemetry.update();

                currentAngle = getAngle();
            }
        }

        // Stop the motors once we hit the targetAngle
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}
