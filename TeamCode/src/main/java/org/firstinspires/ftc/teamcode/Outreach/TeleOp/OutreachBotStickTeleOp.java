package org.firstinspires.ftc.teamcode.Outreach.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Outreach.Hardware.OutreachBotOneHardware;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

@Disabled
@TeleOp (name = "Outreach Joystick TeleOp", group = "Outreach")
public class OutreachBotStickTeleOp extends OpMode {
    OutreachBotOneHardware robot = new OutreachBotOneHardware();
    ElapsedTime timer = new ElapsedTime();
    ImprovedGamepad participantGamepad;
    ImprovedGamepad gmGamepad;
    boolean override = false;
    int difficulty = 1;
    int control = 0;
    String[] controllers = {"Participant", "Game Master"};
    String[] difficultyNames = {"Beginner", "Intermediate", "Lawsuit"};
    double participantLeftPower;
    double participantRightPower;
    double gmLeftPower;
    double gmRightPower;
    boolean isClawOpen = true;

    @Override
    public void init() {
        robot.init(hardwareMap);
        participantGamepad = new ImprovedGamepad(this.gamepad1, this.timer, "GP1");
        gmGamepad = new ImprovedGamepad(this.gamepad2, this.timer, "GP2");
    }

    @Override
    public void loop() {
        participantGamepad.update();
        gmGamepad.update();
        if(gmGamepad.a.isInitialPress()){
            override = !override;
        }
        if(gmGamepad.right_bumper.isInitialPress() && difficulty < 2){
            difficulty++;
        }
        if(gmGamepad.left_bumper.isInitialPress() && difficulty > 0){
            difficulty--;
        }

        /*
        gmLeftPower = gmGamepad.left_stick_y.getValue();
        gmRightPower = gmGamepad.left_stick_y.getValue();

        gmLeftPower += gmGamepad.left_stick_x.getValue();
        gmRightPower -= gmGamepad.left_stick_x.getValue();
        */

        gmLeftPower = gmGamepad.left_stick_y.getValue();
        gmRightPower = gmGamepad.right_stick_y.getValue();


        //Single stick drive?
        /*
        participantLeftPower = participantGamepad.left_stick_y.getValue();
        participantRightPower = participantGamepad.left_stick_y.getValue();

        participantLeftPower += participantGamepad.left_stick_x.getValue();
        participantRightPower -= participantGamepad.left_stick_x.getValue();

         */

        //Dual stick drive
        participantLeftPower = participantGamepad.left_stick_y.getValue();
        participantRightPower = participantGamepad.right_stick_y.getValue();

        if(gmGamepad.b.isInitialPress() || (!override && participantGamepad.b.isInitialPress())){
            isClawOpen = !isClawOpen;
        }

        if(!isClawOpen){
            robot.claw.setPosition(0);
        }else{
            robot.claw.setPosition(.5);
        }

        if(gmGamepad.left_stick.isPressed() || gmGamepad.right_stick.isPressed() ){
            power(gmLeftPower, gmRightPower);
            control = 1;
        }else if(!override && (participantGamepad.left_stick.isPressed() || participantGamepad.right_stick.isPressed())){
            if(difficulty == 0){
                participantLeftPower /= 3;
                participantRightPower /= 3;
            }else if(difficulty == 1){
                participantLeftPower /= (2.0/3);
                participantRightPower /= (2.0/3);
            }
            control = 0;
            power(participantLeftPower, participantRightPower);
        }else{
            power(0,0);
        }
        telemetry.addData("Override: ", override);
        telemetry.addData("In control: ", controllers[control]);
        telemetry.addData("Difficulty: ", difficultyNames[difficulty]);


    }
    public void power(double left, double right){
        robot.leftDrive.setPower(-left);
        robot.rightDrive.setPower(-right);
    }
}