package org.firstinspires.ftc.teamcode.Outreach.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Outreach.Hardware.OutreachBotOneHardware;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.DDRGamepad;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

@Disabled
@TeleOp(name = "Outreach Bot New Control TeleOp", group = "Outreach")
public class NewControlOutreachBot extends OpMode {
    OutreachBotOneHardware robot = new OutreachBotOneHardware();
    /*
    CountDownTimer maxBoostTimer = new CountdownTimer(ElapsedTime.Resolution.MILLISECONDS);
     */
    ElapsedTime timer = new ElapsedTime();
    DDRGamepad participantGP;
    ImprovedGamepad gameMasterGP;

    @Override
    public void init() {
        robot.init(hardwareMap);

        participantGP = new DDRGamepad(this.gamepad1, this.timer, "GP1");
        gameMasterGP = new ImprovedGamepad(this.gamepad2, this.timer, "GP2");
    }

    @Override
    public void loop() {
        gameMasterGP.update();
        robot.leftDrive.setPower((-gameMasterGP.right_stick_y.getValue() + -gameMasterGP.left_stick_y.getValue()) / 4 +
                (-gameMasterGP.left_stick_x.getValue()));
        robot.rightDrive.setPower((-gameMasterGP.right_stick_y.getValue() + -gameMasterGP.left_stick_y.getValue()) / 4 +
                (gameMasterGP.right_stick_x.getValue()));
    }
}
