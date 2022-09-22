package org.firstinspires.ftc.teamcode.Shared.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Shared.Gamepad.DDRGamepad;
import org.firstinspires.ftc.teamcode.Shared.Gamepad.ImprovedGamepad;

@Disabled
@TeleOp(name = "DDR Pad Test", group = "Test")
public class TestDDRPad extends OpMode {

    public ElapsedTime timer = new ElapsedTime();
    public DDRGamepad dancePad;

    @Override
    public void init() {
        this.dancePad = new DDRGamepad(gamepad1, timer, "g1");
    }

    @Override
    public void loop() {
        dancePad.update();

        /*
         * +----+----+----+
         * |    |    |    |
         * |    |    |    |
         * +----+----+----+
         * |    |    |    |
         * |    |    |    |
         * +----+----+----+
         * |    |    |    |
         * |    |    |    |
         * +----+----+----+
         */

        telemetry.addData("Are buttons active", dancePad.areButtonsActive());

        if(dancePad.upArrow.getValue()){
            telemetry.addData("Up Arrow Times Pressed", dancePad.upArrow.getPressedCounts());
            telemetry.addLine("Up arrow");
        }

        if(dancePad.downArrow.getValue()){
            telemetry.addData("Down Arrow Times Pressed", dancePad.downArrow.getPressedCounts());
            telemetry.addLine("Down arrow");
        }

        if(dancePad.leftArrow.getValue()){
            telemetry.addData("Left Arrow Times Pressed", dancePad.leftArrow.getPressedCounts());
            telemetry.addLine("Left arrow");
        }

        if(dancePad.rightArrow.getValue()){
            telemetry.addData("Right Arrow Times Pressed", dancePad.rightArrow.getPressedCounts());
            telemetry.addLine("Right arrow");
        }

        if(dancePad.topLeftArrow.getValue()){
            telemetry.addData("TopLeft Arrow Times Pressed", dancePad.topLeftArrow.getPressedCounts());
            telemetry.addLine("X");
        }

        if(dancePad.topRightArrow.getValue()){
            telemetry.addData("TopRight Arrow Times Pressed", dancePad.topRightArrow.getPressedCounts());
            telemetry.addLine("O");
        }

        if(dancePad.startButton.getValue()){
            telemetry.addData("Start Button Times Pressed", dancePad.startButton.getPressedCounts());
            telemetry.addLine("Start button");
        }

        if(dancePad.a.getValue()){
            telemetry.addData("A Times Pressed", dancePad.a.getPressedCounts());
            telemetry.addLine("a button");
        }

        telemetry.update();
    }
}
