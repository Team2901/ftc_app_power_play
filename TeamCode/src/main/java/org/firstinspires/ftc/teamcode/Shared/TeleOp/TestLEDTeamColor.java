package org.firstinspires.ftc.teamcode.Shared.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Shared.Hardware.ExemplaryBlinkinLED;
import org.firstinspires.ftc.teamcode.Utility.FileUtilities;

import static org.firstinspires.ftc.teamcode.Shared.Hardware.ExemplaryBlinkinLED.CONFIG_TEAM_COLOR_FILENAME;
import static org.firstinspires.ftc.teamcode.Shared.Hardware.ExemplaryBlinkinLED.LED_ERROR;

@Disabled
@TeleOp(name = "Test LED Team Color", group = "Test")
public class TestLEDTeamColor extends OpMode {

    public ExemplaryBlinkinLED blinkinLED = new ExemplaryBlinkinLED();

    @Override
    public void init() {

        blinkinLED.init(hardwareMap, "LED");
        blinkinLED.color = this.readTeamColor();
        blinkinLED.setTeamPattern(ExemplaryBlinkinLED.TeamColorPattern.SOLID);
        telemetry.update();
    }

    public int readTeamColor() {
        try {
            return FileUtilities.readTeamColor(CONFIG_TEAM_COLOR_FILENAME);
        } catch (Exception e) {
            return LED_ERROR;
        }
    }

    @Override
    public void loop() {

    }
}
