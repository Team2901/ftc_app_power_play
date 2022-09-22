package org.firstinspires.ftc.teamcode.Shared.Hardware;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.AQUA;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_FAST;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_SLOW;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.CP1_END_TO_END_BLEND_TO_BLACK;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_MEDIUM;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_SLOW;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.CP1_LARSON_SCANNER;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.CP1_LIGHT_CHASE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.CP1_SHOT;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.CP1_STROBE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.CP2_BREATH_FAST;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.CP2_BREATH_SLOW;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.CP2_END_TO_END_BLEND_TO_BLACK;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_FAST;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_MEDIUM;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_SLOW;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.CP2_LARSON_SCANNER;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.CP2_SHOT;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.CP2_STROBE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.RED;
import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.WHITE;

public class ExemplaryBlinkinLED {

    public static final String CONFIG_TEAM_COLOR_FILENAME = "team_color_config.txt";

    public static final int LED_BLUE = 1;
    public static final int LED_RED = 2;
    public static final int LED_ERROR = -1;
    public int color = 1;
    RevBlinkinLedDriver blinkinLedDriver;

    public void init(HardwareMap hardwareMap, String deviceName) {
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, deviceName);
    }

    public void setTeamPattern(TeamColorPattern teamColorPattern) {
        RevBlinkinLedDriver.BlinkinPattern blinkinPattern = calcTeamPattern(teamColorPattern);

        if (blinkinLedDriver != null) {
            blinkinLedDriver.setPattern(blinkinPattern);
        }
    }

    public RevBlinkinLedDriver.BlinkinPattern calcTeamPattern(TeamColorPattern teamColorPattern) {
        if (color == LED_ERROR) {
            return WHITE;
        }
        switch (teamColorPattern) {
            case END_TO_END_BLEND_TO_BLACK:
                return color == 1 ? CP1_END_TO_END_BLEND_TO_BLACK : CP2_END_TO_END_BLEND_TO_BLACK;
            case LARSON_SCANNER:
                return color == 1 ? CP1_LARSON_SCANNER : CP2_LARSON_SCANNER;
            case LIGHT_CHASE:
                return color == 1 ? CP1_LIGHT_CHASE : CP2_LIGHT_CHASE;
            case HEARTBEAT_SLOW:
                return color == 1 ? CP1_HEARTBEAT_SLOW : CP2_HEARTBEAT_SLOW;
            case HEARTBEAT_MEDIUM:
                return color == 1 ? CP1_HEARTBEAT_MEDIUM : CP2_HEARTBEAT_MEDIUM;
            case HEARTBEAT_FAST:
                return color == 1 ? CP1_HEARTBEAT_FAST : CP2_HEARTBEAT_FAST;
            case BREATH_SLOW:
                return color == 1 ? CP1_BREATH_SLOW : CP2_BREATH_SLOW;
            case BREATH_FAST:
                return color == 1 ? CP1_BREATH_FAST : CP2_BREATH_FAST;
            case SHOT:
                return color == 1 ? CP1_SHOT : CP2_SHOT;
            case STROBE:
                return color == 1 ? CP1_STROBE : CP2_STROBE;
            case SOLID:
                return color == 1 ? AQUA : RED;
            default:
                return RAINBOW_OCEAN_PALETTE;
        }
    }

    public enum TeamColorPattern {
        END_TO_END_BLEND_TO_BLACK,
        LARSON_SCANNER,
        LIGHT_CHASE,
        HEARTBEAT_SLOW,
        HEARTBEAT_MEDIUM,
        HEARTBEAT_FAST,
        BREATH_SLOW,
        BREATH_FAST,
        SHOT,
        STROBE,
        SOLID;

        private static TeamColorPattern[] elements = values();

        public static TeamColorPattern fromNumber(int number) {
            return elements[number % elements.length];
        }

        public TeamColorPattern next() {
            return elements[(this.ordinal() + 1) % elements.length];
        }

        public TeamColorPattern previous() {
            return elements[(this.ordinal() - 1) < 0 ? elements.length - 1 : this.ordinal() - 1];
        }
    }
}
