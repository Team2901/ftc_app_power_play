package org.firstinspires.ftc.teamcode.Outreach.Hardware;

import org.firstinspires.ftc.teamcode.Shared.Gamepad.DDRGamepad;

import java.util.ArrayList;

public class DDRDance {
    enum DanceMoves { LEFT, RIGHT, UP, DOWN, X, O }
    private enum ProgressState { COMPLETED, AWAITING, FAILED}
    private ArrayList<DanceMoves> danceMoves;
    private DDRGamepad gamepad;
    private DanceObserver observer;
    private int progress = 0;

     DDRDance(ArrayList<DanceMoves> moves, DDRGamepad ddrGamepad, DanceObserver danceObserver) {
        danceMoves = moves;
        gamepad = ddrGamepad;
        observer = danceObserver;
    }

    public void update() {

         ProgressState currentState = checkDance(danceMoves.get(progress));
        if (currentState == ProgressState.COMPLETED) {
            progress++;
        } else if (currentState == ProgressState.FAILED) {
            progress = 0;
        }

        if (progress >= danceMoves.size()) {
            observer.onCompleted();
            progress = 0;
        }

    }

    public ProgressState checkDance(DanceMoves move) {
        switch (move) {
            case LEFT:
                if (gamepad.leftArrow.isInitialPress()) {
                    return ProgressState.COMPLETED;
                } else if (gamepad.areButtonsInitialPress()) {
                    return ProgressState.FAILED;
                }
                break;
            case RIGHT:
                if (gamepad.rightArrow.isInitialPress()) {
                    return ProgressState.COMPLETED;
                } else if (gamepad.areButtonsInitialPress()) {
                    return ProgressState.FAILED;
                }
                break;
            case UP:
                if (gamepad.upArrow.isInitialPress()) {
                    return ProgressState.COMPLETED;
                } else if (gamepad.areButtonsInitialPress()) {
                    return ProgressState.FAILED;
                }
                break;
            case DOWN:
                if (gamepad.downArrow.isInitialPress()) {
                    return ProgressState.COMPLETED;
                } else if (gamepad.areButtonsInitialPress()) {
                    return ProgressState.FAILED;
                }
                break;
            case X:
                if (gamepad.startButton.isInitialPress()) {
                    return ProgressState.COMPLETED;
                } else if (gamepad.areButtonsInitialPress()) {
                    return ProgressState.FAILED;
                }
                break;
            case O:
                if (gamepad.a.isInitialPress()) {
                    return ProgressState.COMPLETED;
                } else if (gamepad.areButtonsInitialPress()) {
                    return ProgressState.FAILED;
                }
                break;
        }
        return ProgressState.AWAITING;
    }

}
