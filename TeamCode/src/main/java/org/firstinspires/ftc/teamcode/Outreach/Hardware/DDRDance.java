package org.firstinspires.ftc.teamcode.Outreach.Hardware;

import java.util.ArrayList;

public class DDRDance {
    enum DanceMoves { LEFT, RIGHT, UP, DOWN, X, O }
    private ArrayList<DanceMoves> danceMoves;

    public void DDRDance(ArrayList<DanceMoves> danceMoves) {
        this.danceMoves = danceMoves;
    }


}
