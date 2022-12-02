package org.firstinspires.ftc.teamcode.PowerPlay2901.Autonomous;

public class XYhVector {

    public double x;
    public double y;
    public double h;

    public XYhVector(double x, double y, double h){
        this.x = x;
        this.y = y;
        this.h = h;
    }
    public XYhVector(XYhVector vector){
        x = vector.x;
        y = vector.y;
        h = vector.h;
    }
}
