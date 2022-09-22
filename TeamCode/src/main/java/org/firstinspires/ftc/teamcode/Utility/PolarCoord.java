package org.firstinspires.ftc.teamcode.Utility;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@SuppressLint("DefaultLocale")
public class PolarCoord {

    public static final double MM_TO_INCHES = 0.0393701;

    public double x;
    public double y;
    public double theta;
    public String name;

    public PolarCoord(double x, double y, double theta, String name) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.name = name;
    }

    public PolarCoord(double x, double y, String name) {
        this(x, y, 0, name);
    }

    public PolarCoord(double x, double y, double theta) {
        this(x, y, theta, null);
    }

    public PolarCoord(double x, double y) {
        this(x, y, 0, null);
    }

    public PolarCoord(final OpenGLMatrix location) {
        VectorF translation = location.getTranslation();
        Orientation orientation = Orientation.getOrientation(location,
                AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        this.x = (translation.get(0) * MM_TO_INCHES);
        this.y = (translation.get(1) * MM_TO_INCHES);
        this.theta = AngleUtilities.getNormalizedAngle(orientation.thirdAngle);
    }

    public static double getDistanceBetween(PolarCoord startPolarCoord, PolarCoord goalPolarCoord) {
        return Math.sqrt((Math.pow((goalPolarCoord.x - startPolarCoord.x), 2) +
                Math.pow((goalPolarCoord.y - startPolarCoord.y), 2)));
    }

    public static double getAngleBetween(PolarCoord startPolarCoord, PolarCoord goalPolarCoord) {
        return AngleUtilities.getNormalizedAngle(Math.atan2(goalPolarCoord.y - startPolarCoord.y, goalPolarCoord.x - startPolarCoord.x) * (180 / Math.PI));
    }

    public PolarCoord withTheta(double theta) {
        this.theta = theta;
        return this;
    }

    @Override
    public String toString() {
        if (name != null) {
            return String.format("%s - x:%.2f y:%.2f Θ: %.2f", name, x, y, theta);
        } else {
            return String.format("x:%.2f y:%.2f Θ: %.2f", x, y, theta);
        }
    }
}