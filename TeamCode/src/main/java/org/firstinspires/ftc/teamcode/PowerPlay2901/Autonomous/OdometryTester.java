package org.firstinspires.ftc.teamcode.PowerPlay2901.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.PowerPlay2901.Autonomous.XYhVector;


@TeleOp(name = "Odometry Tester", group = "T265")
public class OdometryTester extends OpMode {
    public static final double encoderTicksPerWheelRev = 8192; //REV encoders
    public static final double wheelCircumference = (2.83465 * Math.PI); //72mm diameter wheels
    public static final double leftRightDistance = 9; //Distance between left and right odometry wheels
    public static final double midpointBackDistance = 5;
    public static final double inchPerTick = wheelCircumference/encoderTicksPerWheelRev;
    public static final double wheelCircumferenceBack = (2 * Math.PI);
    public static final double backInchPerTick = wheelCircumferenceBack/encoderTicksPerWheelRev;

    public DcMotor odoLeft;
    public DcMotor odoRight;
    public DcMotor odoBack;

    public int currentLeftPosition = 0;
    public int currentRightPosition = 0;
    public int currentBackPosition = 0;
    private int oldLeftPosition = 0;
    private int oldRightPosition = 0;
    private int oldBackPosition = 0;

    public XYhVector START_POS = new XYhVector(0, 0, 0);
    public XYhVector pos = new XYhVector(START_POS);
    @Override
    public void init() {
        odoLeft = hardwareMap.get(DcMotor.class, "odoLeft");
        odoRight = hardwareMap.get(DcMotor.class, "odoRight");
        odoBack = hardwareMap.get(DcMotor.class, "odoBack");

        odoLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odoRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odoBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        odometry();
        telemetry.addData("x", pos.x);
        telemetry.addData("y", pos.y);
        telemetry.addData("h", pos.h);
    }

     public void odometry(){
        oldRightPosition = currentRightPosition;
        oldLeftPosition = currentLeftPosition;
        oldBackPosition = currentBackPosition;

        currentRightPosition = odoRight.getCurrentPosition();
        currentLeftPosition = odoLeft.getCurrentPosition();
        currentBackPosition = odoBack.getCurrentPosition();

        int dn1 = currentRightPosition - oldRightPosition;
        int dn2 = currentLeftPosition - oldLeftPosition;
        int dn3 = currentBackPosition - oldBackPosition;

        dn2 = -dn2;

        double dtheta = ((dn2 - dn1) / leftRightDistance) * inchPerTick;
        double dx = ((dn1 + dn2) / 2) * inchPerTick;
        double dy = ((backInchPerTick * dn3) - (inchPerTick * (dn2 - dn1) * midpointBackDistance) / leftRightDistance);

        double theta = pos.h + (dtheta / 2.0);
        pos.y += dx * Math.cos(theta) - dy * Math.sin(theta);
        pos.x -= dx * Math.sin(theta) + dy * Math.cos(theta);
        pos.h += dtheta;


        telemetry.addData("right position", currentRightPosition);
         telemetry.addData("left position", currentLeftPosition);

         telemetry.addData("back position", currentBackPosition);}
}
