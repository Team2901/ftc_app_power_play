package org.firstinspires.ftc.teamcode.GreenTeam.SavedCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.GreenTeam.SavedCode.OpenCVGreenPipeline;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
/* test push */
public class SidearmGreenHardware {
    private ElapsedTime runtime = new ElapsedTime();
    OpenCVGreenPipeline pipeline = new OpenCVGreenPipeline();

    public static final double TICKS_PER_MOTOR_REV = 537.7;
    public static final double DRIVE_GEAR_RATIO = 1;
    public static final double TICKS_PER_DRIVE_REV = TICKS_PER_MOTOR_REV * DRIVE_GEAR_RATIO;
    public static final double WHEEL_CIRCUMFERENCE = 4 * Math.PI;
    public static final double TICKS_PER_INCH = TICKS_PER_DRIVE_REV / WHEEL_CIRCUMFERENCE;

    public DcMotorEx frontLeft;
    public DcMotorEx backLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backRight;
    public DcMotor intakeMotor;
    public DcMotor spinner;
    public DcMotorEx arm;
    public TouchSensor backPlate;
    public RevBlinkinLedDriver lights;
    public Servo cap;
    public enum DuckPosition {
        UNKNOWN,
        LEFT,
        MIDDLE,
        RIGHT
    }
    public DuckPosition winner = DuckPosition.UNKNOWN;

    public BNO055IMU imu;

    public void init(HardwareMap hardwareMap){
        this.init(hardwareMap, false);
    }

    public void init(HardwareMap hardwareMap, boolean cameraChoice) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left");
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right");
        backRight = hardwareMap.get(DcMotorEx.class, "back_right");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        spinner = hardwareMap.get(DcMotor.class, "spinner");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        backPlate = hardwareMap.get(TouchSensor.class, "back_plate");
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "light_controller");
        cap = hardwareMap.get(Servo.class, "capping_stick");


        if(cameraChoice) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
            OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
            camera.setPipeline(new filterYellowHSV());
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }
                @Override
                public void onError(int errorCode)
                {
                    /*
                     * This will be called if the camera could not be opened
                     */
                }
            });
        }

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        intakeMotor.setPower(0);
        spinner.setPower(0);
        arm.setPower(0);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public double getAngle() {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.normalizeDegrees(orientation.firstAngle);
    }
    public void safeWait(int milliseconds){
        runtime.reset();
        while(runtime.milliseconds() < milliseconds){
        }
    }

    class filterYellowHSV extends OpenCvPipeline {
        Mat hsvImg = new Mat();
        Mat finalImage = null;
        Mat mask = null;

        @Override
        public Mat processFrame(Mat input){
            //converts image from rgb to hsv
            Imgproc.cvtColor(input, hsvImg, Imgproc.COLOR_RGB2HSV);
            // creating a black mask of zero
            if(mask == null)
                mask = new Mat( input.size(), CvType.CV_8UC1);
            Scalar min = new Scalar( 0, 125, 125);
            Scalar max = new Scalar( 60, 255, 255);
            //updates mask so that there is ones where ever the color of interest is
            Core.inRange(hsvImg, min, max, mask);
            //creates the final image in all black
            if(finalImage != null)
                finalImage.release();
            finalImage = new Mat(input.size(), input.type(), new Scalar(0, 0, 0));
            //final image were it only shows color of interest while everything else is black
            Core.bitwise_and(input, input, finalImage, mask);
            int thickness = 5;
            Scalar color = new Scalar(68, 114, 196);
            Scalar winningColor = new Scalar(255, 0, 0);

            // get submatrix of the mask
            Rect left = new Rect(0, 150, 106, 50);
            Mat areaLeft = mask.submat( left );
            Imgproc.rectangle(finalImage, left, color, 3);
            int countLeft = Core.countNonZero(areaLeft);
            Point positionLeft = new Point(0, 150);

            Rect middle = new Rect(106, 150, 106, 50);
            Mat areaMiddle = mask.submat( middle );
            Imgproc.rectangle(finalImage, middle, color, 3);
            int countMiddle = Core.countNonZero(areaMiddle);
            Point positionMiddle = new Point(100, 150);

            int width = mask.width();
            int height = mask.height();
            Rect right = new Rect(212, 150, 106, 50);
            Mat areaRight = mask.submat( right );
            Imgproc.rectangle(finalImage, right, color, 3);
            int countRight = Core.countNonZero(areaRight);
            Point positionRight = new Point(200, 150);

            // count the ones in the sub matrix
            // put the count on the screen



            Imgproc.putText(finalImage, String.valueOf(countLeft), positionLeft, Imgproc.FONT_HERSHEY_SIMPLEX, 1, color, 3);
            Imgproc.putText(finalImage, String.valueOf(countMiddle), positionMiddle, Imgproc.FONT_HERSHEY_SIMPLEX, 1, color, 3);
            Imgproc.putText(finalImage, String.valueOf(countRight), positionRight, Imgproc.FONT_HERSHEY_SIMPLEX, 1, color, 3);
            winner = DuckPosition.UNKNOWN;
            if (countMiddle>countRight){
                if(countLeft>countMiddle){
                    winner = DuckPosition.LEFT;
                }else{
                    winner = DuckPosition.MIDDLE;
                }
            }else{
                if(countLeft>countRight){
                    winner = DuckPosition.LEFT;
                }
                if(countRight>countLeft){
                    winner = DuckPosition.RIGHT;
                }
            }
            if(winner== DuckPosition.MIDDLE){
                Imgproc.putText(finalImage, String.valueOf(countMiddle), positionMiddle, Imgproc.FONT_HERSHEY_SIMPLEX, 1, winningColor, 3);
                //Imgproc.putText(finalImage, "Middle wins", positionMiddle, Imgproc.FONT_HERSHEY_SIMPLEX, 1, winningColor, 3);
            }else if(winner == DuckPosition.RIGHT){
                Imgproc.putText(finalImage, String.valueOf(countRight), positionRight, Imgproc.FONT_HERSHEY_SIMPLEX, 1, winningColor, 3);
                //Imgproc.putText(finalImage, "Right wins", positionRight, Imgproc.FONT_HERSHEY_SIMPLEX, 1, winningColor, 3);
            }else{
                Imgproc.putText(finalImage, String.valueOf(countLeft), positionLeft, Imgproc.FONT_HERSHEY_SIMPLEX, 1, winningColor, 3);
                //Imgproc.putText(finalImage, "Left wins", positionLeft, Imgproc.FONT_HERSHEY_SIMPLEX, 1, winningColor, 3);
            }
            return finalImage;
        }

    }


}
