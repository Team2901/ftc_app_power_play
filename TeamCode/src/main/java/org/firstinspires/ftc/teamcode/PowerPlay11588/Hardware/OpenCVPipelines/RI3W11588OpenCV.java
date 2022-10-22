package org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.OpenCVPipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RI3W11588OpenCV extends OpenCvPipeline {

    private Mat lastImage = null;
    private Mat subMat = null;
    public static enum ConeColor { red, green, blue};
    public ConeColor coneColor = null;
    public double redAmount;
    public double blueAmount;
    public double greenAmount;
    public int framesProceeded;
    Rect r = new Rect(100, 100, 100, 100);
    Mat redMask = new Mat();
    Mat blueMask = new Mat();
    Mat greenMask = new Mat();
    //lastImage is used to store an unedited version of the last frame, can be used for comparision
    //Mat class is for matracies. 3 dimensions, width, height, and color(color is stored in the 3rd value)



    Telemetry telemetry;

    public RI3W11588OpenCV(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    //Constructor for RI3W11588OpenCV class

    public void init(Mat input) {

        lastImage = new Mat(input.rows(), input.cols(), input.type());

    }
    @Override
    public Mat processFrame(Mat input) {
        framesProceeded++;
        if (input == null) {
            return null;
//Makes sure doesn't crash when the camera does nothing
        }

        input.copyTo(lastImage);
        //copies the last input to lastImage, actually assigning it

        Imgproc.cvtColor(lastImage, lastImage, Imgproc.COLOR_RGBA2RGB);
        subMat = lastImage.submat(r);


        Imgproc.rectangle(lastImage, r , new Scalar(100, 0, 0));



        Core.inRange(subMat, new Scalar(100, 50, 50), new Scalar(255, 100, 155), redMask);
        Core.inRange(subMat, new Scalar(0, 0, 80), new Scalar(70, 70, 255), blueMask);
        Core.inRange(subMat, new Scalar(0, 80, 0), new Scalar(80, 255, 80), greenMask);




        double nonZeroPixelsRed = Core.countNonZero(redMask);

        double nonZeroPixelsBlue = Core.countNonZero(blueMask);

        double nonZeroPixelsGreen = Core.countNonZero(greenMask);

        //This method creates a new mask that isolates a range of colors

        redAmount = nonZeroPixelsRed / subMat.total() * 100;

        blueAmount = nonZeroPixelsBlue / subMat.total() * 100;

        greenAmount = nonZeroPixelsGreen / subMat.total() * 100;

        //Imgproc.cvtColor(redMask, redMask, Imgproc.COLOR_GRAY2RGB);
        //Core.bitwise_and(subMat, redMask, lastImage);
        //Core.bitwise_and(subMat, subMat, subMat, redMask);

        telemetry.addData("Red Amount", redAmount);
        telemetry.addData("Blue Amount", blueAmount);
        telemetry.addData("Green Amount", greenAmount);
        telemetry.addData("Frames Proceeded", framesProceeded);
        telemetry.update();
        //All Pipeline code must be written above here


        return lastImage;
        //what is returned is what you will see
    }

    public void openCVTelemetry() {
        telemetry.addData("Red Amount", redAmount);
        telemetry.addData("Blue Amount", blueAmount);
        telemetry.addData("Green Amount", greenAmount);
        telemetry.addData("Frames Proceeded", framesProceeded);
        telemetry.update();
    }

}
