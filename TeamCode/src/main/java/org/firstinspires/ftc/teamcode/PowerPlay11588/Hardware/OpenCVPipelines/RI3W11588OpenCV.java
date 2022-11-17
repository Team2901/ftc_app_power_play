package org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.OpenCVPipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.Qual11588Hardware;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RI3W11588OpenCV extends OpenCvPipeline {

    private Mat lastImage = null;
    private Mat subMat = null;
    public enum ConeColor { red, green, blue};
    public ConeColor coneColor = null;
    public double redAmount;
    public double blueAmount;
    public double greenAmount;
//    public double redAmountAllTime;
//    public double greenAmountAllTime;
//    public double blueAmountAllTime;
//    public double redAmountAverage;
//    public double greenAmountAverage;
//    public double blueAmountAverage;
    public int framesProceeded;
    Rect r = new Rect(100, 100, 100, 100);
    Mat redMask = new Mat();
    Mat blueMask = new Mat();
    Mat greenMask = new Mat();

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

        if(coneColor == null) {
            input.copyTo(lastImage);
            //copies the last input to lastImage, actually assigning it

            Imgproc.cvtColor(lastImage, lastImage, Imgproc.COLOR_RGBA2RGB);
            subMat = lastImage.submat(r);

            Imgproc.rectangle(lastImage, r, new Scalar(100, 0, 0));

            Core.inRange(subMat, new Scalar(100, 50, 50), new Scalar(255, 100, 155), redMask);
            Core.inRange(subMat, new Scalar(0, 0, 80), new Scalar(70, 70, 255), blueMask);
            Core.inRange(subMat, new Scalar(0, 80, 0), new Scalar(80, 255, 80), greenMask);
            /*
            This statement above creates the 3 different masks that act as filters. Do not think of this
            as one image, it is really 3 different images with different filters. We use a range for the RBG
            values. If this ever in the future this code doesn't work or isn't sensitive enough try changing the
            values for the filter.
             */

            double nonZeroPixelsRed = Core.countNonZero(redMask);

            double nonZeroPixelsBlue = Core.countNonZero(blueMask);

            double nonZeroPixelsGreen = Core.countNonZero(greenMask);
            //This counts the number of non zero pixels in each mask

            redAmount = nonZeroPixelsRed / subMat.total() * 100;

            blueAmount = nonZeroPixelsBlue / subMat.total() * 100;

            greenAmount = nonZeroPixelsGreen / subMat.total() * 100;
            //This creates a percentage of pixels on the screen, this are not scaled to each other
            //TO a degree each value/mask is arbitrary


            //Imgproc.cvtColor(redMask, redMask, Imgproc.COLOR_GRAY2RGB);
            //Core.bitwise_and(subMat, redMask, lastImage);
            //Core.bitwise_and(subMat, subMat, subMat, redMask);
            //This commented out code is only for visualizing the pipeline

//            redAmountAllTime = redAmountAllTime + redAmount;
//            blueAmountAllTime = blueAmountAllTime + blueAmount;
//            greenAmountAllTime = greenAmountAllTime + greenAmount;
//
//            redAmountAverage = redAmountAllTime / framesProceeded;
//            greenAmountAverage = greenAmountAllTime / framesProceeded;
//            blueAmountAverage = blueAmountAllTime / framesProceeded;

//            This code is literally only for testing and getting an average



            //All Pipeline code must be written above here
            //Simple if statement to determine what is the largest amount/percentage of a color

        }
        return lastImage;
        /*The entire thing is in an if statement because we only want process frame to run one until
        we get a value for coneColor because we don't want it to change in the middle of our run because that
        might change what color it thinks the cone is.
        */

    }

    public void getColor( String color) {
        if (color == "blue") {
            if (redAmount > (blueAmount - 3) && redAmount > greenAmount) {
                coneColor = RI3W11588OpenCV.ConeColor.red;
            } else if ((blueAmount - 3) > redAmount && (blueAmount - 3) > greenAmount) {
                coneColor = RI3W11588OpenCV.ConeColor.blue;
            } else if (greenAmount > redAmount && greenAmount > (blueAmount - 3)) {
                coneColor = RI3W11588OpenCV.ConeColor.green;
            }
        } else if (color == "red") {
            if (redAmount > (blueAmount) && redAmount > greenAmount) {
                coneColor = RI3W11588OpenCV.ConeColor.red;
            } else if ((blueAmount) > redAmount && (blueAmount) > greenAmount) {
                coneColor = RI3W11588OpenCV.ConeColor.blue;
            } else if (greenAmount > redAmount && greenAmount > (blueAmount - 3)) {
                coneColor = RI3W11588OpenCV.ConeColor.green;
            }
        }

    }


    }

//    public void openCVTelemetry() {
//        telemetry.addData("Red Amount", redAmount);
//        telemetry.addData("Blue Amount", blueAmount);
//        telemetry.addData("Green Amount", greenAmount);
//        telemetry.addData("Frames Proceeded", framesProceeded);
//        telemetry.addData("Red average", redAmountAverage);telemetry.addData("Blue average", blueAmountAverage);
//        telemetry.addData("Green average", greenAmountAverage);
//        telemetry.update();
//    }
    //This is a separate method because you will not always want to see OpenCV Telemetry


