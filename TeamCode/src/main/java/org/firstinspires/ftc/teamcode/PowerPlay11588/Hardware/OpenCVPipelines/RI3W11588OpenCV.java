package org.firstinspires.ftc.teamcode.PowerPlay11588.Hardware.OpenCVPipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RI3W11588OpenCV extends OpenCvPipeline {

    public Mat lastImage = null;
    public static enum ConeColor { red, green, blue};
    Mat redMask = new Mat();
    Mat blueMask = new Mat();
    Mat greenMask = new Mat();
    //lastImage is used to store an unedited version of the last frame, can be used for comparision
    //Mat class is for matracies. 3 dimensions, width, height, and color(color is stored in the 3rd value)



    Telemetry telemetry;
    public int nonZeroPixels;

    public RI3W11588OpenCV(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    //Constuctor for NickJFIrstVisionPipeLine class


    @Override
    public Mat processFrame(Mat input) {
        if (input == null) {
            return null;
//Makes sure doesn't crash when the camera does nothing
        }

        if(lastImage != null) {
            lastImage.release();
            //releases memory(openCV is written in C, so memory managment is needed
        }

        lastImage = new Mat();

        input.copyTo(lastImage);
        //copies the last input to lastImage, actually assigning it

        Imgproc.cvtColor(lastImage, lastImage, Imgproc.COLOR_RGBA2RGB);

        Imgproc.rectangle(lastImage, new Rect(100, 100, 100, 100), new Scalar(255, 0, 0));

        Core.inRange(lastImage, new Scalar(100, 0, 0), new Scalar(255, 100, 115), redMask);
        Core.inRange(lastImage, new Scalar(0, 0, 100), new Scalar(155, 100, 255), blueMask);

        nonZeroPixels = Core.countNonZero(redMask);
        //This method creates a new mask that isolates a range of colors

        Imgproc.cvtColor(redMask, redMask, Imgproc.COLOR_GRAY2RGB);

        double percentRed = 100 - lastImage.total() /  nonZeroPixels;


        telemetry.addData("mask size", redMask.size());
        telemetry.addData("image size", lastImage.size());
        telemetry.addData("mask type", redMask.type());
        telemetry.addData("image type", lastImage.type());
        telemetry.addData("Red Percent", percentRed);
        telemetry.update();

        Core.bitwise_and(lastImage, blueMask, lastImage);
        //All Pipeline code must be written above here



        telemetry.addData("Percent Red", percentRed);

        ConeColor coneColor = null;

        if(percentRed > 85){
            coneColor = ConeColor.red;
        }

        if(coneColor != ConeColor.red){


            Core.bitwise_and(lastImage, blueMask, lastImage);
        }

        return lastImage;
        //what is returned is what you will see
    }
}