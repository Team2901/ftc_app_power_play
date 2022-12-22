package org.firstinspires.ftc.teamcode.NewProgrammers.Y2023.Mecanum;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ObjectDetectionPipeline extends OpenCvPipeline {
    Mat lastImage = null;
    private Telemetry telemetry;
    int count = 0;
    public ObjectDetectionPipeline(Telemetry telemetry){
        this.telemetry = telemetry;
    }


    @Override
    public Mat processFrame(Mat input) {

        telemetry.clearAll();
        telemetry.addData("count", count);
        count++;

        if (input == null)
            return null;

        if (lastImage != null) {
            lastImage.release();
        }

        lastImage = new Mat();
        input.copyTo(lastImage);

        //converting the video to a gray scale
        Mat grayImage = new Mat();
        Imgproc.cvtColor(lastImage, grayImage, Imgproc.COLOR_RGB2GRAY);

        //blurring the image
        Mat blurImage = new Mat();
        Imgproc.medianBlur(grayImage, blurImage, 5);

        //detecting hough circles
        Mat circleImage = new Mat();
        //decreasing param2 will have it detect more circles (possibly even too many)
        Imgproc.HoughCircles(grayImage, circleImage, Imgproc.HOUGH_GRADIENT, 1,40, 100, 65, 10,80);

        for (int i = 0; i < circleImage.cols(); i++ ) {
            double[] data = circleImage.get(0, i);
            //drawing circle that has been found
            Point center = new Point(Math.round(data[0]), Math.round(data[1]));
            // circle center
            Imgproc.circle(lastImage, center, 1, new Scalar(0, 0, 255), 3, 8, 0 );
            // circle outline
            int radius = (int) Math.round(data[2]);
            telemetry.addData("Radius: ", radius);
            Imgproc.circle(lastImage, center, radius, new Scalar(0,0,255), 3, 8, 0 );
        }
        if(circleImage.cols() == 1){
            telemetry.addData("1 circle", true);
        }
        if(circleImage.cols() == 2){
            telemetry.addData("2 circles", true);
        }
        if(circleImage.cols() == 3){
            telemetry.addData("3 circles", true);
        }
        telemetry.update();

        //return blurImage;
        return lastImage;
    }
}

