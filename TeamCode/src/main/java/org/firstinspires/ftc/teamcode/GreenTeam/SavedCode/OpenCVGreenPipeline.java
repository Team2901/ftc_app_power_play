package org.firstinspires.ftc.teamcode.GreenTeam.SavedCode;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.Rect;
import org.opencv.core.Core;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OpenCVGreenPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    Rect space1 = new Rect(8, 110, 50, 50);
    Mat slot1;
    Rect space2 = new Rect(126, 120, 50, 50);
    Mat slot2;
    Rect space3 = new Rect(230, 120, 50, 50);
    Mat slot3;
    int slotSelected = -1;


    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public int getSlotSelected(){
        return slotSelected;
    }


    @Override
    public Mat processFrame(Mat inputMat) {
        //Converting the input image to HSV?
        Imgproc.cvtColor(inputMat, inputMat, Imgproc.COLOR_RGB2HSV);
        //Fuzzing up the picture
        Imgproc.GaussianBlur(inputMat, inputMat, new Size(3, 3), 0);
        //Drawing rectangles around the 3 duck spaces
        Imgproc.rectangle(inputMat, space1, new Scalar(0, 0, 255), 3);
        Imgproc.rectangle(inputMat, space2, new Scalar(0, 0, 255), 3);
        Imgproc.rectangle(inputMat, space3, new Scalar(0, 0, 255), 3);
        //taking out the chunks so we can use them
        slot1 = inputMat.submat(space1);
        slot2 = inputMat.submat(space2);
        slot3 = inputMat.submat(space3);

        //Checks the average hue of the area inside the rectangle and prints it out
        double space1Color = Core.mean(slot1).val[0];
        telemetry.addData("Space 1", space1Color);
        double space2Color = Core.mean(slot2).val[0];
        telemetry.addData("Space 2", space2Color);
        double space3Color = Core.mean(slot3).val[0];
        telemetry.addData("Space 3", space3Color);

        //checks if the value is around yellow
        if(space1Color > 30 && space1Color < 90){
            slotSelected = 1;
            Imgproc.rectangle(inputMat, space1, new Scalar(0, 255, 0), 3);
        }else if(space2Color > 30 && space2Color < 90){
            slotSelected = 2;
            Imgproc.rectangle(inputMat, space2, new Scalar(0, 255, 0), 3);
        }else if(space3Color > 30 && space3Color < 90){
            slotSelected = 3;
            Imgproc.rectangle(inputMat, space3, new Scalar(0, 255, 0), 3);
        }

        /*
        if(space1Color >= space2Color && space1Color >= space3Color){
            slotSelected = 1;
            Imgproc.rectangle(inputMat, space1, new Scalar(0, 255, 0), 3);
        } else if (space2Color >= space3Color && space2Color >= space1Color ){
            slotSelected = 2;
            Imgproc.rectangle(inputMat, space2, new Scalar(0, 255, 0), 3);
        } else if (space3Color >= space2Color){
            slotSelected = 3;
            Imgproc.rectangle(inputMat, space3, new Scalar(0, 255, 0), 3); //cope diem lmao
        }
         */
        telemetry.addData("Selected", slotSelected);
        telemetry.update();
        return inputMat;
    }
}
