package org.firstinspires.ftc.teamcode.Utility;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ColorUtilities {
    //TODO create constants for the saturation and light value thresholds and for the RED hue
    public static final int HUE_SAMPLE_RATE = 1;

    //TODO enhance this method to cover the capabilities currently in the method blackWhiteColorDecider
    //TODO enchant this method, pass in the hueDistribution to eliminate code duplication
    // Add a Bitmap parameter to be modified
    public static int[] getColorCount(Bitmap bitmap, int minHue, int maxHue, LinearOpMode opMode) throws InterruptedException {
        int[] counts = {0, 0};
        for (int x = 0; x < bitmap.getWidth(); x = x + HUE_SAMPLE_RATE) { // replace 200 with x pixel size value
            for (int y = 0; y < bitmap.getHeight(); y = y + HUE_SAMPLE_RATE) {
                if (opMode != null && !opMode.opModeIsActive()) {
                    throw new InterruptedException("");
                }

                int color = bitmap.getPixel(x, y);

                int red = Color.red(color);
                int green = Color.green(color);
                int blue = Color.blue(color);

                float[] HSV = new float[3];
                Color.RGBToHSV(red, green, blue, HSV);

                int hue = (int) HSV[0];

                if (HSV[1] < 0.7)//in gray scale protion of color scale.
                {
                    if (HSV[2] > 0.8)//white
                    {
                        counts[1]++;
                    }
                } else if (minHue <= hue && hue <= maxHue) {
                    counts[0]++;
                }
            }
        }

        return counts;
    }

    /**
     * create a Hue distribution for the image passed in.
     * <p>
     * Any color with a saturation less than 70%
     * is considered to be in the center white-gray-black cylinder. If the light value is greater
     * then 80% then the colour is considered to be white and is counted as RED Hue. If any other gray
     * scale value, its not counted at all.
     * <p>
     * Any Color with a saturation greater than or equal to 70% is counted in its associated
     * hue entry.
     * <p>
     * Hue values are integer values ranging from 0 to 360
     *
     * @param bitmap bitmap to compute hue distribution over
     * @param opMode opMode used to work around stuckInStop bug
     * @return An array of 360 hue counts
     * @throws InterruptedException thrown if stuckInStop is encountered because excessive time
     *                              has been spent processing the image
     */
    public static int[] getHueDistribution(Bitmap bitmap, LinearOpMode opMode) throws InterruptedException {
        int[] colorCounts = new int[361];

        for (int x = 0; x < bitmap.getWidth(); x = x + HUE_SAMPLE_RATE) {
            for (int y = 0; y < bitmap.getHeight(); y = y + HUE_SAMPLE_RATE) {
                if (opMode != null && !opMode.opModeIsActive()) {
                    throw new InterruptedException("");
                }

                int color = bitmap.getPixel(x, y);

                int red = Color.red(color);
                int green = Color.green(color);
                int blue = Color.blue(color);

                float[] HSV = new float[3];
                Color.RGBToHSV(red, green, blue, HSV);

                int hue = (int) HSV[0];

                int hueCount = colorCounts[hue];
                hueCount++;

                // Color with hue value < 70% are in the white-gray-black cylinder
                if (HSV[1] < 0.7) {
                    // Colors with a light value > 80% are considered to be white
                    if (HSV[2] > 0.8) {
                        // white is not represented on the hue domain. Map white to RED
                        colorCounts[360]++;
                        //TODO, count white as 361 (and increase array size to 362)
                    }
                } else {
                    colorCounts[hue] = hueCount;
                }
            }
        }

        return colorCounts;
    }

    //TODO deprecate this method once its capabilities are combined with getColorCounts
    public static Bitmap blackWhiteColorDecider(Bitmap bitmap, int minHue, int maxHue, LinearOpMode opMode) throws InterruptedException {
        Bitmap babyBitmapBW = Bitmap.createBitmap(bitmap.getWidth(), bitmap.getHeight(), bitmap.getConfig());
        int newColor = 0;
        for (int x = 0; x < bitmap.getWidth(); x++) { // replace 200 with x pixel size value
            for (int y = 0; y < bitmap.getHeight(); y++) {
                if (opMode != null && !opMode.opModeIsActive()) {
                    throw new InterruptedException("");
                }

                int color = bitmap.getPixel(x, y);

                int red = Color.red(color);
                int green = Color.green(color);
                int blue = Color.blue(color);

                float[] HSV = new float[3];
                Color.RGBToHSV(red, green, blue, HSV);

                int hue = (int) HSV[0];

                if (HSV[1] < 0.7)//in gray scale protion of color scale.
                {
                    if (HSV[2] > 0.8)//white
                    {
                        newColor = Color.WHITE;
                    } else if (HSV[2] < 0.2) //black
                    {
                        newColor = Color.BLACK;
                    } else {
                        newColor = Color.GRAY;

                    }
                    babyBitmapBW.setPixel(x, y, newColor);
                } else if (minHue <= hue && hue <= maxHue) {
                    HSV[1] = (float) 1.0;
                    HSV[2] = (float) 1.0;
                    babyBitmapBW.setPixel(x, y, Color.CYAN);
                } else {
                    HSV[1] = (float) 1.0;
                    HSV[2] = (float) 1.0;
                    newColor = Color.HSVToColor(HSV);
                    babyBitmapBW.setPixel(x, y, newColor);
                }
            }
        }
        return babyBitmapBW;
    }
}
