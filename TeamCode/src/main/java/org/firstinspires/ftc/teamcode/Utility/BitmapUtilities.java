package org.firstinspires.ftc.teamcode.Utility;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.Paint;

import com.vuforia.Image;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.teamcode.RoverRuckus.Autonomous.BaseRoverRuckusAuto;

import java.io.IOException;
import java.util.List;

public class BitmapUtilities {

  public static final int PIXEL_FORMAT_RGB565 = 1;

  public static Bitmap getBabyBitmap(Bitmap bitmap, List<Integer> config) {
    return getBabyBitmap(bitmap, config.get(0), config.get(1), config.get(2), config.get(3));
  }

  public static Bitmap getBabyBitmap(Bitmap bitmap,
                                     int sampleLeftXPct, int sampleTopYPct,
                                     int sampleRightXPct, int sampleBotYPct) {
    int startXPx = (int) ((sampleLeftXPct / 100.0) * bitmap.getWidth());
    int startYPx = (int) ((sampleTopYPct / 100.0) * bitmap.getHeight());
    int endXPx = (int) ((sampleRightXPct / 100.0) * bitmap.getWidth());
    int endYPx = (int) ((sampleBotYPct / 100.0) * bitmap.getHeight());
    int width = endXPx - startXPx;
    int height = endYPx - startYPx;

    return Bitmap.createBitmap(bitmap, startXPx, startYPx, width, height);
  }

  public static Bitmap drawSamplingBox(String filename, Bitmap bitmap,
                                       int sampleLeftXPct, int sampleTopYPct,
                                       int sampleRightXPct, int sampleBotYPct) throws IOException {
    double xPercent = (bitmap.getWidth()) / 100.0;
    double yPercent = (bitmap.getHeight()) / 100.0;
    Bitmap mutableBitmap = bitmap.copy(Bitmap.Config.ARGB_8888, true);
    Canvas c = new Canvas(mutableBitmap);
    Paint p = new Paint();
    p.setARGB(100, 0, 200, 0);
    c.drawRect((int) (sampleLeftXPct * xPercent),
            (int) (sampleTopYPct * yPercent),
            (int) (sampleRightXPct * xPercent),
            (int) (sampleBotYPct * yPercent), p);
    //writeBitmapFile(filename, mutableBitmap);
    return mutableBitmap;
  }

  public static Bitmap getVuforiaImage(VuforiaLocalizer vuforia) {
    if (vuforia != null) {
      try {
        VuforiaLocalizer.CloseableFrame closeableFrame = vuforia.getFrameQueue().take();
        for (int i = 0; i < closeableFrame.getNumImages(); i++) {
          Image image = closeableFrame.getImage(i);
          if (image.getFormat() == PIXEL_FORMAT_RGB565) {
            Bitmap bitmap = Bitmap.createBitmap(image.getWidth(),
                    image.getHeight(),
                    Bitmap.Config.RGB_565);
            bitmap.copyPixelsFromBuffer(image.getPixels());
            Matrix matrix = new Matrix();
            matrix.postRotate(90);

            return Bitmap.createBitmap(bitmap, 0, 0, bitmap.getWidth(), bitmap.getHeight(), matrix, true);
          }
        }
      } catch (Exception e) {
        return null;
      }
    }
    return null;
  }
}

