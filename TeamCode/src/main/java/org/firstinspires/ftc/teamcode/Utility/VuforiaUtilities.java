package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class VuforiaUtilities {

    public static final float MM_TO_INCHES = 0.0393701f;
    public static final float INCHES_TO_MM = 25.4f;

    // Constants for perimeter targets
    public static final float WALL_IMAGE_Z_OFFSET_INCHES = 6; // Images are 6 inches off the ground
    public static final float FULL_FIELD_LENGTH_INCHES = (12 * 12 - 2);  // the FTC field is ~11'10" center-to-center of the glass panels
    public static final float HALF_FIELD_LENGTH_INCHES = FULL_FIELD_LENGTH_INCHES / 2;

    public static final float WALL_IMAGE_Z_OFFSET_MM = WALL_IMAGE_Z_OFFSET_INCHES * INCHES_TO_MM;
    public static final float FULL_FIELD_LENGTH_MM = FULL_FIELD_LENGTH_INCHES * INCHES_TO_MM;
    public static final float HALF_FIELD_LENGTH_MM = HALF_FIELD_LENGTH_INCHES * INCHES_TO_MM;

    public static final int PIXEL_FORMAT_RGB565 = 1;

    public final static String VUFORIA_KEY = "AYhwTMH/////AAABmR7oFvU9lEJTryl5O3jDSusAPmWSAx5CHlcB/" +
            "IUoT+t7S1pJqTo7n3OwM4f2vVULA0T1uZVl9i61kWldhVqxK2+kyBNI4Uld8cYgHaNIQFsL/NsyBrb3Zl+1ZFBR" +
            "tpI5BjPnJkivkDsGU0rAFd+vPkyZt0p3/Uz+50eEwMZrZh499IsfooWkGX1wobjOFeA7DYQU+5ulhc1Rdp4mqjj" +
            "uKrS24Eop0MKJ+PwvNJhnN4LqIWQSfSABmcw9ogaeEsCzJdowrpXAcSo9d+ykJFZuB92iKN16lC9dRG3PABt26o" +
            "lSUCeXJrC4g6bEldHlmTc51nRpix6i1sGfvNuxlATzuRf5dtX/YlQm2WvvG9TilHbz";

    public static VuforiaLocalizer.Parameters getBackCameraParameters(final HardwareMap hardwareMap) {
        return getBackCameraParameters(hardwareMap, true);
    }

    public static VuforiaLocalizer.Parameters getBackCameraParameters(final HardwareMap hardwareMap,
                                                                      final boolean withView) {
        VuforiaLocalizer.Parameters parameters = getParameters(hardwareMap, withView);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        return parameters;
    }

    public static VuforiaLocalizer.Parameters getWebCameraParameters(final HardwareMap hardwareMap,
                                                                     final WebcamName webcamName) {
        return getWebCameraParameters(hardwareMap, webcamName, true);
    }

    public static VuforiaLocalizer.Parameters getWebCameraParameters(final HardwareMap hardwareMap,
                                                                     final WebcamName webcamName,
                                                                     final boolean withView) {
        VuforiaLocalizer.Parameters parameters = getParameters(hardwareMap, withView);
        parameters.cameraName = webcamName;
        return parameters;
    }

    private static VuforiaLocalizer.Parameters getParameters(final HardwareMap hardwareMap,
                                                             final boolean withView) {
        VuforiaLocalizer.Parameters parameters;
        if (withView) {
            int cameraMonitorViewId = getCameraMonitorViewId(hardwareMap);
            parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        } else {
            parameters = new VuforiaLocalizer.Parameters();
        }

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        // Prevents memory of previously seen images.
        parameters.useExtendedTracking = true;
        return parameters;
    }

    public static VuforiaLocalizer getVuforia(final VuforiaLocalizer.Parameters parameters) {
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.setFrameQueueCapacity(1);
        vuforia.enableConvertFrameToBitmap();
        return vuforia;
    }

    public static int getCameraMonitorViewId(final HardwareMap hardwareMap) {
        return hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    }

    public static int getTfodMonitorViewId(final HardwareMap hardwareMap) {
        return hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    }
}