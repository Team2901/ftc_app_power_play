package org.firstinspires.ftc.teamcode.Utility;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class FileUtilities {
    public final static String PICTURES_FOLDER_NAME = "Team";
    public final static String TEAM_FOLDER_NAME = "Team";

    public static void writeConfigFile(String filename,
                                       List<? extends Object> config) throws IOException {

        final File teamDir = new File(Environment.getExternalStorageDirectory(), TEAM_FOLDER_NAME);
        boolean newDir = teamDir.mkdirs();
        final File file = new File(teamDir, filename);

        try (BufferedWriter writer = new BufferedWriter(new FileWriter(file))) {
            for (Object o : config) {
                writer.write(o.toString());
                writer.newLine();
            }
        }
    }

    public static void writeConfigFile(String filename, Object value) throws IOException {
        final File teamDir = new File(Environment.getExternalStorageDirectory(), TEAM_FOLDER_NAME);
        boolean newDir = teamDir.mkdirs();
        final File file = new File(teamDir, filename);

        try (BufferedWriter writer = new BufferedWriter(new FileWriter(file))) {
            writer.write(value.toString());
        }
    }

    public static int readTeamColor(String filename) throws IOException {
        final File teamDir = new File(Environment.getExternalStorageDirectory(), TEAM_FOLDER_NAME);
        boolean newDir = teamDir.mkdirs();
        final File file = new File(teamDir, filename);

        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            String line = reader.readLine();
            return Integer.valueOf(line);
        }
    }

    public static List<String> readConfigFile(String filename) throws IOException {

        final File teamDir = new File(Environment.getExternalStorageDirectory(), TEAM_FOLDER_NAME);
        boolean newDir = teamDir.mkdirs();
        final File file = new File(teamDir, filename);

        final List<String> config = new ArrayList<>();

        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            String line = reader.readLine();
            while (line != null) {
                config.add(line);
                line = reader.readLine();
            }
        }

        return config;
    }

    public static List<Integer> readIntegerConfigFile(String filename) throws IOException {

        final List<String> stringConfig = readConfigFile(filename);

        final List<Integer> config = new ArrayList<>();
        for (String string : stringConfig) {
            config.add(Integer.valueOf(string));
        }

        return config;
    }

    public static List<Double> readDoubleConfigFile(String filename) throws IOException {

        final List<String> stringConfig = readConfigFile(filename);

        final List<Double> config = new ArrayList<>();
        for (String string : stringConfig) {
            config.add(Double.valueOf(string));
        }

        return config;
    }

    public static void writeBitmapFile(String filename, Bitmap bitmap) throws IOException {
        final String filePath = Environment.getExternalStorageDirectory().getAbsolutePath() + "/" + PICTURES_FOLDER_NAME;

        try (FileOutputStream fileOutputStream = new FileOutputStream(filePath + "/" + filename)) {
            bitmap.compress(Bitmap.CompressFormat.PNG, 100, fileOutputStream);
            fileOutputStream.flush();
        }
    }

    public static Bitmap readBitmapFile(String filename) {

        final String filePath = Environment.getExternalStorageDirectory().getAbsolutePath() + "/" + PICTURES_FOLDER_NAME;
        final File image = new File(filePath, filename);

        final BitmapFactory.Options bmOptions = new BitmapFactory.Options();
        Bitmap bitmap;

        try {
            bitmap = BitmapFactory.decodeFile(image.getAbsolutePath(), bmOptions);
        } catch (Exception e) {
            bitmap = null;
        }

        return bitmap;
    }

    public static void writeHueFile(String filename,
                                    Bitmap bitmap,
                                    LinearOpMode opMode) throws IOException, InterruptedException {
        int[] colorCounts = ColorUtilities.getHueDistribution(bitmap, opMode);

        final File teamDir = new File(Environment.getExternalStorageDirectory(), TEAM_FOLDER_NAME);
        boolean newDir = teamDir.mkdirs();
        final File file = new File(teamDir, filename);

        try (BufferedWriter writer = new BufferedWriter(new FileWriter(file))) {
            for (int i = 0; i < colorCounts.length; i++) {
                int colorCount = colorCounts[i];
                writer.write(String.valueOf(colorCount));
                writer.newLine();
            }
        }
    }
}
