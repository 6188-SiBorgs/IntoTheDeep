package org.firstinspires.ftc.teamcode.xendy;

import android.os.Environment;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;

public class AutonomousSwitcherHelpers {
    public static final String PATH = Environment.getExternalStorageDirectory().getAbsolutePath() + "/XendysPessimisticAutonomous/";
    public static String G1A = "";
    public static String G1B = "";
    public static String G1X = "";
    public static String G1Y = "";
    public static String G1DpadUp = "";
    public static String G1DpadDown = "";
    public static String G1DpadRight = "";
    public static String G1DpadLeft = "";

    public static String selected = "";
    public static String getSelectedName() {
        if (selected.equals("G1A")) return G1A;
        if (selected.equals("G1B")) return G1B;
        if (selected.equals("G1X")) return G1X;
        if (selected.equals("G1Y")) return G1Y;
        if (selected.equals("G1DpadDown")) return G1DpadDown;
        if (selected.equals("G1DpadLeft")) return G1DpadLeft;
        if (selected.equals("G1DpadRight")) return G1DpadRight;
        if (selected.equals("G1DpadUp")) return G1DpadUp;
        return "";
    }

    public static void update() {
        try {
            G1A = getNameOf("G1A");
            G1B = getNameOf("G1B");
            G1X = getNameOf("G1X");
            G1Y = getNameOf("G1Y");
            G1DpadUp = getNameOf("G1DpadUp");
            G1DpadDown = getNameOf("G1DpadDown");
            G1DpadRight = getNameOf("G1DpadRight");
            G1DpadLeft = getNameOf("G1DpadLeft");
        }
        catch (IOException | ClassNotFoundException ignored) {

        }

        getPrimary();
    }

    public static String getNameOf(String path) throws IOException, ClassNotFoundException {
        File f = new File(PATH + path + ".robotStates");
        if(f.exists() && !f.isDirectory()) {
            return loadStatesFromFile(path).name;
        }
        else {
            return "";
        }
    }

    public static PathData loadStatesFromFile(String name) throws IOException, ClassNotFoundException {
        FileInputStream fileInputStream = new FileInputStream(PATH + name + ".robotStates");
        ObjectInputStream objectInputStream = new ObjectInputStream(fileInputStream);
        PathData data = (PathData) objectInputStream.readObject();
        objectInputStream.close();
        return data;
    }

    public static String getPrimary() {
        File f = new File(PATH + "selection.uwu");
        if(f.exists() && !f.isDirectory()) {
            try {
                FileInputStream fileInputStream = new FileInputStream(PATH + "selection");
                ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream();

                int readByte;
                while ((readByte = fileInputStream.read()) != -1) {
                    byteArrayOutputStream.write(readByte);
                }

                return byteArrayOutputStream.toString(StandardCharsets.UTF_8.name());
            }
            catch (IOException ignore){}
        }
        else {
            setPrimary("G1A");
            return "G1A";
        }
        return "";
    }

    public static void setPrimary(String p) {
        try {
            FileOutputStream fileOutputStream = new FileOutputStream(PATH + "selection.uwu");
            fileOutputStream.write(p.getBytes(StandardCharsets.UTF_8));
            selected = p;
        }
        catch (IOException ignore){}
    }
}
