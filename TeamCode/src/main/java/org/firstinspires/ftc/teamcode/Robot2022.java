package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.CameraDevice;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

public class Robot2022 extends Robot {
    DcMotor RF, LF, RB, LB;
    //Servo S1, S2;

    VuforiaLocalizerImpl vuforia;

    @Override
    void init() { //Инициализация:

        LF = hwmp.get(DcMotor.class, "LF");
        LB = hwmp.get(DcMotor.class, "LB");
        RB = hwmp.get(DcMotor.class, "RB");
        RF = hwmp.get(DcMotor.class, "RF");

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //Режим остоновки: торможение
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //S1 = hwmp.get(Servo.class, "S1");
        //S2 = hwmp.get(Servo.class, "S2");

        //initVuforia();

    }

    @Override
    void initFields(Telemetry telemetry, LinearOpMode L, HardwareMap hwmp) { //Инициализация
        this.telemetry = telemetry;
        this.L = L;
        this.hwmp = hwmp;

    }

    public void initVuforia() {
        final String VUFORIA_KEY = "AXmpKJj/////AAABmcT291bCTEQeoCRi1kDOyP8ApoUammAer00aO1owWHeTV7AmOtKkjy/8jRV99nQLFDMqq8eFrFk02tC3e24Hk9u4pnB+m2zRTTtVlIJ9G248PtXINEGUoPi+W2t53dbLT5+RSxBdMGDAKC7aeTv0tlpN1zNLnxYbVKqgbsIKU5C5FOGByrJU7xGP/qLDsY/TAlNbcq73vL9ClSAGo0Im+77mABDEXUVZilP05IR5sbXJYHo/J9O2U8ZfX4KnpnNbPWzzGBFpyKrVRNYihX7s/pjlitY6Fok2sQ+PX4XDoCu3dw/9rtnqpMwTkBtrzvmVuL01zVmKcf8e31FWafJ2I1CBJ5t2OJbrOO0m4NiELoUL";

        OpenGLMatrix targetPose     = null;
        String targetName           = "";

        int cameraMonitorViewId = hwmp.appContext.getResources().getIdentifier("Camera", "id", hwmp.appContext.getPackageName());
        VuforiaLocalizer.Parameters parametersWebcam = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parametersWebcam.vuforiaLicenseKey = VUFORIA_KEY;
        parametersWebcam.useExtendedTracking = false;
        parametersWebcam.cameraName = hwmp.get(WebcamName.class, "Webcam");
        parametersWebcam.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = new VuforiaLocalizerImpl(parametersWebcam);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);
        CameraDevice.getInstance().setFlashTorchMode(true);


    }

    Bitmap getImage() throws InterruptedException {
        Image img;
        img = getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565);
        Bitmap btm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
        btm.copyPixelsFromBuffer(img.getPixels());
        return btm;

    }

    Image getImageFromFrame(VuforiaLocalizer.CloseableFrame frame, int format) {
        long NI = frame.getNumImages();
        for (int i = 0; i < NI; i++) {
            if (frame.getImage(i).getFormat() == format) {
                return frame.getImage(i);
            }
        }
        return null;
    }

    boolean ConusRq(double rr, double gr, double br, double angle, double r, double g, double b) {
        double anglerr = Math.acos((rr * r + gr * g + br * b) / (Math.sqrt(rr * rr + gr * gr + br * br) * Math.sqrt(r * r + g * g + b * b)));
        telemetry.addData("anglerr", anglerr);
        telemetry.addData("anglenon", angle);
        if (angle > anglerr) {
            return true;
        }
        return false;
    }


    void DEBUG() {
        telemetry.addData("debug", "debug");
        telemetry.update();
        RB.setPower(-1);
        RF.setPower(1);
        LF.setPower(-1);
        LB.setPower(1);
        delay(3000);
        RB.setPower(0);
        RF.setPower(0);
        LF.setPower(0);
        LB.setPower(0);

    }

    void wheelbase() {

    }


}