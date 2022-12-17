package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
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
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

public class Robot2022 extends Robot {
    DcMotor RF, LF, RB, LB, LT;
    Servo KL;
    BNO055IMU imu; //Акселерометр

    VuforiaLocalizerImpl vuforia;

    Orientation angles;
    Acceleration gravity;

    @Override
    void init() { //Инициализация:

        LF = hwmp.get(DcMotor.class, "LF");
        LB = hwmp.get(DcMotor.class, "LB");
        RB = hwmp.get(DcMotor.class, "RB");
        RF = hwmp.get(DcMotor.class, "RF");

        LT = hwmp.get(DcMotor.class, "LT");

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //Режим остоновки: торможение
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        KL = hwmp.get(Servo.class, "KL");

        initVuforia();


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); //Акселерометра
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwmp.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        while (!imu.isGyroCalibrated()) { //Калибровка акселерометра
            delay(30);
            telemetry.addData("Wait", "Calibration"); //Сообщение о калибровке
            telemetry.update();
        }
        telemetry.addData("Done!", "Calibrated"); //Сообщение об окончании калибровки
        telemetry.update();
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
        //telemetry.addData("anglerr", anglerr);
        //telemetry.addData("anglenon", angle);
        if (angle > anglerr) {
            return true;
        }
        return false;
    }

    void Katet(double cm, int direction) { //direction: 1 - forward, 2 - right, 3 - backward, 4 - left
        double lbs = 1; // lb signum
        double rbs = 1;
        double lfs = 1;
        double rfs = 1;
        double kc = 1;
        if (direction == 1) {
            rbs = -1;
            rfs = -1;
        }
        if (direction == 2) {
            lbs = -1;
            rbs = -1;
        }
        if (direction == 3) {
            lbs = -1;
            lfs = -1;
            //kc = 2;
        }
        if (direction == 4) {
            lfs = -1;
            rfs = -1;
            //kc = 2;
        }
        double pw = 1;
        double ccx = ((400 * cm) / 32.97)*kc;
        double cc = ccx/Math.sqrt(2);
        double Er0 = cc;
        double ErLast = 0;
        double D = 1;
        double startAngle = getAngle();
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while ( ( Math.abs(D) > 0.1 ||  Math.abs(cc) - Math.abs(LF.getCurrentPosition()) > 10*Math.signum(cc)) && L.opModeIsActive()) {

            double Er = Math.abs(cc) - Math.abs(LF.getCurrentPosition());

            double kp = 0.8;
            double P = kp * Er / Er0 * pw ;

            double kd = 0.2;
            double ErD = Er - ErLast;
            D = kd * ErD * (1 / Er);

            if (Math.signum(D) > Math.signum(P)) {  D=P; }

            double kr = 0.2*Math.signum(cc);
            double Rele = kr * Math.signum(Er);

            ErLast = Er;

            double lbr = 0; // lb rotate
            double lfr = 0;
            double rfr = 0;
            double rbr = 0;
            double rotP =0;

            if ( Math.abs(startAngle - getAngle()) > 1 ) {
                double rotkp = 0.03;
                rotP = -(rotkp * (startAngle - getAngle()));
                if ( direction == 1 ) {
                    rfr += rotP;
                    rbr += rotP;
                }
                if ( direction == 2 ) {
                    lbr += rotP;
                    rbr += rotP;
                }
                if ( direction == 3 ) {
                    lfr += rotP;
                    lbr += rotP;
                }
                if ( direction == 4 ) {
                    rfr += rotP;
                    lfr += rotP;
                }
            }


            double pwf = (pw * (P+D+Rele))*Math.signum(Er); //Регулятор

            if ( rotP > pwf ) { rotP = pwf*0.6; }

            //telemetry.addData("currPosition", LF.getCurrentPosition());
            //telemetry.addData("cc", cc);
            //telemetry.addData("Err", Er);
            //telemetry.addData("cond1", cc - LF.getCurrentPosition());
            //telemetry.addData("cond2", 10*Math.signum(cc));
            //telemetry.addData("pwf", pwf);
            //telemetry.addData("D", D);
            //telemetry.update();

            setMtPower(pwf*lfs+lfr, pwf*lbs+lbr, pwf*rfs+rfr, pwf*rbs+rbr);


            /*telemetry.addData("cc", cc);
            telemetry.addData("Er0", Er0);
            telemetry.addData("Er", Er);
            telemetry.addData("getCurrentPosition", LB.getCurrentPosition());
            telemetry.addData("kp", kp);
            telemetry.addData("Rele", Rele);
            telemetry.addData("D", D);
            telemetry.addData("pw", pw);
            telemetry.addData("pwf", pwf);
            telemetry.addData("rotEr", startAngle-getAngle());
            telemetry.addData("lfr", lfr);
            telemetry.addData("lbr", lbr);
            telemetry.addData("rfr", rfr);
            telemetry.addData("rbr", rbr);
            telemetry.addData("rotP", rotP);
            telemetry.update();*/

        }

        delay(500);

        while ( Math.abs(startAngle - getAngle()) > 1  && L.opModeIsActive()) {

            double kp = 0.02;
            double P = -(kp * (startAngle - getAngle()));

            setMtPower(P, P, P, P);

        }

        setMtPower(0, 0, 0, 0);

        delay(500);
    }

    void katetPlus(double cm, int direction, double kp, double kd, boolean isRot) { //direction: 1 - forward, 2 - right, 3 - backward, 4 - left
        double lbs = 1; // lb signum
        double rbs = 1;
        double lfs = 1;
        double rfs = 1;
        double kc = 1;
        if (direction == 1) {
            rbs = -1;
            rfs = -1;
        }
        if (direction == 2) {
            lbs = -1;
            rbs = -1;
        }
        if (direction == 3) {
            lbs = -1;
            lfs = -1;
            //kc = 2;
        }
        if (direction == 4) {
            lfs = -1;
            rfs = -1;
            //kc = 2;
        }
        double pw = 1;
        double ccx = ((400 * cm) / 32.97)*kc;
        double cc = ccx/Math.sqrt(2);
        double Er0 = cc;
        double ErLast = 0;
        double D = 1;
        double startAngle = getAngle();
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while ( ( Math.abs(D) > 0.1 ||  Math.abs(cc) - Math.abs(LF.getCurrentPosition()) > 10*Math.signum(cc)) && L.opModeIsActive()) {

            double Er = Math.abs(cc) - Math.abs(LF.getCurrentPosition());

            double P = kp * Er / Er0 * pw ;

            double ErD = Er - ErLast;
            D = kd * ErD * (1 / Er);

            if (Math.signum(D) > Math.signum(P)) {  D=P; }

            double kr = 0.2*Math.signum(cc);
            double Rele = kr * Math.signum(Er);

            ErLast = Er;

            double lbr = 0; // lb rotate
            double lfr = 0;
            double rfr = 0;
            double rbr = 0;
            double rotP =0;

            if ( Math.abs(startAngle - getAngle()) > 1 && isRot ) {
                double rotkp = 0.03;
                rotP = -(rotkp * (startAngle - getAngle()));
                if ( direction == 1 ) {
                    rfr += rotP;
                    rbr += rotP;
                }
                if ( direction == 2 ) {
                    lbr += rotP;
                    rbr += rotP;
                }
                if ( direction == 3 ) {
                    lfr += rotP;
                    lbr += rotP;
                }
                if ( direction == 4 ) {
                    rfr += rotP;
                    lfr += rotP;
                }
            }


            double pwf = (pw * (P+D+Rele))*Math.signum(Er); //Регулятор

            if ( rotP > pwf ) { rotP = pwf*0.6; }

            //telemetry.addData("currPosition", LF.getCurrentPosition());
            //telemetry.addData("cc", cc);
            //telemetry.addData("Err", Er);
            //telemetry.addData("cond1", cc - LF.getCurrentPosition());
            //telemetry.addData("cond2", 10*Math.signum(cc));
            //telemetry.addData("pwf", pwf);
            //telemetry.addData("D", D);
            //telemetry.update();

            setMtPower(pwf*lfs+lfr, pwf*lbs+lbr, pwf*rfs+rfr, pwf*rbs+rbr);


            /*telemetry.addData("cc", cc);
            telemetry.addData("Er0", Er0);
            telemetry.addData("Er", Er);
            telemetry.addData("getCurrentPosition", LB.getCurrentPosition());
            telemetry.addData("kp", kp);
            telemetry.addData("Rele", Rele);
            telemetry.addData("D", D);
            telemetry.addData("pw", pw);
            telemetry.addData("pwf", pwf);
            telemetry.addData("rotEr", startAngle-getAngle());
            telemetry.addData("lfr", lfr);
            telemetry.addData("lbr", lbr);
            telemetry.addData("rfr", rfr);
            telemetry.addData("rbr", rbr);
            telemetry.addData("rotP", rotP);
            telemetry.update();*/

        }

        delay(500);

        while ( Math.abs(startAngle - getAngle()) > 1  && L.opModeIsActive() && isRot ) {

            kp = 0.2;
            double P = -(kp * (startAngle - getAngle()));

            setMtPower(P, P, P, P);

        }

        setMtPower(0, 0, 0, 0);

        delay(500);
    }

    void go(double cm) { //
        double pw = 1;
        double cc = (400 * cm) / 32.97;
        double Er0 = cc;
        double errorFix=0;
        double ErLast = 0;
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /*if (degrees < -180) {
            degrees += 360;
            pw = pw * -1;
        }
        if (degrees > 180) {
            degrees -= 360;
            pw = pw * -1;
        }*/
        double D = 1;
        //while (LB.getCurrentPosition() < m) { setMtPower(-pw, -pw, pw, pw); }
        while ( ( Math.abs(D) > 0.1 ||  Math.abs(cc) - Math.abs(LF.getCurrentPosition()) > 10*Math.signum(cc)) && L.opModeIsActive()) {

            double Er = Math.abs(cc) - Math.abs(LF.getCurrentPosition());

            double kp = 0.9;
            double P = kp * Er / Er0 * pw;

            double kd = 0.15;
            double ErD = Er - ErLast;
            D = kd * ErD * (1 / Er);

            if (Math.signum(D) > Math.signum(P)) {  D=P; }

            double kr = 0.2*Math.signum(cc);
            double Rele = kr * Math.signum(Er);

            ErLast = Er;



            double pwf = (pw * (P+D+Rele))*Math.signum(Er); //Регулятор

            //telemetry.addData("currPosition", LF.getCurrentPosition());
            //telemetry.addData("cc", cc);
            //telemetry.addData("Err", Er);
            //telemetry.addData("cond1", cc - LF.getCurrentPosition());
            //telemetry.addData("cond2", 10*Math.signum(cc));
            //telemetry.addData("pwf", pwf);
            //telemetry.addData("D", D);
            //telemetry.update();

            LF.setPower(pwf);
            RB.setPower(-pwf);


            /*telemetry.addData("cc", cc);
            telemetry.addData("Er0", Er0);
            telemetry.addData("Er", Er);
            telemetry.addData("getCurrentPosition", LB.getCurrentPosition());
            telemetry.addData("kp", kp);
            telemetry.addData("Rele", Rele);
            telemetry.addData("D", D);
            telemetry.addData("pw", pw);
            telemetry.addData("pwf", pwf);
            telemetry.update();*/

        }

        LF.setPower(0);
        RB.setPower(0);

        delay(500);
    }

    double getAngle() { //Функция получения данных с акселерометра
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();
        return angles.firstAngle;
    }

    void rotate (double degrees) { //Функция автонома: поворот
        double pw = 1;
        double Er0 = -degrees;
        double errorFix=0;
        double ErLast = 0;
        if (Er0 > 0) { pw = -1; }
        degrees = getAngle() - degrees;
        if (degrees < -180) {
            //degrees += 360;
            Er0 = Er0 * -1;
            pw *= -1;
            errorFix=1;
        }
        if (degrees > 180) {
            //degrees -= 360;
            Er0 = Er0 * -1;
            pw *= -1;
            errorFix=2;
        }
        while ( Math.abs(degrees - getAngle()) > 3  && L.opModeIsActive()) {
            if (getAngle() > 0 && errorFix==1) { Er0 = Er0 * -1; degrees += 360; pw *= -1; errorFix=0; }
            if (getAngle() < 0 && errorFix==2) { Er0 = Er0 * -1; degrees -= 360; pw *= -1; errorFix=0; }

            double Er = degrees - (getAngle());

            double kp = 0.6;
            double P = kp * Er / Er0 * pw;

            double kd = 0.2;
            double ErD = Er - ErLast;
            double D = kd * ErD * (1/Er);

            if (Math.signum(D) > Math.signum(P)) {  D=P; }

            double kr = -0.025;
            double Rele = kr * Math.signum(Er);

            ErLast = Er;


            double pwf = P + Rele; //Регулятор


            LB.setPower(pwf);
            LF.setPower(pwf);
            RB.setPower(pwf);
            RF.setPower(pwf);

                /*telemetry.addData("degrees", degrees);
                telemetry.addData("getAngle()", getAngle());
                telemetry.addData("Er (degrees - getAngle)", Er);
                telemetry.addData("Er0", Er0);
                telemetry.addData("kp", kp);
                telemetry.addData("P", P);
                telemetry.addData("D", D);
                telemetry.addData("Rele", Rele);
                telemetry.addData("pw", pw);
                telemetry.addData("pwf", pwf);
                telemetry.update();*/

        }
        LB.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        RF.setPower(0);
        delay(500);
    }

    void rotateS(double degrees) { //Функция автонома: поворот
        double pw = 1;
        double Er0 = -degrees;
        double errorFix=0;
        double ErLast = 0;
        if (Er0 > 0) { pw = -1; }
        degrees = getAngle() - degrees;
        if (degrees < -180) {
            //degrees += 360;
            Er0 = Er0 * -1;
            pw *= -1;
            errorFix=1;
        }
        if (degrees > 180) {
            //degrees -= 360;
            Er0 = Er0 * -1;
            pw *= -1;
            errorFix=2;
        }
        while ( Math.abs(degrees - getAngle()) > 3  && L.opModeIsActive()) {
            if (getAngle() > 0 && errorFix==1) { Er0 = Er0 * -1; degrees += 360; pw *= -1; errorFix=0; }
            if (getAngle() < 0 && errorFix==2) { Er0 = Er0 * -1; degrees -= 360; pw *= -1; errorFix=0; }

            double Er = degrees - (getAngle());

            double kp = 0.4;
            double P = kp * Er / Er0 * pw;

            double kd = 0.1;
            double ErD = Er - ErLast;
            double D = kd * ErD * (1/Er);

            if (Math.signum(D) > Math.signum(P)) {  D=P; }

            double kr = -0.025;
            double Rele = kr * Math.signum(Er);

            ErLast = Er;


            double pwf = P + Rele; //Регулятор


            LB.setPower(pwf);
            LF.setPower(pwf);
            RB.setPower(pwf);
            RF.setPower(pwf);

                /*telemetry.addData("degrees", degrees);
                telemetry.addData("getAngle()", getAngle());
                telemetry.addData("Er (degrees - getAngle)", Er);
                telemetry.addData("Er0", Er0);
                telemetry.addData("kp", kp);
                telemetry.addData("P", P);
                telemetry.addData("D", D);
                telemetry.addData("Rele", Rele);
                telemetry.addData("pw", pw);
                telemetry.addData("pwf", pwf);
                telemetry.update();*/

        }
        LB.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        RF.setPower(0);
        delay(500);
    }

    void DEBUG() {
        telemetry.addData("debug", "debug");
        telemetry.update();
        RB.setPower(-1);
        RF.setPower(-1);
        LF.setPower(1);
        LB.setPower(1);
        delay(3000);
        RB.setPower(0);
        RF.setPower(0);
        LF.setPower(0);
        LB.setPower(0);

    }

    void wheelbase() {
        double rf = gamepad1.left_stick_y+gamepad1.left_stick_x+(gamepad1.right_stick_x*0.6)-(gamepad1.left_trigger*0.6)+(gamepad1.right_trigger*0.6);
        double rb = gamepad1.left_stick_y-gamepad1.left_stick_x+(gamepad1.right_stick_x*0.6)-(gamepad1.left_trigger*0.6)+(gamepad1.right_trigger*0.6);
        double lf = -gamepad1.left_stick_y+gamepad1.left_stick_x+(gamepad1.right_stick_x*0.6)-(gamepad1.left_trigger*0.6)+(gamepad1.right_trigger*0.6);
        double lb = -gamepad1.left_stick_y-gamepad1.left_stick_x+(gamepad1.right_stick_x*0.6)-(gamepad1.left_trigger*0.6)+(gamepad1.right_trigger*0.6);
        RF.setPower(rf);
        RB.setPower(rb);
        LF.setPower(lf);
        LB.setPower(lb);
    }

    Thread liftControllerT = new Thread() { //Поток для лифта
        @Override
        public void run() {
            boolean hold = false;
            double Power = 0;
            while (L.opModeIsActive() && !L.isStopRequested()) {
                telemetry.addData("y", gamepad2.left_stick_y);
                telemetry.update();
                LT.setPower((gamepad2.right_stick_y/-1.7)+Power); //Управление лифтом стиком
                if (gamepad2.y) { //Поднять до конца
                    LT.setPower(0.75);  //начальное ускорение
                    delay(200);
                    LT.setPower(0.4);    //спокойная скорость
                    delay(350);
                    LT.setPower(0);      //стоп
                    Power = 0.14;
                    hold = true;
                }
                if (gamepad2.a) {
                    if ( hold ) {
                        Power = 0;
                        hold = false;
                        delay(300);
                    }
                    else {
                        Power = 0.24;
                        hold = true;
                        delay(300);
                    }
                }
            }
        }
    };


    void setMtPower(double lf, double lb, double rf, double rb) {
        LF.setPower(lf);
        LB.setPower(lb);
        RF.setPower(rf);
        RB.setPower(rb);
    }

    int MgI = 0;
    int GrI = 0;
    int CnI = 0;
    void analyze(int red, int green, int blue) {
        boolean Mg = ConusRq(220, 70, 120, 0.2, red, green, blue);
        boolean Gr = ConusRq(50, 120, 50, 0.3, red, green, blue);
        boolean Cn = ConusRq(50, 130, 140, 0.15, red, green, blue);
        if (Mg) {MgI=MgI+1; telemetry.addData("Mg detecked! Total count", MgI); }
        if (Gr) {GrI=GrI+1; telemetry.addData("Gr detecked! Total count", GrI); }
        if (Cn) {CnI=CnI+1; telemetry.addData("Cn detecked! Total count", CnI); }
    }

    //double servoPos=0.1;
    void servoController() {
        if (gamepad2.left_bumper ) {
            KL.setPosition(0.4); }
        if (gamepad2.right_bumper) {
            KL.setPosition(0.7); }
    }

    void liftUp() {
        LT.setPower(0.6);
        delay(1000);
    }
}