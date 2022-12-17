package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="КР-СЛВ: Нижний > Сигнал", group="red")
public class RedDownColorParking extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2022 R = new Robot2022();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        R.KL.setPosition(0.7);
        while ( !isStarted() ) {
            R.MgI = 0;
            R.GrI = 0;
            R.CnI = 0;

            telemetry.addData("State", "Detecking");
            telemetry.update();

            for (int i = 250; i < 390; i = i + 10) {
                for (int l = 150; l < 210; l = l + 10) {
                    int Red = Color.red(R.getImage().getPixel(i, l));
                    int Green = Color.green(R.getImage().getPixel(i, l));
                    int Blue = Color.blue(R.getImage().getPixel(i, l));
                    R.analyze(Red, Green, Blue);
                    telemetry.addData("i", i);
                    telemetry.addData("l", l);
                    telemetry.update();
                }
            }
            telemetry.addData("Exit from for.", R.MgI);
        }
        waitForStart();

        //okay, let's go!
        R.KL.setPosition(0.4);

        String s = "Ns";
        if (R.MgI > R.GrI && R.MgI > R.CnI) {
            s = "Mg";
            telemetry.addData("Color", "Mg");
            telemetry.addData("mg", "███╗░░░███╗░██████╗░");
            telemetry.addData("mg", "████╗░████║██╔════╝░");
            telemetry.addData("mg", "██╔████╔██║██║░░██╗░");
            telemetry.addData("mg", "██║╚██╔╝██║██║░░╚██╗");
            telemetry.addData("mg", "██║░╚═╝░██║╚██████╔╝");
            telemetry.addData("mg", "╚═╝░░░░░╚═╝░╚═════╝░");
        }
        if (R.GrI > R.MgI && R.GrI > R.CnI) {
            s = "Gr";
            telemetry.addData("Color", "Gr");
            telemetry.addData("gr", "░██████╗░██████╗░");
            telemetry.addData("gr", "██╔════╝░██╔══██╗");
            telemetry.addData("gr", "██║░░██╗░██████╔╝");
            telemetry.addData("gr", "██║░░╚██╗██╔══██╗");
            telemetry.addData("gr", "╚██████╔╝██║░░██║");
            telemetry.addData("gr", "░╚═════╝░╚═╝░░╚═╝");
        }
        if (R.CnI > R.GrI && R.CnI > R.MgI) {
            s = "Cn";
            telemetry.addData("Color", "Cn");
            telemetry.addData("cn", "░█████╗░███╗░░██╗");
            telemetry.addData("cn", "██╔══██╗████╗░██║");
            telemetry.addData("cn", "██║░░╚═╝██╔██╗██║");
            telemetry.addData("cn", "██║░░██╗██║╚████║");
            telemetry.addData("cn", "╚█████╔╝██║░╚███║");
            telemetry.addData("cn", "░╚════╝░╚═╝░░╚══╝");
        }
        telemetry.addData("Mg count", R.MgI);
        telemetry.addData("Gr count", R.GrI);
        telemetry.addData("Cn count", R.CnI);
        telemetry.addData("SMACHNAYA", "PAPERDELINA");
        telemetry.update();
        R.delay(400);

        R.liftUp();
        R.LT.setPower(0.24);
        R.setMtPower(0.3, 0.3, -0.3, -0.3);
        R.delay(400);
        R.setMtPower(0, 0, 0, 0);
        R.delay(200);
        R.rotateS(42);
        R.delay(1000);
        R.LT.setPower(0.1);
        R.delay(1000);
        R.KL.setPosition(0.7);
        R.delay(400);
        R.LT.setPower(0.6);
        R.delay(440);
        R.LT.setPower(0.2);
        R.rotateS(-42);
        R.delay(200);
        R.LT.setPower(0.05);
        R.delay(500);
        R.LT.setPower(0);
        R.delay(200);
        R.setMtPower(-0.3, -0.3, 0.3, 0.3);
        R.delay(500);
        R.setMtPower(0, 0, 0, 0);
        R.delay(300);

        if (s == "Ns") {
            R.Katet(70, 4);
        }
        if (s == "Mg") {
            R.Katet(80, 1);
            R.rotate(90);
            R.Katet(80, 3);
            R.rotate(-90);
        }
        if (s == "Gr") {
            R.Katet(80, 1);
        }
        if (s == "Cn") {
            R.Katet(80, 1);
            R.rotate(-90);
            R.Katet(80, 3);
            R.rotate(90);
        }
    }

}
