package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="Тест колор драйв", group="")
public class AutoTestColorDrive extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        Robot2022 R = new Robot2022();
        R.initFields(telemetry, this, hardwareMap);
        R.init();

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


        waitForStart();

        //okay, let's go!

        //R.setMtPower(0.25, 0.25, -0.22, 0.22);


        telemetry.addData("Color", "Next String (if ns - empty string)");
        String s = "Ns";
        if (R.MgI > R.GrI && R.MgI > R.CnI) {
            s = "Mg";
            telemetry.addData("Color", "Mg");
        }
        if (R.GrI > R.MgI && R.GrI > R.CnI) {
            s = "Gr";
            telemetry.addData("Color", "Gr");
        }
        if (R.CnI > R.GrI && R.CnI > R.MgI) {
            s = "Cn";
            telemetry.addData("Color", "Cn");
        }
        telemetry.addData("Mg count", R.MgI);
        telemetry.addData("Gr count", R.GrI);
        telemetry.addData("Cn count", R.CnI);
        telemetry.update();
        R.delay(1000);
        //R.RyanGosling();
        R.delay(1500);
        //R.doColor(s);
        R.delay(10000);

    }

}
