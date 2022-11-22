package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Проверка цветов", group="")
public class CheckColor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2022 R = new Robot2022();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        waitForStart();

        //okay, let's go!

        while (!isStopRequested()) {
            int Red = Color.red(R.getImage().getPixel(500, 250));
            int Green = Color.green(R.getImage().getPixel(500, 250));
            int Blue = Color.blue(R.getImage().getPixel(500, 250));
            R.analyze(Red, Green, Blue);
            telemetry.addData("red", Red);
            telemetry.addData("green", Green);
            telemetry.addData("blue", Blue);
            telemetry.update();
            R.delay(500);
        }
    }

}
