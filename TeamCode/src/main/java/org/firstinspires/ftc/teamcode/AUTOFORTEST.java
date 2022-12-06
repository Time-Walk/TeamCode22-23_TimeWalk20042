package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="Автоном для теста", group="")
//@Disable
public class AUTOFORTEST extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2022 R = new Robot2022();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        waitForStart();

        //okay, let's go!
        R.Katet(50, 1);
        R.Katet(100, 3);
        R.Katet(50, 1);
        R.Katet(50, 2);
        R.Katet(100, 4);
        R.Katet(50, 2);

    }

}
