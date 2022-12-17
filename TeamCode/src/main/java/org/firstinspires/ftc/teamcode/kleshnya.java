package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="ПО ПРИКАЗУ ГЕНЕРАЛА ГАВСА", group="")
//@Disable
public class kleshnya extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2022 R = new Robot2022();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        waitForStart();

        //okay, let's go!
        R.KL.setPosition(0.7);
        R.delay(4500);
        R.KL.setPosition(0.4);
        R.delay(1000);

    }

}
