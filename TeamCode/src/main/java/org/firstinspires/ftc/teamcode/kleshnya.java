package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="Закрыть-Открыть клешню", group="")
//@Disable
public class kleshnya extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2022 R = new Robot2022();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        waitForStart();

        //okay, let's go!
        //R.S1.setPosition(0.75);
        //R.S2.setPosition(0.05);
        R.delay(1000);
        //R.S1.setPosition(0.3);
        //R.S2.setPosition(0.4);
        R.delay(1000);

    }

}
