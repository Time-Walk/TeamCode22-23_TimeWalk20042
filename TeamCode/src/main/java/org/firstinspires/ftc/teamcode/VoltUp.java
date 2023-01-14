package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="Тест подъема лифта с вольтметром", group="")
public class VoltUp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2022 R = new Robot2022();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        waitForStart();

        //okay, let's go!
        R.telemetry.addData("Voltage", R.vs.getVoltage());
        double V = R.vs.getVoltage();

        R.LT.setPower(0.7);
        R.delay(Math.round(200*(1 / V)));
        R.LT.setPower(0.4);
        R.delay(Math.round(350*(1 / V))); //Не финальная версия
        R.LT.setPower(0);


    }

}
