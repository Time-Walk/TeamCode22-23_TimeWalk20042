package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TeleOp")
public class teleOpo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot2022 R = new Robot2022();
        R.initFields(telemetry, this, hardwareMap);
        R.init();
        R.gamepad_init(gamepad1,gamepad2);
        //R.KL.setPosition(R.servoPos);
        while ( ! gamepad1.left_bumper && ! gamepad2.right_bumper ) { telemetry.addData("State", "Робот заблокирован"); telemetry.update(); }
        telemetry.addData("ROBOT UNLOCKED!", "!");
        telemetry.update();
        waitForStart();
        R.liftControllerT.start();  //Запуск работы лифта
        while (!isStopRequested()){
            R.wheelbase();   //Передвижение колесной базы
            R.servoController();    //Контроль серво на клешне
        }
        R.LT.setPower(0);

    }
}
