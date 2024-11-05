package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Comands.Etesito;


@TeleOp
public class color_test extends OpMode {

    Servo servo;

    Etesito etesito = new Etesito();

    @Override
    public void init() {

        etesito.init(hardwareMap);

    }

    @Override
    public void loop() {

        if (gamepad2.dpad_right){

            etesito.wrist.setPosition(0.6);
        } else if (gamepad2.dpad_left){
            etesito.wrist.setPosition(0.5);

        }

        if (gamepad2.left_bumper){
            etesito.claw.setPosition(0.8);


        }else if (gamepad2.right_bumper){

            etesito.claw.setPosition(1);

        }

        telemetry.addData("rojo", etesito.color.red());
        telemetry.addData("azul", etesito.color.blue());
        telemetry.addData("verde", etesito.color.green());

        if (etesito.color.red() > 100) {
            servo.setPosition(0);

        } else if (etesito.color.blue() > 65) {
            servo.setPosition(1);

        } else if (etesito.color.green() > 100) {
            servo.setPosition(1);

        }



        if (etesito.color.red() > 100) {
            etesito.claw.setPosition(1);

        } else if (etesito.color.blue() > 65) {
            etesito.claw.setPosition(1);

        } else if (etesito.color.green() > 100) {
            etesito.claw.setPosition(1);

        }
    }
}
