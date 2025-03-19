package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Comands.Etesito;

@TeleOp
public class palitosTest extends OpMode {

    Servo palitoLeft;
    Servo palitoRight;

    Etesito etesito = new Etesito();

    @Override
    public void init() {
        palitoLeft = hardwareMap.get(Servo.class, "plL");
        palitoRight = hardwareMap.get(Servo.class, "plR");

        etesito.init(hardwareMap, false, false);
    }


    @Override
    public void loop() {
        if (gamepad2.left_bumper) {
            palitoLeft.setPosition(0.5);
            palitoRight.setPosition(0.5);

        } else if (gamepad2.right_bumper) {
            palitoLeft.setPosition(0.63);
            palitoRight.setPosition(0.37);

        }

        if (gamepad2.dpad_right){
            etesito.servosUping();

        } else if (gamepad2.dpad_left) {
            etesito.servos_test();

        }


    }

}
