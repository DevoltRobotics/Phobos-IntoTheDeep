package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Comands.Etesito;

@TeleOp
public class Colgación_test extends OpMode {

    Etesito etesito = new Etesito();

    @Override
    public void init() {
        etesito.init(hardwareMap);
    }

    @Override
    public void loop() {

        if (gamepad1.dpad_up) {
            etesito.C1.setPower(1);
            etesito.C2.setPower(-1);

        } else if (gamepad1.dpad_down) {
            etesito.C1.setPower(-1);
            etesito.C2.setPower(1);

        } else {
            etesito.C1.setPower(0);
            etesito.C2.setPower(0);

        }
        if (gamepad1.a) {
            etesito.servoC1.setPosition(0.48);
            etesito.servoC2.setPosition(0.9);

        } else if (gamepad1.b) {
            etesito.servoC1.setPosition(1);
            etesito.servoC2.setPosition(0);

        } else if (gamepad1.y) {
            etesito.servoC1.getController().pwmDisable();
            etesito.servoC2.getController().pwmDisable();

        }
    }
}


