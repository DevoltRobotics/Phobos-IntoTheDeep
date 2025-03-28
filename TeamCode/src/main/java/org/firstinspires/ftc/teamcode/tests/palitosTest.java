package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Comands.Etesito;

@TeleOp
public class palitosTest extends OpMode {

    Etesito etesito = new Etesito();

    @Override
    public void init() {
        etesito.init(hardwareMap, false, false);

        etesito.launcherLeft.setPosition(0.5);
        etesito.launcherRight.setPosition(0.5);


    }


    @Override
    public void loop() {
        if (gamepad2.left_bumper) {
            etesito.supportHangArms();

        } else if (gamepad2.right_bumper) {
            etesito.launchHangArms();

        }

        if (gamepad2.dpad_right){
            etesito.servosHanging();

        } else if (gamepad2.dpad_left) {
            etesito.servos_test();

        }


    }

}
