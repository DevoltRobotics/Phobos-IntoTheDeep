package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Comands.Etesito;

@TeleOp
public class clawtest extends OpMode {

    Etesito etesito = new Etesito();

    @Override
    public void init() {
        etesito.init(hardwareMap);
    }


    @Override
    public void loop() {
        if (gamepad2.left_bumper) {
            etesito.dropSpecimen();


        } else if (gamepad2.right_bumper) {
            etesito.pickSpecimen();

        }
    }

}
