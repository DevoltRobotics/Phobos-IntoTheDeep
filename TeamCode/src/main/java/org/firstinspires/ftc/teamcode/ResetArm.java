package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Comands.Etesito;

public class ResetArm extends OpMode {

    Etesito etesito = new Etesito();

    @Override
    public void init() {
        etesito.init(hardwareMap, true, true);

        etesito.resetArmEncoder();
        etesito.resetRodeEncoder();

    }

    @Override
    public void loop() {

    }
}
