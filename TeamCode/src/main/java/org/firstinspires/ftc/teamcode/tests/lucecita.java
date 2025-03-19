package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Comands.Etesito;

@TeleOp
public class lucecita extends OpMode {

    Etesito etesito = new Etesito();

    @Override
    public void init() {
        etesito.init(hardwareMap, false, false);
    }

    @Override
    public void loop() {
        etesito.setLight("orange");
    }

}
