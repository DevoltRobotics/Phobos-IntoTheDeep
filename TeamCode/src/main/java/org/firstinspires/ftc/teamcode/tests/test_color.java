package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Comands.Etesito;

public class test_color extends OpMode {

    Etesito etesito = new Etesito();

    private double hsv[] = {0.0f, 0.0f, 0.0f};

    private double rojo, verde, azul;

    private double scale_factor = 255;



    @Override
    public void init() {

        etesito.init(hardwareMap);

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

    }

    @Override
    public void loop() {

        etesito.color.enableLed(true);

        rojo = etesito.color.red();
        verde = etesito.color.green();
        azul = etesito.color.green();

        telemetry.addData("red", rojo);
        telemetry.addData("blue", azul);
        telemetry.addData("green", verde);


    }
}
