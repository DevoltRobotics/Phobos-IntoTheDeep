package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class lucesitaTest extends OpMode {
    Servo light;
    @Override
    public void init() {
        light = hardwareMap.get(Servo.class, "rgb");


    }

    @Override
    public void loop() {
        light.setPosition(1);





    }
}
