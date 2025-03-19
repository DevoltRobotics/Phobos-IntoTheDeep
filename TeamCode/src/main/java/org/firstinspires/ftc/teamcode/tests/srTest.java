package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class srTest extends OpMode {

    Servo servo;
    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "sr");
    }


    @Override
    public void loop() {
        if (gamepad2.left_bumper) {
            servo.setPosition(0.5);


        } else if (gamepad2.right_bumper) {
            servo.setPosition(0.3);

        }
    }

}
