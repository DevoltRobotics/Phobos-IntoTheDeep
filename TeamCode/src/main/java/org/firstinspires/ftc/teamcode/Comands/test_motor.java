package org.firstinspires.ftc.teamcode.Comands;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class test_motor extends OpMode {

    DcMotorEx arm;

    Servo servo;


    @Override
    public void init() {

        arm = hardwareMap.get(DcMotorEx.class, "mc1");

        servo = hardwareMap.get(Servo.class, "wr");

    }

    @Override
    public void loop() {

        arm.setPower(gamepad2.right_stick_y);

        if (gamepad2.a){

            servo.setPosition(0.6);
        } else if (gamepad2.b){

            servo.setPosition(0.5);
        }

    }
}
