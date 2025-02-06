package org.firstinspires.ftc.teamcode.tests;

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

        arm = hardwareMap.get(DcMotorEx.class, "arm");

    }


    @Override
    public void loop() {

        arm.setPower(-gamepad2.left_stick_y * 0.5);


    }
}
