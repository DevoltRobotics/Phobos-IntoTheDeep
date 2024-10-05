package org.firstinspires.ftc.teamcode.Comands;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class test_motor extends OpMode {

    DcMotorEx arm;


    @Override
    public void init() {

        arm = hardwareMap.get(DcMotorEx.class, "arm");

    }

    @Override
    public void loop() {

        arm.setPower(gamepad2.right_stick_y);

    }
}
