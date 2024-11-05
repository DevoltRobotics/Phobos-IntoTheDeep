package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@Disabled
@TeleOp
public class tecmicrea extends OpMode {

    DcMotor L;
    DcMotor R;

    @Override
    public void init() {

        L=hardwareMap.dcMotor.get("l");
        R=hardwareMap.dcMotor.get("r");

    }

    @Override
    public void loop() {

        double a = gamepad1.right_stick_x * 0.5;
        double b = gamepad1.left_stick_y * 0.5;

        L.setPower(a - b);
        R.setPower(a + b);

    }
}
