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
    DcMotor arador;
    DcMotor semillero;

    @Override
    public void init() {

        L=hardwareMap.dcMotor.get("l");
        R=hardwareMap.dcMotor.get("r");
        arador =hardwareMap.dcMotor.get("ar");
        semillero = hardwareMap.dcMotor.get("sm");

    }

    @Override
    public void loop() {

        double a = gamepad1.right_stick_x * 0.4;
        double b = gamepad1.left_stick_y * 0.4;

        L.setPower(a + b);
        R.setPower(a - b);

        if (gamepad1.dpad_up){
            arador.setPower(0.6);

        }else if (gamepad1.dpad_down){
            arador.setPower(-0.6);

        }else{
            arador.setPower(0);

        }

        if (gamepad1.a){
            semillero.setPower(0.2);

        }else if (gamepad1.b){
            semillero.setPower(-0.2);

        }else{
            semillero.setPower(0);

        }



    }


}
