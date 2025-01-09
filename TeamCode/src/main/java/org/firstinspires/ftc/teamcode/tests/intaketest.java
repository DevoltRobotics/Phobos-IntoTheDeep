package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class intaketest extends OpMode {
    CRServo intake;
    @Override
    public void init() {
        intake = hardwareMap.get(CRServo.class, "in");

    }

    @Override
    public void loop() {
        if (gamepad2.right_bumper){
            intake.setPower(-1);
        }else if (gamepad2.left_bumper){
            intake.setPower(1);

        }else {
            intake.setPower(0);

        }



    }
}
