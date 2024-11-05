package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Comands.Etesito;

@TeleOp
public class Colgación_test extends OpMode {

    Etesito etesito = new Etesito();

    private ElapsedTime timer = new ElapsedTime();
    private boolean servito;

    @Override
    public void init() {
        etesito.init(hardwareMap);

        etesito.servoC1.setPosition(0.5);
        etesito.servoC2.setPosition(0.5);


        timer.reset();
    }

    @Override
    public void loop() {

        etesito.C1.setPower(gamepad2.right_stick_y);

        etesito.C2.setPower(-gamepad2.left_stick_y);

        if (gamepad2.a) {
            etesito.servoC1.setPosition(0.7);
            etesito.servoC2.setPosition(0.3);

        } else if (gamepad2.b) {
            etesito.servoC1.setPosition(0.5);
            etesito.servoC2.setPosition(0.5);

        } else if (gamepad2.y) {
            etesito.servoC1.setPosition(0.3);
            etesito.servoC2.setPosition(0.7);

            //timer.reset();
            //servito = true;

        }

        /*if (timer.seconds() <= 1 && servito){
            etesito.servoC1.getController().pwmDisable();
            etesito.servoC2.getController().pwmDisable();

        }*/
    }
}


