package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.Comands.Constants.contractAbramPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.midOpenAbramPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.openAbramPos;

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
        if (gamepad2.dpad_right) {
            servo.setPosition(openAbramPos);


        } else if (gamepad2.dpad_up) {
            servo.setPosition(midOpenAbramPos);

        }else if (gamepad2.dpad_left) {
            servo.setPosition(contractAbramPos);

        }
    }

}
