package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class servo_test extends OpMode {

    Servo servo;

    double position;

    @Override
    public void init() {

        servo = hardwareMap.servo.get("sc1");

        position = 0;



    }

    double before = 0;

    @Override
    public void loop() {

        double delta = before + position;

        servo.setPosition(delta);

        position = 0;

        if (gamepad2.dpad_down){
            position -= 0.05;

        } else if (gamepad2.dpad_up){
            position += 0.05;

        } else {

            position = 0;

        }

        before = servo.getPosition();


        telemetry.addData( "pos", servo.getPosition());

        }

    }

