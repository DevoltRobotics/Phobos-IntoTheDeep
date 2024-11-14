package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
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




        position = 0;

        if (gamepad2.dpad_down){
            position -= 0.05;

        } else if (gamepad2.dpad_up){
            position += 0.05;

        } else if (gamepad2.dpad_right){

            position = 0;

        }

        double delta = before + position;

        servo.setPosition(delta);

        before = servo.getPosition();

        telemetry.addData( "pos", servo.getPosition());

        }

    }

