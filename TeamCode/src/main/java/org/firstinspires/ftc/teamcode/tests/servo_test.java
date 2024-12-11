package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class servo_test extends OpMode {

    Servo claw;
    Servo wrist;
    Servo sc1;
    Servo sc2;

    private boolean alternar_Garra;
    private boolean alternar_wrist;
    private boolean alternar_sc1;
    private boolean alternar_sc2;

    ElapsedTime alternar_garraTimer = new ElapsedTime();
    ElapsedTime alternar_wristTimer = new ElapsedTime();
    ElapsedTime alternar_sc1Timer = new ElapsedTime();
    ElapsedTime alternar_sc2Timer = new ElapsedTime();

    double position;

    @Override
    public void init() {

        claw = hardwareMap.servo.get("cw"); // sc1
        wrist = hardwareMap.servo.get("wr"); //cw
        sc1 = hardwareMap.servo.get("sc1"); //wrist
        sc2 = hardwareMap.servo.get("sc2"); //sc2

        position = 0;

    }

    double before = 0;

    @Override
    public void loop() {

        if (gamepad2.a && alternar_Garra && alternar_garraTimer.seconds() > 0.2){
            claw.setPosition(0.3);
            alternar_garraTimer.reset();
        } else if (gamepad2.a && !alternar_Garra  && alternar_garraTimer.seconds() > 0.2){
            claw.setPosition(0.6);
            alternar_garraTimer.reset();
        }

        if (gamepad2.b && alternar_wrist && alternar_garraTimer.seconds() > 0.2){
            wrist.setPosition(0.3);
            alternar_wrist = false;
            alternar_garraTimer.reset();
        } else if (gamepad2.b && !alternar_wrist  && alternar_garraTimer.seconds() > 0.2){
            wrist.setPosition(0.6);
            alternar_wrist = true;
            alternar_garraTimer.reset();
        }

        if (gamepad2.y && alternar_sc1 && alternar_garraTimer.seconds() > 0.2){
            sc1.setPosition(0.3);
            alternar_sc1 = false;
            alternar_garraTimer.reset();
        } else if (gamepad2.y && !alternar_sc1  && alternar_garraTimer.seconds() > 0.2){
            sc1.setPosition(0.6);
            alternar_sc1 = true;
            alternar_garraTimer.reset();
        }

        if (gamepad2.x && alternar_sc2 && alternar_garraTimer.seconds() > 0.2){
            sc2.setPosition(0.3);
            alternar_sc2 = false;
            alternar_garraTimer.reset();
        } else if (gamepad2.x && !alternar_sc2  && alternar_garraTimer.seconds() > 0.2){
            sc2.setPosition(0.6);
            alternar_sc2 = true;
            alternar_garraTimer.reset();
        }

        //cw = sc1
        //wr = cw
        //sc1 = wr
        //sc2 = sc2


        telemetry.addData( "pos", claw.getPosition());

        }

    }

