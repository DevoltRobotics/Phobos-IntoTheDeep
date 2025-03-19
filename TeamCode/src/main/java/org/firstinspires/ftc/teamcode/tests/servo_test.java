package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Comands.Etesito;


@TeleOp
public class servo_test extends OpMode {

    Etesito etesito = new Etesito();

    private boolean alternar_Garra;
    private boolean alternar_wrist;
    private boolean alternar_sc1;
    private boolean alternar_sc2;

    ElapsedTime alternar_garraTimer = new ElapsedTime();
    ElapsedTime alternar_wristTimer = new ElapsedTime();
    ElapsedTime alternar_sc1Timer = new ElapsedTime();
    ElapsedTime alternar_sc2Timer = new ElapsedTime();

    @Override
    public void init() {

        etesito.init(hardwareMap, false, false);

    }


    @Override
    public void loop() {

        telemetry.addLine("a ---- cw");
        telemetry.addLine("b ---- wr");
        telemetry.addLine("y ---- sc1");
        telemetry.addLine("x ---- sc2");
        telemetry.addLine("right_stick ---- in");


        if (alternar_garraTimer.seconds() > 0.2 && gamepad2.a){
            if (alternar_Garra){
                etesito.claw.setPosition(0.4);
                alternar_Garra = false;

            }else {
                etesito.claw.setPosition(0.5);
                alternar_Garra = true;

        }

            alternar_garraTimer.reset();
        }

        if (alternar_wristTimer.seconds() > 0.2 && gamepad2.b){
            if (alternar_wrist){
                etesito.wrist.setPosition(0.4);
                alternar_Garra = false;

            }else {
                etesito.wrist.setPosition(0.5);
                alternar_wrist = true;

            }

            alternar_wristTimer.reset();
        }

        if (alternar_sc1Timer.seconds() > 0.2 && gamepad2.y){
            if (alternar_sc1){
                etesito.sC1.setPosition(0.4);
                alternar_sc1 = false;

            }else {
                etesito.sC1.setPosition(0.5);
                alternar_sc1 = true;

            }

            alternar_sc1Timer.reset();
        }

        if (alternar_sc2Timer.seconds() > 0.2 && gamepad2.x){
            if (alternar_sc2){
                etesito.sC2.setPosition(0.4);
                alternar_sc2 = false;

            }else {
                etesito.sC2.setPosition(0.5);
                alternar_sc2 = true;

            }

            alternar_sc2Timer.reset();
        }

        etesito.intake.setPower(gamepad2.right_stick_y);
        }

    }

