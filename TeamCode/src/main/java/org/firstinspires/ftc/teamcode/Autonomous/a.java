package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class a extends LinearOpMode {

    DcMotor L;
    DcMotor R;
    DcMotor arador;
    DcMotor semillero;

    @Override
    public void runOpMode() throws InterruptedException {

        L = hardwareMap.dcMotor.get("l");
        R = hardwareMap.dcMotor.get("r");
        arador = hardwareMap.dcMotor.get("ar");
        semillero = hardwareMap.dcMotor.get("sm");

        waitForStart();

        enfrente(2000);
        arador.setPower(0.5);
        sleep(800);
        arador.setPower(0);
        sleep(50);
        girar(1400);
        enfrentesemillas(2000);




    }

    public void enfrente(int time){
        L.setPower(-0.3);
        R.setPower(0.3);
        sleep(time);

        L.setPower(0);
        R.setPower(0);
        sleep(50);

    }

    public void enfrentesemillas(int time){
        L.setPower(-0.3);
        R.setPower(0.3);
        semillero.setPower(0.3);
        sleep(time);

        L.setPower(0);
        R.setPower(0);
        semillero.setPower(0);
        sleep(50);

    }

    public void girar(int time){
        L.setPower(0.3);
        R.setPower(0.3);
        sleep(time);

        L.setPower(0);
        R.setPower(0);
        sleep(50);

    }



}


