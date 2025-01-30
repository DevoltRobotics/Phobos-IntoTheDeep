package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Comands.Etesito;

@TeleOp
public class test_color extends OpMode {

    Etesito etesito = new Etesito();

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {

        etesito.init(hardwareMap);
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

    }

    @Override
    public void loop() {

        if(timer.seconds() > 0.2) {
            etesito.colorTelemetry(telemetry);
            timer.reset();
        }

        if (etesito.getColorRed() > etesito.redTarget){
            etesito.wrist_Contract();

        }



        if (gamepad2.left_bumper){
            etesito.dropSample();

        } else if (gamepad2.right_bumper){
            etesito.pickSample();

        }else{
            etesito.intake0();

        }

        if (gamepad2.dpad_right){
            etesito.wrist_down();

        } else if (gamepad2.dpad_left) {
            etesito.wrist_Contract();

        }




    }
}
