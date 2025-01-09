package org.firstinspires.ftc.teamcode.Comands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import kotlin.text.CharDirectionality;

public class Claw {
    Servo claw;
    Etesito etesito = new Etesito();

    public Claw(HardwareMap hardwareMap) {
        claw = hardwareMap.servo.get("cw");
    }



    public class DropSpecimen implements Action {
        Servo claw;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.setPosition(etesito.Pos_open);
            return false;
        }
    }

    public class PickSpecimen implements Action {
        Servo claw;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.setPosition(etesito.Pos_close);
            return false;
        }
    }

    public Action dropSpecimen(){
        return new DropSpecimen();

    }

    public Action pickSpecimen(){
        return new PickSpecimen();

    }
}
