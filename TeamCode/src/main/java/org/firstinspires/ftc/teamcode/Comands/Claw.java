package org.firstinspires.ftc.teamcode.Comands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    private Servo claw;

    Etesito etesito = new Etesito();

    public Claw(HardwareMap hardwareMap) {

        claw = hardwareMap.get(Servo.class, "cw");
    }

    public class Pick implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            claw.setPosition(etesito.Pos_close);
            return false;
        }
    }

    public Action pick(){
        return new Pick();

    }

    public class Drop implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            claw.setPosition(etesito.Pos_open);
            return false;
        }
    }

    public Action drop(){
        return new Drop();

    }


}
