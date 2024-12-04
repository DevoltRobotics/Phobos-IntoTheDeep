package org.firstinspires.ftc.teamcode.Comands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClimbServos {

    private Servo sc1;
    private Servo sc2;

    Etesito etesito = new Etesito();

    public ClimbServos(HardwareMap hardwareMap) {

        sc1 = hardwareMap.get(Servo.class, "sc1");
        sc2 = hardwareMap.get(Servo.class, "sc2");
    }

    public class ServosUp implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            sc1.setPosition(0.7);
            sc2.setPosition(0.3);

            return false;
        }
    }

    public Action servosUp(){
        return new ServosUp();

    }

    public class ServosInit implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            sc1.setPosition(0.5);
            sc2.setPosition(0.5);
            return false;
        }
    }

    public Action servosInit(){
        return new ServosInit();

    }


}
