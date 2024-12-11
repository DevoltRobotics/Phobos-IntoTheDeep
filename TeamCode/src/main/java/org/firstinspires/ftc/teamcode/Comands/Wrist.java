package org.firstinspires.ftc.teamcode.Comands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {

    private Servo wrist;

    Etesito etesito = new Etesito();

    public Wrist(HardwareMap hardwareMap) {

        wrist = hardwareMap.get(Servo.class, "wr");
    }

    public class WristUp implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(etesito.Up_wrist);
            return false;
        }
    }

    public Action wristUp(){
        return new WristUp();
    }

    public class WristSpecimen implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(etesito.Specimen_wristAutonomous);
            return false;
        }
    }

    public Action wristSpecimen(){
        return new WristSpecimen();
    }

    public class WristMedium implements Action  {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            wrist.setPosition(etesito.Medium_wrist);
            return false;
        }
    }

    public Action wristMedium(){
        return new WristMedium();

    }

    public class WristDown implements Action  {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            wrist.setPosition(etesito.Down_wrist);
            return false;
        }
    }

    public Action wristDown(){
        return new WristDown();

    }

    public class WristDownM implements Action  {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            wrist.setPosition(etesito.Down_M_wrist);
            return false;
        }
    }

    public Action wristDownM(){
        return new WristDownM();

    }

    public class WristInit implements Action  {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            wrist.setPosition(etesito.Init_wrist);
            return false;
        }
    }

    public Action wristInit(){
        return new WristInit();

    }

    public class WristDisabled implements Action  {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            wrist.getController().pwmDisable();
            return false;
        }
    }

    public Action wristDisabled(){
        return new WristDisabled();


    }


}
