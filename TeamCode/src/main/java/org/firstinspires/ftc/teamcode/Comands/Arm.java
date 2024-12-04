package org.firstinspires.ftc.teamcode.Comands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Arm {

    private DcMotorEx armMotor;
    private DcMotorEx rode;

    private int ratio = 8;

    Etesito etesito = new Etesito();

    private double  powerArm;
    private double  powerRode;

    public static PIDFController.PIDCoefficients armCoefficients = new PIDFController.PIDCoefficients(0.0015, 0, 0.0017);
    PIDFController armcontroller = new PIDFController(armCoefficients);

    public static PIDFController.PIDCoefficients rodeCoefficients = new PIDFController.PIDCoefficients(0.015, 0, 0.007);
    PIDFController rodecontroller = new PIDFController(rodeCoefficients);

    ElapsedTime timer = new ElapsedTime();

    public Arm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rode = hardwareMap.get(DcMotorEx.class, "rd");
        rode.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rode.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armcontroller.reset();
        rodecontroller.reset();
    }

    public void updating() {
        armMotor.setPower(-armcontroller.update(armMotor.getCurrentPosition()) * powerArm);
        rode.setPower(rodecontroller.update(rode.getCurrentPosition() * powerRode));

    }

    public class ArmSpecimen implements Action {
        // checks if the lift motor has been powered on

        boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            if (!initialized) {
                armcontroller.targetPosition = etesito.specimen_ArmPos + 150;
                powerArm = 0.4;
                initialized = true;

            }

            double armPos = armMotor.getCurrentPosition();

            armMotor.setPower(-armcontroller.update(armMotor.getCurrentPosition()) * powerArm);


            if (armPos > etesito.specimen_ArmPos) {
                return true;
            } else {
                armcontroller.targetPosition = etesito.specimen_ArmPos;
                powerArm = 0.4;
                return false;
            }
        }
    }

    public Action armSpecimen() {
        return new ArmSpecimen();
    }

    public class ArmInit implements Action {

        // checks if the lift motor has been powered on}
        boolean initialized = false;


        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armcontroller.targetPosition = -1200;
                powerArm = 0.4;
                initialized = true;

            }

            armMotor.setPower(-armcontroller.update(armMotor.getCurrentPosition()) * powerArm);

            double armPos = armMotor.getCurrentPosition();

            packet.put("armPos",armPos);
            packet.put("armPower", armMotor.getPower());

            if (armPos > -900){
                return true;

            } else {
                armcontroller.targetPosition = -900;
                powerArm = 0.4;
                return false;

            }

        }
    }

    public Action armInit() {
        return new ArmInit();
    }

    public class ArmDown implements Action {
        // checks if the lift motor has been powered on

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            powerArm = 0.1;
            armcontroller.targetPosition = 0;

            int armPos = armMotor.getCurrentPosition();

            // checks lift's current position

            if (armPos < -150) {
                return true;
            } else {
                armcontroller.targetPosition = 0;
                powerArm = 0.1;
                return false;
            }

        }

    }

    public Action armDown() {
        return new ArmDown();
    }

    public class RodeSpecimen implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            powerRode = 0.09;
            rodecontroller.targetPosition = etesito.rode_specimen;

            double rodePos = rode.getCurrentPosition();

            if (rodePos > etesito.rode_specimen + 150) {
                return true;

            } else {
                rodecontroller.targetPosition = etesito.rode_specimen + 150;
                powerRode = 0.09;
                return false;
            }
        }
    }

    public Action rodeSpecimen() {
        return new RodeSpecimen();

    }

    public class RodeDown implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            powerRode = 0.09;
            rodecontroller.targetPosition = 50;

            double rodePos = rode.getCurrentPosition();

            if (rodePos < 0) {
                return true;

            } else {
                powerRode = 0.09;
                rodecontroller.targetPosition = 0;

                return false;

            }

        }

    }

    public Action rodeDown() {
        return new RodeDown();

    }

    public class ArmInPos implements Action {

        // checks if the lift motor has been powered on}
        boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            armMotor.setPower(-armcontroller.update(armMotor.getCurrentPosition()) * powerArm);
            rode.setPower(rodecontroller.update(rode.getCurrentPosition() * powerRode));

            double armPos = armMotor.getCurrentPosition();

            if (armPos < 500){
                return true;

            } else {
                return false;

            }

        }
    }

    public Action armInPos() {
        return new ArmInPos();
    }


}




