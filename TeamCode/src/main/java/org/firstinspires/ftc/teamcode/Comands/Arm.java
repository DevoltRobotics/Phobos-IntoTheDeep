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
    private DcMotorEx rodeMotor;

    private int ratio = 8;

    Etesito etesito = new Etesito();

    public static PIDFController.PIDCoefficients armCoefficients = new PIDFController.PIDCoefficients(0.0015, 0, 0.0017);
    PIDFController armController = new PIDFController(armCoefficients);

    public static PIDFController.PIDCoefficients rodeCoefficients = new PIDFController.PIDCoefficients(0.015, 0, 0.007);
    PIDFController rodeController = new PIDFController(rodeCoefficients);

    ElapsedTime timer = new ElapsedTime();

    public Arm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rodeMotor = hardwareMap.get(DcMotorEx.class, "rd");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rodeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rodeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armController.reset();
        rodeController.reset();
    }

    public void updateArm() {
        armMotor.setPower(-armController.update(armMotor.getCurrentPosition()) * 0.4);

    }

    public class ArmUp implements Action {
        // checks if the lift motor has been powered on

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            armController.targetPosition = etesito.highArmpos - 50;
            return false;
        }
    }

    public Action armUp() {
        return new ArmUp();
    }

    public class ArmSpecimen implements Action {
        // checks if the lift motor has been powered on

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            armController.targetPosition = etesito.specimenArmPos;
            return false;
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

            armController.targetPosition = etesito.initArmpos;
            return false;

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

            armController.targetPosition = 100;
            armMotor.setPower(armController.update(armMotor.getCurrentPosition()) * 0.05);

            if (armMotor.getCurrentPosition() > -100){
                return true;
            } else {
                armController.targetPosition = 0;
                return false;

            }

        }

    }

    public Action armDown() {
        return new ArmDown();
    }

    public class RodeUp implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            rodeController.targetPosition = etesito.highRodePos - 50;
            rodeMotor.setPower(rodeController.update(rodeMotor.getCurrentPosition()) * 0.09);

            if (rodeController.getPositionError(rodeMotor.getCurrentPosition()) > -100){
                return true;
            } else {
                rodeController.targetPosition = etesito.highRodePos;
                return false;

            }

        }
    }

    public Action rodeUp() {
        return new RodeUp();

    }

    public class RodeSpecimen implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            rodeController.targetPosition = etesito.specimenRodePos - 120;
            rodeMotor.setPower(rodeController.update(rodeMotor.getCurrentPosition()) * 0.09);

            if (rodeController.getPositionError(rodeMotor.getCurrentPosition()) > -100){
                return true;
            } else {
                rodeController.targetPosition = etesito.specimenRodePos - 50;
                return false;

            }

        }
    }

    public Action rodeSpecimen() {
        return new RodeSpecimen();

    }

    public class RodeLow implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            rodeController.targetPosition = -500;
            rodeMotor.setPower(rodeController.update(rodeMotor.getCurrentPosition()) * 0.09);

            if (rodeController.getPositionError(rodeMotor.getCurrentPosition()) > -100){
                return true;
            } else {
                rodeController.targetPosition = -450;
                return false;

            }

        }
    }

    public Action rodeLow() {
        return new RodeLow();

    }

    public class RodeDown implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            rodeController.targetPosition = 100;
            rodeMotor.setPower(rodeController.update(rodeMotor.getCurrentPosition()) * 0.09);

            if (rodeMotor.getCurrentPosition() > -100){
                return true;
            } else {
                rodeController.targetPosition = 0;
                return false;

            }


            }

        }

        public Action rodeDown() {
            return new RodeDown();

        }

    public class ArmInPos implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            rodeMotor.setPower(rodeController.update(rodeMotor.getCurrentPosition()) * 0.09);
            armMotor.setPower(-armController.update(armMotor.getCurrentPosition()) * 0.4);

            return true;


        }

    }

    public Action armInPos() {
        return new ArmInPos();

    }




    }






