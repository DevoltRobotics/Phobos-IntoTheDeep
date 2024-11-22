package org.firstinspires.ftc.teamcode.Comands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class Arm {

    private DcMotorEx arm;
    private DcMotorEx rode;

    private int ratio = 8;

    Etesito etesito = new Etesito();

    private int beforeArmPos = 0;

    public static PIDFController.PIDCoefficients armCoefficients = new PIDFController.PIDCoefficients(0.0015, 0, 0.0017);
    PIDFController armcontroller = new PIDFController(armCoefficients);

    public static PIDFController.PIDCoefficients rodeCoefficients = new PIDFController.PIDCoefficients(0.015, 0, 0.007);
    PIDFController rodecontroller = new PIDFController(rodeCoefficients);

    public Arm(HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotorEx.class, "liftMotor");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorEx.Direction.FORWARD);

        rode = hardwareMap.get(DcMotorEx.class, "liftMotor");
        rode.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rode.setDirection(DcMotorEx.Direction.FORWARD);

        armcontroller.reset();
        rodecontroller.reset();

    }


    public class Arm_up implements Action {

        private double powerArm;
        // checks if the lift motor has been powered on

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            int armPos = arm.getCurrentPosition();

            arm.setPower(-armcontroller.update(arm.getCurrentPosition()) * powerArm);
            rode.setPower(rodecontroller.update(rode.getCurrentPosition()) * 0.09);

            powerArm = 0.4;
            armcontroller.targetPosition = etesito.high_Armpos;

            // checks lift's current position

            double deltaArmPos = arm.getCurrentPosition() - beforeArmPos;

            rodecontroller.targetPosition += (deltaArmPos / ratio);

            beforeArmPos = arm.getCurrentPosition();

            packet.put("armPos", armPos);
            return false;

        }
    }

    public class Arm_mediumHigh implements Action {

        private double powerArm;
        // checks if the lift motor has been powered on

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            int armPos = arm.getCurrentPosition();

            arm.setPower(-armcontroller.update(arm.getCurrentPosition()) * powerArm);
            rode.setPower(rodecontroller.update(rode.getCurrentPosition()) * 0.09);

            powerArm = 0.4;
            armcontroller.targetPosition = etesito.mediumHigh_Armpos;

            // checks lift's current position

            double deltaArmPos = arm.getCurrentPosition() - beforeArmPos;

            rodecontroller.targetPosition += (deltaArmPos / ratio);

            beforeArmPos = arm.getCurrentPosition();

            packet.put("armPos", armPos);
            return false;

        }
    }

    public class Arm_mediumLow implements Action {

        private double powerArm;
        // checks if the lift motor has been powered on

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            int armPos = arm.getCurrentPosition();

            arm.setPower(-armcontroller.update(arm.getCurrentPosition()) * powerArm);
            rode.setPower(rodecontroller.update(rode.getCurrentPosition()) * 0.09);

            powerArm = 0.4;
            armcontroller.targetPosition = etesito.mediumLow_Armpos;

            // checks lift's current position

            double deltaArmPos = arm.getCurrentPosition() - beforeArmPos;

            rodecontroller.targetPosition += (deltaArmPos / ratio);

            beforeArmPos = arm.getCurrentPosition();

            packet.put("armPos", armPos);
            return false;

        }
    }

    public class Arm_down implements Action {

        private double powerArm;
        // checks if the lift motor has been powered on

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            int armPos = arm.getCurrentPosition();

            arm.setPower(-armcontroller.update(arm.getCurrentPosition()) * powerArm);
            rode.setPower(rodecontroller.update(rode.getCurrentPosition()) * 0.09);

            powerArm = 0.4;
            armcontroller.targetPosition = etesito.down_ArmPos;

            // checks lift's current position

            double deltaArmPos = arm.getCurrentPosition() - beforeArmPos;

            rodecontroller.targetPosition += (deltaArmPos / ratio);

            beforeArmPos = arm.getCurrentPosition();

            packet.put("armPos", armPos);
            return false;

        }

        public class Arm_mediumHigh implements Action {

            private double powerArm;
            // checks if the lift motor has been powered on

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                int armPos = arm.getCurrentPosition();

                arm.setPower(-armcontroller.update(arm.getCurrentPosition()) * powerArm);
                rode.setPower(rodecontroller.update(rode.getCurrentPosition()) * 0.09);

                powerArm = 0.4;
                armcontroller.targetPosition = etesito.mediumHigh_Armpos;

                // checks lift's current position

                double deltaArmPos = arm.getCurrentPosition() - beforeArmPos;

                rodecontroller.targetPosition += (deltaArmPos / ratio);

                beforeArmPos = arm.getCurrentPosition();

                packet.put("armPos", armPos);
                return false;

            }
        }


    }

    public class Rode_up implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            int rodePos = rode.getCurrentPosition();

            rode.setPower(rodecontroller.update(rode.getCurrentPosition()) * 0.09);

            rodecontroller.targetPosition = etesito.rode_High;

            packet.put("rodePos", rodePos);
            return false;

        }
    }

    public class Rode_medium implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            int rodePos = rode.getCurrentPosition();

            rode.setPower(rodecontroller.update(rode.getCurrentPosition()) * 0.09);

            rodecontroller.targetPosition = etesito.rode_medium;

            packet.put("rodePos", rodePos);
            return false;

        }
    }

    public class Rode_down implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            int rodePos = rode.getCurrentPosition();

            rode.setPower(rodecontroller.update(rode.getCurrentPosition()) * 0.09);

            rodecontroller.targetPosition = etesito.rode_down;

            packet.put("rodePos", rodePos);
            return false;

        }
    }


}




