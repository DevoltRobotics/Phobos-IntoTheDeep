package org.firstinspires.ftc.teamcode.Comands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ChassisSubsystem {

    public DcMotorEx FL, BL, BR, FR;

    private int ArmTarget;
    private int RodeTarget;
    private double powerArm;

    private int ratio = 8;

    Etesito etesito = new Etesito();

    ElapsedTime timer = new ElapsedTime();

    public ChassisSubsystem(HardwareMap hardwareMap) {
        FL = hardwareMap.get(DcMotorEx.class, "fl");
        BL = hardwareMap.get(DcMotorEx.class, "bl");
        BR = hardwareMap.get(DcMotorEx.class, "br");
        FR = hardwareMap.get(DcMotorEx.class, "fr");

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);

    }


    /*class MoveChassis implements Action {
        double power;
        ElapsedTime timer;
        double timeTarget;

        public MoveChassis(double power, double timeTarget) {
            this.power = power;
            this.timeTarget = timeTarget;

        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null){
                timer = new ElapsedTime();
            }

            //

            FL.setPower(power);
            BL.setPower(power);
            FR.setPower(power);
            BR.setPower(power);

            return timer.seconds() <= timeTarget;
        }
    }

    public Action moveChassis() {
        return new MoveChassis(0.4,0.2);
    }
    */


}