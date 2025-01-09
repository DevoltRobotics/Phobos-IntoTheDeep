package org.firstinspires.ftc.teamcode.Comands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;


import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


public class ArmSubsystem {

    private DcMotorEx armMotor;
    private DcMotorEx rodeMotor;

    private int ArmTarget;
    private int RodeTarget;
    private double powerArm;

    private int ratio = 8;

    Etesito etesito = new Etesito();

    PIDFController armController = new PIDFController(etesito.armCoefficients);
    PIDFController rodeController = new PIDFController(etesito.rodeCoefficients);

    ElapsedTime timer = new ElapsedTime();

    public ArmSubsystem(HardwareMap hardwareMap) {
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

    class ArmToPos implements Action {
        int ticks;

        public ArmToPos(int ticks) {
            this.ticks = ticks;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            ArmTarget = ticks;
            return false;
        }

    }

    class ArmToPosSmooth implements Action {
        int ticks;
        double timeSeconds;

        int currentTicks;

        ElapsedTime timer = null;

        public ArmToPosSmooth(int ticks, double timeSeconds) {
            this.ticks = ticks;
            this.timeSeconds = timeSeconds;
        }

        private double lerp(double start, double end, double t) {
            return start * (1 - t) + end * t;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(timer == null) {
                timer = new ElapsedTime();
                currentTicks = armMotor.getCurrentPosition();
            }

            double t = Range.clip(timer.seconds() / timeSeconds, 0, 1);

            ArmTarget = (int) lerp(currentTicks, ticks, t);

            return timer.seconds() <= timeSeconds;
        }
    }

    class RodeToPos implements Action {
        int ticks;

        public RodeToPos(int ticks) {
            this.ticks = ticks;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            RodeTarget = ticks;
            return false;
        }

    }

    public void ArmUpdateVoid() {
        armController.targetPosition = ArmTarget;
        armMotor.setPower(-armController.update(armMotor.getCurrentPosition()) * 0.4);

        rodeController.targetPosition = RodeTarget;
        rodeMotor.setPower(rodeController.update(rodeMotor.getCurrentPosition()) * 0.09);
    }

    class ArmUpdate implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            armController.targetPosition = ArmTarget;
            armMotor.setPower(-armController.update(armMotor.getCurrentPosition()) * 0.4);

            rodeController.targetPosition = RodeTarget;
            rodeMotor.setPower(rodeController.update(rodeMotor.getCurrentPosition()) * 0.09);

            return true;
        }

    }

    class RodeReset implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            rodeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rodeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            return false;
        }

    }

    class SamplePickColor implements Action {

        int Color;
        double Time;
        ElapsedTime timer;

        public SamplePickColor(int color, double time) {
            this.Color = color;
            this.Time = time;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
                etesito.pickSample();
                rodeMotor.setPower(0.05);

            }

            return (Color < 350 || etesito.getColorGreen() < 600) || timer.seconds() < Time;
        }

    }

    public Action armSpecimen1() {
        return new ParallelAction(
                new ArmToPos(etesito.specimenArmPos - 80),
                new RodeToPos(etesito.specimenDownRodePos - 80)

        );
    }

    public Action armSpecimen2() {
        return new ParallelAction(
                new ArmToPos(etesito.specimenArmPos - 80),
                new RodeToPos(etesito.specimenDownRodePos - 320)

        );
    }

    public Action armSpecimen3() {
        return new ParallelAction(
                new ArmToPos(etesito.specimenArmPos - 80),
                new RodeToPos(etesito.specimenDownRodePos - 440)

        );
    }

    public Action armUp() {
        return new SequentialAction(
                new ParallelAction(
                        new ArmToPos(etesito.highBasketArmpos - 50),
                        new RodeToPos(-150)
                ),
                new SleepAction(0.5),
                new RodeReset()

        );
    }

    public Action armHighBasketFront() {
        return new SequentialAction(
                new ParallelAction(
                        new ArmToPos(etesito.highBasketfrontArmpos),
                        new RodeToPos(-130)
                ),
                new SleepAction(0.5),
                new RodeReset()

        );
    }

    public Action armInit() {
        return new SequentialAction(
                new ParallelAction(
                        new ArmToPos(etesito.initArmpos),
                        new RodeToPos(-75)
                ),
                new SleepAction(0.2),
                new RodeReset()

        );
    }

    public Action armDown() {
        return new SequentialAction(
                new ParallelAction(
                        new ArmToPosSmooth(0, 1),
                        new RodeToPos(150)
                ),
                new SleepAction(0.2),
                new RodeReset()
        );
    }

    public Action armDown2() {
        return new SequentialAction(
                new ParallelAction(
                        new ArmToPosSmooth(0, 1),
                        new RodeToPos(0)
                ),
                new SleepAction(0.2),
                new RodeReset()
        );
    }

    public Action armDownLast() {
        return new SequentialAction(
                new ParallelAction(
                        new ArmToPosSmooth(0, 1),
                        new RodeToPos(-250)
                ),
                new SleepAction(0.2),
                new RodeReset()
        );
    }

    public Action rodeDown() {
        return new RodeToPos(-150);

    }

    public Action rodeDown2() {
        return new RodeToPos(100);

    }



    public Action rodePickSampleSample1() {
        return new RodeToPos(-900);

    }

    public Action rodePickSampleSample2() {
        return new RodeToPos(-950);

    }

    public Action rodePickSample1() {
        return new RodeToPos(-1100);

    }

    public Action rodePutSample1() {
        return new RodeToPos(-1300);

    }

    public Action rodePickSample2() {
        return new RodeToPos(-950);

    }

    public Action rodePutSample2() {
        return new RodeToPos(-950);

    }

    public Action rodePickSample3() {
        return new RodeToPos(-1000);

    }

    public Action rodePutSample3() {
        return new RodeToPos(-950);

    }

    public Action rodeHighBasket() {
        return new RodeToPos(etesito.highRodePos + 100);

    }

    public Action rodeSpecimenPrevious() {
        return new RodeToPos(etesito.specimenDownRodePos);

    }

    public Action rodeSpecimen() {
        return new RodeToPos(etesito.specimenRodePos - 130);

    }

    public Action armUpdate() {
        return new ArmUpdate();

    }

    public Action samplePickRedSpecimen() {
        return new SamplePickColor(etesito.getColorRed(), 0.5);

    }

    public Action samplePickRedSubmersible() {
        return new SamplePickColor(etesito.getColorRed(), 2);

    }

    public Action samplePickBlue() {
        return new SamplePickColor(etesito.getColorBlue(), 2);

    }

}






