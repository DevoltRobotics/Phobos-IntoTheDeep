package org.firstinspires.ftc.teamcode.Comands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;


import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


public class ArmSubsystem {

    private DcMotorEx armMotor;
    private DcMotorEx rodeMotor;

    private CRServo intake;

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

        intake = hardwareMap.get(CRServo.class, "in");

        etesito.init(hardwareMap);

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

        public ArmToPosSmooth(int ticksPos, double timeSeconds) {
            this.ticks = ticksPos;
            this.timeSeconds = timeSeconds;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(timer == null) {
                timer = new ElapsedTime();
                currentTicks = armMotor.getCurrentPosition();
            }

            double t = Range.clip(timer.seconds() / timeSeconds, 0, 1);

            ArmTarget = (int) Etesito.lerp(currentTicks, ticks, t);

            return timer.seconds() <= timeSeconds;
        }
    }

    class RodeToPosSmooth implements Action {
        int ticks;
        double timeSeconds;

        int currentTicks;

        ElapsedTime timer = null;

        public RodeToPosSmooth(int ticksPos, double timeSeconds) {
            this.ticks = ticksPos;
            this.timeSeconds = timeSeconds;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(timer == null) {
                timer = new ElapsedTime();
                currentTicks = rodeMotor.getCurrentPosition();
            }

            double t = Range.clip(timer.seconds() / timeSeconds, 0, 1);

            RodeTarget = (int) Etesito.lerp(currentTicks, ticks, t);

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
            rodeController.reset();

            return false;
        }

    }

    public Action pickUpSampleSubmersible() {
        return new SequentialAction(
                rodePickSampleSubmersible1(),
                new SleepAction(0.1),
                etesito.pickSampleAction(),
                etesito.wristDownAction(),
                new SleepAction(0.2),
                rodePickSampleSubmersible2(),
                etesito.wristContractAction(),
                new SleepAction(0.1)
                );
    }

    public Action putSampleSubmersible() {
        return new SequentialAction(
                rodeDown(),
                new SleepAction(0.3),

                new ParallelAction(
                armUp(),
                        etesito.wristBasketAction()
                ),

                new SleepAction(0.4),
                new RodeToPos(Etesito.highRodePos)

                );
    }

    public Action pickUpSample() {
        return new SequentialAction(
                rodePickSampleSample2(),
                new SleepAction(0.3),
                etesito.wristBasketAction(),
                rodeDown(),
                etesito.pickSampleSlowAction(),
                new SleepAction(0.2),
                armUp(),
                new SleepAction(0.4)

        );
    }

    public Action dropearSample() {
        return new SequentialAction(
                new SleepAction(0.05),
                etesito.dropSampleAction(),
                new SleepAction(0.7),
                etesito.wristDownAction(),
                new SleepAction(0.1),
                rodeDown(),
                etesito.pickSampleSlowAction(),
                new SleepAction(0.5)

        );
    }

    public Action putSpecimenFirst() {
        return new SequentialAction(
                rodeSpecimen(),
                new SleepAction(0.2),
                etesito.dropSpecimenAction(),
                new SleepAction(0.2)

                );
    }

    public Action armDownSpecimenFirst() {
        return new SequentialAction(
                new SleepAction(0.6),
                etesito.wristDownMAction(),
                armDown()
        );
    }

    public Action putSpecimenSecond() {
        return new SequentialAction(
                rodeSpecimen(),
                new SleepAction(0.2),
                armPostSpecimen(),
                new SleepAction(0.2),
                etesito.dropSpecimenAction()
        );
    }

    public Action armDownSpecimenSecond() {
        return new SequentialAction(
                rodeDown(),
                new SleepAction(0.15),
                armDown(),
                etesito.wristPickSpecimenAction()
        );
    }

    public Action putSpecimen() {
        return new SequentialAction(
                rodeSpecimen(),
                new SleepAction(0.2)
                );
    }

    public Action armDownSpecimen() {
        return new SequentialAction(
                armPostSpecimen(),
                new SleepAction(0.2),
                etesito.dropSpecimenAction(),
                rodeDown(),
                new SleepAction(0.2),
                armDown(),
                etesito.wristPickSpecimenAction()
        );
    }

    public Action pickSpecimen() {
        return new SequentialAction(
                rodePickSpecimen(),
                new SleepAction(0.15),

                etesito.pickSpecimenActionAction(),
                new SleepAction(0.3),
                armSpecimen(),
                new SleepAction(0.15),
                rodeDown()

        );
    }

    public Action pickSpecimenLast() {
        return new SequentialAction(
                rodePickSpecimen(),
                new SleepAction(0.15),

                etesito.pickSpecimenActionAction(),
                new SleepAction(0.3),
                armUp(),
                new SleepAction(0.15)

        );
    }

    public Action putSpecimenLast() {
        return new SequentialAction(
                rodeSpecimen(),
                new SleepAction(0.2),
                armPostSpecimen(),
                new SleepAction(0.2),
                etesito.dropSpecimenAction(),

                new ParallelAction(
                        rodeDown(),
                        armDownLast()
                )

        );
    }

    public Action armInit() {
        return new ArmToPos(Etesito.initArmpos);
    }

    public Action armSpecimen() {
        return new ArmToPos(Etesito.specimenArmPos);

    }

    public Action armSpecimen2() {
        return new ArmToPos(Etesito.specimenArmPos - 250);
    }

    public Action rodeSpecimen() {
        return new RodeToPos(Etesito.specimenRodePos);
    }

    public Action rodefirstSpecimen() {
        return new RodeToPos(Etesito.firstSpecimenRodePos);
    }

    public Action armFirstSpecimen() {
        return new ArmToPos(Etesito.firstSpecimenArmPos);
    }

    public Action armPostSpecimen() {
        return new ArmToPos(Etesito.postSpecimenArmPos);
    }

    public Action armDown() {
        return new ArmToPosSmooth(0, 0.7);
    }

    public Action armDownLast() {
        return new ArmToPosSmooth(0, 0.15);
    }

    public Action rodeDown() {
        return new RodeToPos(0);
    }

    public Action rodePickSampleSample1() {
        return new RodeToPos(-850);
    }

    public Action rodePickSampleSample2() {
        return new RodeToPos(-1000);
    }

    public Action rodePickSampleSpecimen1() {
        return new RodeToPos(-1250);
    }

    public Action rodePickSampleSpecimen2() {
        return new RodeToPos(-1400);
    }

    public Action rodePickSpecimen() {
        return new RodeToPos(-700);
    }

    public Action rodePickSampleSubmersible1() {
        return new RodeToPos(-100);
    }

    public Action rodePickSampleSubmersible2() {
        return new RodeToPosSmooth(-1400, 0.8);
    }

    public Action rodePutSample1() {
        return new RodeToPos(-1000);
    }

    public Action rodePutSample2() {
        return new RodeToPos(-750);
    }

    public Action rodePutSample3() {
        return new RodeToPos(-650);
    }

    public Action armUp() {
        return new ArmToPos(Etesito.highBasketArmpos);
    }

    public Action rodeHighBasket() {
        return new RodeToPos(Etesito.highRodePos - 100);
    }


    public Action armUpdate() {
        return new ArmUpdate();
    }

}