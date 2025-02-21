package org.firstinspires.ftc.teamcode.Comands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;


import com.acmerobotics.roadrunner.ParallelAction;
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

            ArmTarget = (int) etesito.lerp(currentTicks, ticks, t);

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

            RodeTarget = (int) etesito.lerp(currentTicks, ticks, t);

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

    class SamplePickColor implements Action {

        String colorTarget;

        int ticks;
        double timeSeconds;

        int currentTicks;

        ElapsedTime timer = null;

        boolean detener = true;

        public SamplePickColor(String color, int ticksPos, double timeSeconds) {
            this.ticks = ticksPos;
            this.timeSeconds = timeSeconds;
            this.colorTarget = color;
        }

        private double lerp(double start, double end, double t) {
            return start * (1 - t) + end * t;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
                currentTicks = rodeMotor.getCurrentPosition();
            }
            intake.setPower(-1);

            double t = Range.clip(timer.seconds() / timeSeconds, 0, 1);

            RodeTarget = (int) lerp(currentTicks, ticks, t);

            if (timer.seconds() <= timeSeconds){
                detener = !etesito.isSampleDetected(colorTarget);

            }else {
                detener = false;

            }

            return detener;
        }
    }

    public Action armInit() {
        return new ArmToPos(etesito.initArmpos);
    }

    public Action armSpecimenFirst() {
        return new ParallelAction(
                new ArmToPos(etesito.specimenArmPos - 70),
                new RodeToPos(etesito.specimenDownRodePos - 100)

        );
    }

    public Action armSpecimen() {
        return new ParallelAction(
                new ArmToPos(etesito.specimenArmPos - 70),
                new RodeToPos(etesito.specimenDownRodePos)

        );
    }

    public Action armSpecimenPrueba1() {
        return new ArmToPos(etesito.specimenArmPos - 150);

    }

    public Action armSpecimenPrueba2() {
        return new ArmToPos(etesito.specimenArmPos - 200);

    }

    public Action rodeSpecimenPrueba() {
        return new RodeToPos(-900);
    }


    public Action armSpecimenPut() {
        return new ArmToPos(etesito.specimenArmPos + 400);
    }

    public Action rodeSpecimen() {
        return new RodeToPos(etesito.specimenRodePos - 250);
    }

    public Action rodeSpecimenPrevious() {
        return new RodeToPos(etesito.specimenDownRodePos);
    }

    public Action armDown() {
        return new ArmToPosSmooth(0, 1);
    }

    public Action armDownLast() {
        return new ArmToPos(0);
    }

    public Action armPostSpecimen() {
        return new ArmToPos(-1200);
    }

    public Action rodeDown() {
        return new RodeToPos(0);
    }

    public Action rodSemiDown() {
        return new RodeToPos(-70);
    }

    public Action rodePickSubmersible() {
        return new RodeToPos(-700);
    }

    public Action rodePickSample() {
        return new RodeToPosSmooth(-800, 0.5);
    }

    public Action rodePutSample1() {
        return new RodeToPos(-800);
    }

    public Action rodePutSample2() {
        return new RodeToPos(-800);
    }

    public Action rodePutSample3() {
        return new RodeToPos(-900);
    }

    public Action armUp() {
        return new ArmToPos(etesito.highBasketArmpos);
    }

    public Action rodeHighBasket() {
        return new RodeToPos(etesito.highRodePos - 100);
    }


    public Action armUpdate() {
        return new ArmUpdate();
    }

    public Action samplePickRedSM() {
        return new SamplePickColor("red", -1600, 2);
    }
}