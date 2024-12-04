package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Comands.Arm;
import org.firstinspires.ftc.teamcode.Comands.Claw;
import org.firstinspires.ftc.teamcode.Comands.ClimbServos;
import org.firstinspires.ftc.teamcode.Comands.Etesito;
import org.firstinspires.ftc.teamcode.Comands.PIDFController;
import org.firstinspires.ftc.teamcode.Comands.Wrist;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;


@Autonomous
public class Autonomo_prueba extends LinearOpMode {

    public DcMotorEx armMotor;
    public DcMotorEx rodeMotor;

    public static PIDFController.PIDCoefficients armCoefficients = new PIDFController.PIDCoefficients(0.0015, 0, 0.0017);
    PIDFController armcontroller = new PIDFController(armCoefficients);

    public static PIDFController.PIDCoefficients rodeCoefficients = new PIDFController.PIDCoefficients(0.015, 0, 0.007);
    PIDFController rodecontroller = new PIDFController(rodeCoefficients);

    public Etesito etesitoAu = new Etesito();

    @Override
    public void runOpMode() throws InterruptedException{

        Pose2d initialPose = new Pose2d(10, -60, Math.abs(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        ClimbServos climbServos = new ClimbServos(hardwareMap);

        ArmSubsystem armSubsystem = new ArmSubsystem(hardwareMap);

        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rodeMotor = hardwareMap.get(DcMotorEx.class, "rd");
        rodeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rodeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armcontroller.reset();
        rodecontroller.reset();
        /*TrajectoryActionBuilder first_specimen = drive.actionBuilder(initialPose)
                .setTangent(0)
                .strafeToLinearHeading(new Vector2d(10, -33), Math.toRadians(270))
                //extend rode
                .strafeToLinearHeading(new Vector2d(10, -35), Math.toRadians(270));
                //open_claw || timer || wrist_down &&  && contract_rode && down_arm

                //arm_down && wrist_medium

                //FIRST_SPECIMEN

        TrajectoryActionBuilder second_specimen = first_specimen.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(32, -52), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(32, -60), Math.toRadians(270))
                //close_claw
                .strafeToLinearHeading(new Vector2d(5, -33), Math.toRadians(270))
                //armMotor && wrist_specimen
                .strafeToLinearHeading(new Vector2d(5, -35), Math.toRadians(270));
                //open_claw || timer || wrist_down &&  && contract_rode && down_arm

                //SECOND_SPECIMEN

        TrajectoryActionBuilder third_specimen = second_specimen.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(32, -52), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(32, -60), Math.toRadians(270))
                //close_claw
                .strafeToLinearHeading(new Vector2d(0, -33), Math.toRadians(270))
                //armMotor && wrist_specimen
                .strafeToLinearHeading(new Vector2d(0, -35), Math.toRadians(270));
                //open_claw || timer || wrist_down &&  && contract_rode && down_arm

                //THIRD_SPECIMEN

        TrajectoryActionBuilder estacionar = third_specimen.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(40, -58), Math.toRadians(90));

         */


        TrajectoryActionBuilder firstSpecimenPut = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(10, -28), Math.toRadians(270));

        TrajectoryActionBuilder firstSpecimenAccommodate = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(10, -26), Math.toRadians(270));

        TrajectoryActionBuilder secondSpecimenPick = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(270))
                .splineTo(new Vector2d(32, -48), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(32, -60), Math.toRadians(270));

        Actions.runBlocking(new SequentialAction(
                claw.pick(),
                arm.armInit(),
                climbServos.servosInit(),
                wrist.wristInit()
                ));

        while (opModeInInit() ){
            arm.updating();
        }

        waitForStart();

        while (opModeIsActive()){
            arm.updating();
        }

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                firstSpecimenPut.build(),
                                wrist.wristSpecimen(),
                                arm.armSpecimen(),
                                climbServos.servosUp()
                        ),
                arm.rodeSpecimen()
                /*new ParallelAction(
                        firstSpecimenAccommodate.build(),
                        claw.drop()
                )*/
                /*new ParallelAction(
                        armMotor.rodeDown(),
                        wrist.wristMedium()

                ),
                new ParallelAction(
                        armMotor.armDown(),
                        secondSpecimenPick.build()

                )*/


                ));





}

}

class ArmSubsystem {

    Autonomo_prueba hardware = new Autonomo_prueba();

    ElapsedTime timer = new ElapsedTime();

    Etesito etesito = new Etesito();


    public double  powerArm;
    public double  powerRode;

    public ArmSubsystem(HardwareMap hardwareMap) {

    }

    public void updating() {
        hardware.armMotor.setPower(-hardware.armcontroller.update(hardware.armMotor.getCurrentPosition()) * powerArm);
        hardware.rodeMotor.setPower(hardware.rodecontroller.update(hardware.rodeMotor.getCurrentPosition() * powerRode));

    }

    public class ArmSpecimen implements Action {
        // checks if the lift motor has been powered on

        boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            if (!initialized) {
                hardware.armcontroller.targetPosition = etesito.specimen_ArmPos + 150;
                powerArm = 0.4;
                initialized = true;

            }

            double armPos = hardware.armMotor.getCurrentPosition();

            hardware.armMotor.setPower(-hardware.armcontroller.update(hardware.armMotor.getCurrentPosition()) * powerArm);


            if (armPos > etesito.specimen_ArmPos) {
                return true;
            } else {
                hardware.armcontroller.targetPosition = etesito.specimen_ArmPos;
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
                hardware.armcontroller.targetPosition = -1200;
                powerArm = 0.4;
                initialized = true;

            }

            hardware.armMotor.setPower(-hardware.armcontroller.update(hardware.armMotor.getCurrentPosition()) * powerArm);

            double armPos = hardware.armMotor.getCurrentPosition();

            packet.put("armPos",armPos);
            packet.put("armPower", hardware.armMotor.getPower());

            if (armPos > -900){
                return true;

            } else {
                hardware.armcontroller.targetPosition = -900;
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
            hardware.armcontroller.targetPosition = 0;

            int armPos = hardware.armMotor.getCurrentPosition();

            // checks lift's current position

            if (armPos < -150) {
                return true;
            } else {
                hardware.armcontroller.targetPosition = 0;
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
            hardware.rodecontroller.targetPosition = etesito.rode_specimen;

            double rodePos = hardware.rodeMotor.getCurrentPosition();

            if (rodePos > etesito.rode_specimen + 150) {
                return true;

            } else {
                hardware.rodecontroller.targetPosition = etesito.rode_specimen + 150;
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
            hardware.rodecontroller.targetPosition = 50;

            double rodePos = hardware.rodeMotor.getCurrentPosition();

            if (rodePos < 0) {
                return true;

            } else {
                powerRode = 0.09;
                hardware.rodecontroller.targetPosition = 0;

                return false;

            }

        }

    }

    public Action rodeDown() {
        return new RodeDown();

    }

    /*public class ArmInPos implements Action {

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
    }*/


}


