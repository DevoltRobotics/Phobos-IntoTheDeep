package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Comands.Arm;
import org.firstinspires.ftc.teamcode.Comands.Claw;
import org.firstinspires.ftc.teamcode.Comands.ClimbServos;
import org.firstinspires.ftc.teamcode.Comands.Wrist;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.opencv.core.Mat;


@Autonomous
public class AutonomoSpecimenChido extends LinearOpMode {

    boolean initialized = false;

    @Override
    public void runOpMode() throws InterruptedException{

        Pose2d initialPose = new Pose2d(10, -60, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        ClimbServos climbServos = new ClimbServos(hardwareMap);

        TrajectoryActionBuilder firstSpecimenPut = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(15, -24.5), Math.toRadians(270))
                ;

        TrajectoryActionBuilder firstSpecimenAccommodate = firstSpecimenPut.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(10, -26), Math.toRadians(270))
                ;

        TrajectoryActionBuilder secondSpecimenMove = firstSpecimenAccommodate.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(49, -33, Math.toRadians(270)), 0)
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(65, -2, Math.toRadians(270)), Math.toRadians(50))
                ;

        TrajectoryActionBuilder secondSpecimenPick = secondSpecimenMove.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(58, -42), Math.toRadians(270))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(58, -37), Math.toRadians(270))
                .waitSeconds(0.6)
                .strafeToLinearHeading(new Vector2d(58, -42), Math.toRadians(270))
        ;

        TrajectoryActionBuilder secondSpecimenPut = secondSpecimenPick.endTrajectory().fresh()
                .setReversed(true)
                .setTangent(Math.toRadians(180))                .strafeToLinearHeading(new Vector2d(58, -42), Math.toRadians(270))

                .splineToConstantHeading(new Vector2d(0, -47), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(0, -26), Math.toRadians(270))

                ;

        /*TrajectoryActionBuilder secondSpecimenAccommodate = secondSpecimenPut.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(0, -28), Math.toRadians(270))
                ;

        TrajectoryActionBuilder thirdSpecimenPick = secondSpecimenAccommodate.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(53, -38), Math.toRadians(270))
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(53, -45), Math.toRadians(270))
                ;

        TrajectoryActionBuilder thirdSpecimenPut = thirdSpecimenPick.endTrajectory().fresh()
                .setReversed(true)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-12, -47), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-12, -26.5), Math.toRadians(270))

                ;

        TrajectoryActionBuilder thirdSpecimenAccommodate = thirdSpecimenPut .endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-12, -28), Math.toRadians(270))
                ;

        TrajectoryActionBuilder park = thirdSpecimenAccommodate .endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(32, -52), 0)
                ;*/


        Actions.runBlocking(new SequentialAction(
                claw.pick(),
                new SleepAction(0.8),
                arm.armInit(),
                new SleepAction(1.2),
                climbServos.servosInit(),
                wrist.wristInit()
                ));

        while (opModeInInit() ){
            arm.updateArm();
        }

        waitForStart();

        Actions.runBlocking(new ParallelAction(arm.armInPos(),
                        new SequentialAction(
                                arm.armSpecimen(),
                                new ParallelAction(
                                firstSpecimenPut.build(),
                                        climbServos.servosUp(),
                                        wrist.wristSpecimen()

                                        ),
                new SleepAction(0.3),
                arm.rodeSpecimen(),
                new SleepAction(0.6),
                firstSpecimenAccommodate.build(),
                claw.drop(),
                new SleepAction(0.2),
                wrist.wristDownM(),
                new SleepAction(0.3),
                arm.rodeDown(),
                new SleepAction(0.8),
                arm.armDown(),

                                secondSpecimenMove.build()
                //FIRST_PUT

/*
                secondSpecimenPick.build(),
                new SleepAction(0.3),
                claw.pick(),
                                new SleepAction(0.3),
                                wrist.wristSpecimen(),
                                new SleepAction(0.4),
                                arm.armSpecimen(),
                                secondSpecimenPut.build(),
                new SleepAction(0.3),
                arm.rodeSpecimen(),
                new SleepAction(0.6),
                secondSpecimenAccommodate.build(),
                claw.drop(),
                new SleepAction(0.2),
                wrist.wristMedium(),
                new SleepAction(0.3),
                arm.rodeDown(),
                new SleepAction(0.8),
                arm.armDown(),

                                /////SECOND_PUT

                                thirdSpecimenPick.build(),
                                new SleepAction(0.3),
                                claw.pick(),
                                new SleepAction(0.3),
                                wrist.wristSpecimen(),
                                new SleepAction(0.4),
                                arm.armSpecimen(),
                                thirdSpecimenPut.build(),
                                new SleepAction(0.3),
                                arm.rodeSpecimen(),
                                new SleepAction(0.6),
                                thirdSpecimenAccommodate.build(),
                                claw.drop(),
                                new SleepAction(0.2),
                                wrist.wristMedium(),
                                new SleepAction(0.3),
                                arm.rodeDown(),
                                new SleepAction(0.8),
                                arm.armDown(),
                                park.build(),
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


                        )));





        }

}


