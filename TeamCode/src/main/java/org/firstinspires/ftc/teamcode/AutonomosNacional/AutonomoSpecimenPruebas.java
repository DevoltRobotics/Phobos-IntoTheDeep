package org.firstinspires.ftc.teamcode.AutonomosNacional;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Comands.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Comands.Etesito;
import org.firstinspires.ftc.teamcode.RR.PinpointDrive;

@Autonomous
public class AutonomoSpecimenPruebas extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(10, -60, 0);

        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        Etesito etesito = new Etesito();
        ArmSubsystem arm = new ArmSubsystem(hardwareMap);

        etesito.init(hardwareMap);

        Actions.runBlocking(new SequentialAction(
                etesito.pickSpecimenAction(),
                new SleepAction(0.2),
                etesito.wristContract(),
                new SleepAction(0.1),
                arm.armInit(),
                new SleepAction(0.66),
                etesito.wristInit(),
                etesito.servosInit()

        ));

        while (opModeInInit()){
            arm.ArmUpdateVoid();

        }

        TrajectoryActionBuilder firstSpecimenPut = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-6, -27), Math.toRadians(270))
                ;

        TrajectoryActionBuilder firstSampleMove1 = firstSpecimenPut.endTrajectory().fresh()
                .setTangent(Math.toRadians(260))
                .splineToLinearHeading(new Pose2d(30, -24, Math.toRadians(356)), Math.toRadians(70))
                ;

        TrajectoryActionBuilder firstSampleMove2 = firstSampleMove1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(43, -32), Math.toRadians(290))
                ;

        TrajectoryActionBuilder secondSampleMove1 = firstSampleMove2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(45, -25), Math.toRadians(357))
                ;

        TrajectoryActionBuilder secondSampleMove2 = secondSampleMove1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(52, -40), Math.toRadians(290))
                ;

        TrajectoryActionBuilder thirdSampleMove1 = secondSampleMove2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(55, -25), Math.toRadians(0))
                ;

        TrajectoryActionBuilder thirdSampleMove2 = thirdSampleMove1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(35, -40), Math.toRadians(-60))
                ;

        TrajectoryActionBuilder secondSpecimenPick = thirdSampleMove2.endTrajectory().fresh()
                .turnTo(Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(35.5, -57), Math.toRadians(270))
                ;

        TrajectoryActionBuilder secondSpecimenPut = secondSpecimenPick.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-5, -28), Math.toRadians(270))
                ;

        TrajectoryActionBuilder thirdSpecimenPick = secondSpecimenPut.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(35.5, -57), Math.toRadians(270))
                //.setTangent(Math.toRadians(270))
                //.splineToLinearHeading(new Pose2d(33, -57.5, Math.toRadians(270)), Math.toRadians(290))

                ;

        TrajectoryActionBuilder thirdSpecimenPut = thirdSpecimenPick.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(0, -28), Math.toRadians(270))
                ;

        TrajectoryActionBuilder fourSpecimenPick = thirdSpecimenPut.endTrajectory().fresh()

                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(32.5, -57.5, Math.toRadians(270)), Math.toRadians(290))

                ;

        TrajectoryActionBuilder fourSpecimenPut = fourSpecimenPick.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(2, -28), Math.toRadians(270))
                ;

        TrajectoryActionBuilder fiveSpecimenPick = fourSpecimenPut.endTrajectory().fresh()

                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(40, -54, Math.toRadians(270)), Math.toRadians(270))
                ;

        TrajectoryActionBuilder fiveSpecimenPut = fiveSpecimenPick.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(6, -28), Math.toRadians(270))
                ;


        waitForStart();

        Actions.runBlocking(new ParallelAction(
                arm.armUpdate(),
                new SequentialAction(

                        new ParallelAction(
                                firstSpecimenPut.build(),
                                arm.armSpecimen1(),
                                etesito.wristSpecimen(),
                                etesito.servosClimbing()
                                ),

                        arm.rodeSpecimen(),
                        new SleepAction(0.4),
                        etesito.dropSpecimenAction(),
                        new SleepAction(0.2),
                        new SleepAction(0.2),
                        arm.rodeDown(),
                        new SleepAction(0.35),

                        //FIRST_SPECIMEN_PUT


                        new ParallelAction(
                        arm.armDown(),
                        etesito.wristDown(),
                        firstSampleMove1.build()
                                ),

                        new ParallelAction(
                                arm.rodePickSample1(),
                                etesito.pickSampleAction()
                        ),

                        new SleepAction(0.5),
                        etesito.pickSampleSlowAction(),
                        etesito.wristContract(),
                        new SleepAction(0.05),
                        new ParallelAction(
                        firstSampleMove2.build(),
                                arm.rodePutSample1()

                                ),
                                etesito.dropSampleAction(),
                                new SleepAction(0.3),

                        //MOVE_FIRST_TO_HUMAN

                        new ParallelAction(
                        etesito.mantenerSampleAction(),
                        etesito.wristDown(),
                        arm.rodeDown()
                                ),
                        new SleepAction(0.1),
                        secondSampleMove1.build(),


                        new ParallelAction(
                            arm.rodePickSample2(),
                                etesito.pickSampleAction()
                        ),
                                new SleepAction(0.5),
                        etesito.pickSampleSlowAction(),
                        etesito.wristContract(),
                        new SleepAction(0.05),
                        new ParallelAction(
                        secondSampleMove2.build(),
                        arm.rodePutSample2()
                        ),

                        etesito.dropSampleAction(),
                        new SleepAction(0.3),

                        //SECOND_TOHUMAN

        new ParallelAction(
                etesito.mantenerSampleAction(),
                etesito.wristDown(),
                arm.rodeDown()
        ),
                new SleepAction(0.1),
                thirdSampleMove1.build(),


                new ParallelAction(
                        arm.rodePickSample3(),
                        etesito.pickSampleAction()
                ),
                new SleepAction(0.5),
                etesito.pickSampleSlowAction(),
                etesito.wristContract(),
                new SleepAction(0.05),
                new ParallelAction(
                        thirdSampleMove2.build(),
                        arm.rodePutSample3()
                ),

                etesito.dropSampleAction(),
                new SleepAction(0.3),

           //   THIRD_TO_HUMAN

                                new ParallelAction(
                                        etesito.mantenerSampleAction(),
                                        arm.rodeDown(),
                                        etesito.wristDown(),
                                        secondSpecimenPick.build()
                                )

                        /*

                        etesito.pickSpecimenAction(),
                        new SleepAction(0.35),
                        new ParallelAction(
                                arm.armSpecimenSecond(),
                                etesito.wristSpecimen(),
                                secondSpecimenPut.build()
                        ),
                arm.rodeSpecimen(),
                new SleepAction(0.4),
                etesito.dropSpecimenAction(),
                new SleepAction(0.2),
                etesito.wristDown(),
                new SleepAction(0.2),
                arm.rodeDown(),
                new SleepAction(0.35),

                        //SECOND_SPECIMENPUT

                        new ParallelAction(
                                etesito.wristDown(),
                                arm.armDown(),
                                thirdSpecimenPick.build()
                        ),


                        etesito.pickSpecimenAction(),
                        new SleepAction(0.35),
                        new ParallelAction(
                                arm.armSpecimen2(),
                                etesito.wristSpecimen(),
                                thirdSpecimenPut.build()
                        ),
                        arm.rodeSpecimen(),
                        new SleepAction(0.4),
                        etesito.dropSpecimenAction(),
                        new SleepAction(0.2),
                        etesito.wristDown(),
                        new SleepAction(0.2),
                        arm.rodeDown(),
                        new SleepAction(0.35),

                        //THIRDSPECIMENPUT

                        new ParallelAction(
                                etesito.wristDown(),
                                arm.armDown(),
                                fourSpecimenPick.build()
                        ),

                        etesito.pickSpecimenAction(),
                        new SleepAction(0.35),
                        new ParallelAction(
                                arm.armSpecimen3(),
                                etesito.wristSpecimen(),
                                thirdSpecimenPut.build()
                        ),
                        new SleepAction(0.02),
                        arm.rodeSpecimen(),
                        new SleepAction(0.45),
                        etesito.dropSpecimenAction(),
                        new SleepAction(0.2),
                        etesito.wristDown(),
                        new SleepAction(0.2),
                        arm.rodeDown(),
                        new SleepAction(0.35),
                        arm.armDown()




*/


                        ))


        );


    }
}
