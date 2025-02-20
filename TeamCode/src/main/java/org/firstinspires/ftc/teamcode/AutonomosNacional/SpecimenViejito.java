package org.firstinspires.ftc.teamcode.AutonomosNacional;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Comands.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Comands.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Comands.Etesito;
import org.firstinspires.ftc.teamcode.RR.PinpointDrive;

@Disabled
@Autonomous
public class SpecimenViejito extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(10, -60, Math.toRadians(0));

        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        Etesito etesito = new Etesito();
        ArmSubsystem arm = new ArmSubsystem(hardwareMap);
        etesito.init(hardwareMap);

        ChassisSubsystem chassis = new ChassisSubsystem(hardwareMap);

        Actions.runBlocking(new SequentialAction(
                etesito.pickSpecimenActionAction(),
                new SleepAction(0.2),
                arm.armInit(),
                new SleepAction(0.7),
                etesito.wristInitAction(),
                etesito.servosInitAction()

        ));

        TrajectoryActionBuilder firstSpecimenPut = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(2, -27.5), Math.toRadians(270));

        TrajectoryActionBuilder firstSampleMove1 = firstSpecimenPut.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(29, -31), Math.toRadians(24));

        //TrajectoryActionBuilder firstSampleMove2 = firstSampleMove1.endTrajectory().fresh()
          //      .turnTo(Math.toRadians(24));

        TrajectoryActionBuilder firstSamplePut = firstSampleMove1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(29, -35), Math.toRadians(320));

        TrajectoryActionBuilder secondSampleMove1 = firstSamplePut.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(39, -31), Math.toRadians(25));

        //TrajectoryActionBuilder secondSampleMove2 = secondSampleMove1.endTrajectory().fresh()
          //      .turnTo(Math.toRadians(17));

        TrajectoryActionBuilder secondSamplePut = secondSampleMove1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(39, -35), Math.toRadians(330));

        TrajectoryActionBuilder thirdSampleMove1 = secondSamplePut.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(49, -31), Math.toRadians(27));

        //TrajectoryActionBuilder thirdSampleMove2 = thirdSampleMove1.endTrajectory().fresh()
          //      .turnTo(Math.toRadians(17));

        TrajectoryActionBuilder thirdSamplePut = thirdSampleMove1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(38, -48), Math.toRadians(270));

        TrajectoryActionBuilder secondSpecimenPut1 = thirdSamplePut.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(2, -24));

        TrajectoryActionBuilder secondSpecimenPut2 = secondSpecimenPut1.endTrajectory().fresh()
              .strafeToConstantHeading(new Vector2d(-8, -24));

        TrajectoryActionBuilder thirdSpecimenPick = secondSpecimenPut2.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(36, -48));

        TrajectoryActionBuilder thirdSpecimenPut = thirdSpecimenPick.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-4, -24));

        TrajectoryActionBuilder fourSpecimenPick = thirdSpecimenPut.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(36, -48));

        TrajectoryActionBuilder fourSpecimenPut = fourSpecimenPick.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(0, -24));

        TrajectoryActionBuilder fiveSpecimenPick = fourSpecimenPut.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(36, -48))
                ;

        TrajectoryActionBuilder fiveSpecimenPut = fiveSpecimenPick.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(4, -24))
                ;

        if (opModeInInit()) {
            while (opModeInInit()) {
                arm.ArmUpdateVoid();
            }

        }

        waitForStart();

        Actions.runBlocking(new ParallelAction(
                arm.armUpdate(),

                new SequentialAction(
                        new ParallelAction(
                                firstSpecimenPut.build(),
                                arm.armSpecimen(),
                                etesito.wristSpecimenAction(),
                                etesito.servosClimbingAction()

                                ),

                        arm.putSpecimenFirst(),

                        //FIRST_SPECIMEN_PUT

                        new ParallelAction(
                                firstSampleMove1.build(),
                                arm.armDownSpecimenFirst(),
                                etesito.pickSampleAction()

                                ),
                                etesito.wristDownAction(),
                                new SleepAction(0.1),

                                arm.rodePickSampleSpecimen2(),

                        new SleepAction(0.5),

                        new ParallelAction(
                                etesito.pickSampleSlowAction(),
                                etesito.wristContractAction(),
                                firstSamplePut.build(),
                                arm.rodePutSample1()

                        ),
                        etesito.dropSampleAction(),
                        new SleepAction(0.25),

                        //MOVE_FIRST_TO_HUMAN

                        new ParallelAction(
                                etesito.wristContractAction(),
                                secondSampleMove1.build(),
                                arm.rodeDown(),
                                etesito.pickSampleAction()
                                ),

                        etesito.wristDownAction(),
                        new SleepAction(0.1),

                                arm.rodePickSampleSpecimen2(),

                        new SleepAction(0.5),

                        new ParallelAction(
                                etesito.pickSampleSlowAction(),
                                etesito.wristContractAction(),
                                secondSamplePut.build(),
                                arm.rodePutSample2()

                        ),
                        etesito.dropSampleAction(),
                        new SleepAction(0.25),


                        //SECOND_TOHUMAN


                        new ParallelAction(
                                thirdSampleMove1.build(),
                                arm.rodeDown(),
                                etesito.wristContractAction(),
                                etesito.pickSampleAction()
                                ),

                        etesito.wristDownAction(),
                        new SleepAction(0.1),

                        arm.rodePickSampleSpecimen1(),
                        new SleepAction(0.5),
                        etesito.wristDownMAction(),
                        arm.rodeDown(),
                        new SleepAction(0.1),


                        thirdSamplePut.build(),

                        arm.rodePickSpecimen(),
                        etesito.wristPickSpecimenAction(),
                        new SleepAction(0.1),

                        new ParallelAction(
                        etesito.dropSampleAction(),
                        arm.pickSpecimen()
                                ),

                        //THIRD_TO_HUMAN && PICK_SECOND

                        new ParallelAction(
                                etesito.mantenerSampleAction(),
                                etesito.wristSpecimenAction(),
                                secondSpecimenPut1.build()
                        ),
                                new ParallelAction(
                                arm.putSpecimenSecond(),
                                        secondSpecimenPut2.build()
                                ),

                        //SECOND_SPECIMENPUT

                        new ParallelAction(
                                arm.armDownSpecimenSecond(),
                                thirdSpecimenPick.build()
                        ),
                        arm.pickSpecimen(),

                        new ParallelAction(
                                etesito.wristSpecimenAction(),
                                thirdSpecimenPut.build()
                        ),
                        arm.putSpecimen(),

                        //THIRD_SPECIMENPUT

                        new ParallelAction(
                                arm.armDownSpecimen(),
                                fourSpecimenPick.build()
                        ),
                        arm.pickSpecimen(),

                        new ParallelAction(
                                etesito.wristSpecimenAction(),
                                fourSpecimenPut.build()
                        ),
                        arm.putSpecimen(),

                        //FOUR_SPECIMENPUT

                        new ParallelAction(
                                arm.armDownSpecimen(),
                                fiveSpecimenPick.build()
                        ),
                        arm.pickSpecimen(),

                        new ParallelAction(
                                etesito.wristSpecimenAction(),
                                fiveSpecimenPut.build()
                        ),

                        arm.putSpecimenLast(),
                        new SleepAction(0.2)
                        //Five_specimen

                        ))
        );

    }
}