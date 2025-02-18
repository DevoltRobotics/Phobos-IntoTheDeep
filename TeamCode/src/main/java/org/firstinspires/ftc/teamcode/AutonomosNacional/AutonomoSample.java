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
public class AutonomoSample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-38, -60, Math.toRadians(90));

        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        Etesito etesito = new Etesito();
        ArmSubsystem arm = new ArmSubsystem(hardwareMap);

        etesito.init(hardwareMap);

        Actions.runBlocking(new SequentialAction(
                arm.armInit(),
                new SleepAction(0.7),
                etesito.wristInit(),
                etesito.servosInit()

        ));

        while (opModeInInit()) {
            arm.ArmUpdateVoid();

        }

        TrajectoryActionBuilder firstSamplePut1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-38, -53.5), Math.toRadians(90)); // TODO:

        TrajectoryActionBuilder firstSamplePut2 = firstSamplePut1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-61, -53.5), Math.toRadians(50));

        TrajectoryActionBuilder secondSamplePick1 = firstSamplePut2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-35, -35), Math.toRadians(142));

        TrajectoryActionBuilder secondSamplePick2 = secondSamplePick1.endTrajectory().fresh()
                .turnTo(Math.toRadians(146));

        TrajectoryActionBuilder secondSamplePut = secondSamplePick2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-53, -47), Math.toRadians(52))
                ;

        TrajectoryActionBuilder thirdSamplePick1 = secondSamplePut.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-45, -35), Math.toRadians(142));
                ;

        TrajectoryActionBuilder thirdSamplePick2 = thirdSamplePick1.endTrajectory().fresh()
                .turnTo(Math.toRadians(146));
                ;

        TrajectoryActionBuilder thirdSamplePut = thirdSamplePick2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-54, -48), Math.toRadians(52));

        TrajectoryActionBuilder fourSamplePick1 = thirdSamplePut.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-55, -35), Math.toRadians(142));


        TrajectoryActionBuilder fourSamplePick2 = fourSamplePick1.endTrajectory().fresh()
                .turnTo(Math.toRadians(146));

        TrajectoryActionBuilder fourSamplePut = fourSamplePick2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-54, -49), Math.toRadians(52));

        TrajectoryActionBuilder fiveSamplePick = fourSamplePut.endTrajectory().fresh()
                .setTangent(Math.toRadians(120))
                .splineToLinearHeading(new Pose2d(-25, 0, Math.toRadians(0)), Math.toRadians(0));

        TrajectoryActionBuilder fiveSamplePut1 = fiveSamplePick.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-45, 0), Math.toRadians(0));

        TrajectoryActionBuilder fiveSamplePut2 = fiveSamplePut1.endTrajectory().fresh()
                .setTangent(Math.toRadians(120))
                .splineToLinearHeading(new Pose2d(-55, -47, Math.toRadians(48)), Math.toRadians(180));
        
        TrajectoryActionBuilder park = fiveSamplePut2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-45, -25), Math.toRadians(270));

        waitForStart();

        Actions.runBlocking(new ParallelAction(
                arm.armUpdate(),
                new SequentialAction(

                        new ParallelAction(
                                firstSamplePut1.build(),
                                arm.armUp(),
                                etesito.wristBasket(),
                                etesito.servosClimbing(),
                                etesito.pickSampleSlowAction()
                        ),

                        new ParallelAction(
                                firstSamplePut2.build(),
                                arm.rodeHighBasket()
                        ),
                        new SleepAction(0.1),

                        etesito.dropSampleAction(),
                        new SleepAction(0.8),
                        etesito.wristDown(),
                        new SleepAction(0.1),
                        arm.rodeDown(),
                        etesito.pickSampleSlowAction(),
                        new SleepAction(0.5),

                        //PUT_FIRST

                        new ParallelAction(
                                secondSamplePick1.build(),
                                arm.armDown()
                        ),
                        new ParallelAction(
                        arm.rodePickSampleSample1(),
                        etesito.pickSampleAction()
                        ),

                        new SleepAction(0.25),

                        secondSamplePick2.build(),
                        arm.rodePickSampleSample2(),

                        new SleepAction(0.3),
                        etesito.wristBasket(),
                        arm.rodeDown(),
                        etesito.pickSampleSlowAction(),
                        new SleepAction(0.2),
                        arm.armUp(),
                        new SleepAction(0.4),
                        new ParallelAction(
                                etesito.pickSampleSlowAction(),
                                secondSamplePut.build(),
                                arm.rodeHighBasket()
                        ),
                        new SleepAction(0.1),

                        etesito.dropSampleAction(),
                        new SleepAction(0.6),
                        etesito.wristDown(),
                        new SleepAction(0.1),
                        arm.rodeDown(),
                        etesito.pickSampleSlowAction(),
                        new SleepAction(0.5),

                        //SECOND_SAMPLE_PUT


                        new ParallelAction(
                                thirdSamplePick1.build(),
                                arm.armDown()
                        ),
                        new ParallelAction(
                                arm.rodePickSampleSample1(),
                                etesito.pickSampleAction()
                        ),

                        new SleepAction(0.25),

                        thirdSamplePick2.build(),
                        arm.rodePickSampleSample2(),

                        new SleepAction(0.3),
                        etesito.wristBasket(),
                        arm.rodeDown(),
                        etesito.pickSampleSlowAction(),
                        new SleepAction(0.2),
                        arm.armUp(),
                        new SleepAction(0.4),
                        new ParallelAction(
                                etesito.pickSampleSlowAction(),
                                thirdSamplePut.build(),
                                arm.rodeHighBasket()
                        ),
                        new SleepAction(0.1),

                        etesito.dropSampleAction(),
                        new SleepAction(0.6),
                        etesito.wristDown(),
                        new SleepAction(0.1),
                        arm.rodeDown(),
                        etesito.pickSampleSlowAction(),
                        new SleepAction(0.5),

                        //THIRD_SAMPLEPUT

                        new ParallelAction(
                                fourSamplePick1.build(),
                                arm.armDown()
                        ),
                        new ParallelAction(
                                arm.rodePickSampleSample1(),
                                etesito.pickSampleAction()
                        ),

                        new SleepAction(0.25),

                        fourSamplePick2.build(),
                        arm.rodePickSampleSample2(),

                        new SleepAction(0.3),
                        etesito.wristBasket(),
                        arm.rodeDown(),
                        etesito.pickSampleSlowAction(),
                        new SleepAction(0.2),
                        arm.armUp(),
                        new SleepAction(0.4),
                        new ParallelAction(
                                etesito.pickSampleSlowAction(),
                                fourSamplePut.build(),
                                arm.rodeHighBasket()
                        ),
                        new SleepAction(0.1),

                        etesito.dropSampleAction(),
                        new SleepAction(0.6),
                        etesito.wristDown(),
                        new SleepAction(0.1),
                        arm.rodeDown(),
                        etesito.mantenerSampleAction(),
                        new SleepAction(0.5),

                        //FOUR_SAMPLE_PUT

                        new ParallelAction(
                                fiveSamplePick.build(),
                                etesito.wristContract(),
                                arm.armDown(),
                                etesito.pickSampleSlowAction()
                                ),

                        arm.rodePickSampleSubmersible1(),
                        new SleepAction(0.1),
                        etesito.pickSampleAction(),
                        etesito.wristDown(),
                        new SleepAction(0.2),
                        arm.rodePickSampleSubmersible2(),
                        etesito.wristContract(),
                        new SleepAction(0.1),

                        new ParallelAction(
                        arm.rodeDown(),
                        fiveSamplePut1.build()
                                ),

                        new ParallelAction(
                                fiveSamplePut2.build(),
                                arm.armUp(),
                                etesito.wristBasket(),

                                new SleepAction(0.5),
                                arm.rodeHighBasket()
                        ),
                        new SleepAction(0.05),

                        etesito.dropSampleAction(),
                        new SleepAction(0.4),
                        etesito.wristContract(),
                        new SleepAction(0.1),

                        new ParallelAction(
                        arm.rodeDown(),
                        etesito.mantenerSampleAction(),
                                park.build()
                                ),
                        arm.armDownLast(),
                        new SleepAction(0.5)





                        //FOUR_SAMPLE_PUT


                )));


    }
}