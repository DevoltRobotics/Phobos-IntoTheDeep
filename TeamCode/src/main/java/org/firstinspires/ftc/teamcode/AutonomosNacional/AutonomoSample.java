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
                etesito.wristInitAction(),
                etesito.servosInitAction()

        ));

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
                .strafeToLinearHeading(new Vector2d(-55, 50), Math.toRadians(52));

        TrajectoryActionBuilder fiveSamplePick = fourSamplePut.endTrajectory().fresh()
                .setTangent(Math.toRadians(120))
                .splineToLinearHeading(new Pose2d(-25, 0, Math.toRadians(0)), Math.toRadians(0));

        TrajectoryActionBuilder fiveSamplePut = fiveSamplePick.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-56, -48, Math.toRadians(48)), Math.toRadians(250));
        
        TrajectoryActionBuilder park = fiveSamplePut.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-30, -25), Math.toRadians(270));

        if (opModeInInit()) {
            while (opModeInInit()) {
                arm.ArmUpdateVoid();

                if (isStarted()) break;
            }

        }
        waitForStart();

        Actions.runBlocking(new ParallelAction(
                arm.armUpdate(),
                new SequentialAction(

                        new ParallelAction(
                                firstSamplePut1.build(),
                                arm.armUp(),
                                etesito.wristBasketAction(),
                                etesito.servosClimbingAction(),
                                etesito.pickSampleSlowAction()
                        ),

                        new ParallelAction(
                                firstSamplePut2.build(),
                                arm.rodeHighBasket()
                        ),
                        arm.dropearSample(),

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

                        arm.pickUpSample(),

                        new ParallelAction(
                                etesito.pickSampleSlowAction(),
                                secondSamplePut.build(),
                                arm.rodeHighBasket()
                        ),
                        arm.dropearSample(),

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

                        arm.pickUpSample(),

                        new ParallelAction(
                                etesito.pickSampleSlowAction(),
                                thirdSamplePut.build(),
                                arm.rodeHighBasket()
                        ),
                        arm.dropearSample(),

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

                        arm.pickUpSample(),

                        new ParallelAction(
                                etesito.pickSampleSlowAction(),
                                fourSamplePut.build(),
                                arm.rodeHighBasket()
                        ),

                        arm.dropearSample(),

                        //FOUR_SAMPLE_PUT

                        new ParallelAction(
                                fiveSamplePick.build(),
                                etesito.wristContractAction(),
                                arm.armDown(),
                                etesito.pickSampleSlowAction()
                                ),

                        arm.pickUpSampleSubmersible(),

                        new ParallelAction(
                        arm.putSampleSubmersible(),
                        fiveSamplePut.build()
                                ),

                        new SleepAction(0.05),

                       arm.dropearSample(),

                        new ParallelAction(
                                arm.armDownLast(),
                                etesito.mantenerSampleAction(),
                                park.build()
                                )

                        //FOUR_SAMPLE_PUT

                )));
    }
}