package org.firstinspires.ftc.teamcode.AutonomosTorreon;

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

        while (opModeInInit()){
            arm.ArmUpdateVoid();

        }

        TrajectoryActionBuilder firstSamplePut1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-38, -53), Math.toRadians(90))
                ;

        TrajectoryActionBuilder firstSamplePut2 = firstSamplePut1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-60, -56.5), Math.toRadians(45))
                ;

        TrajectoryActionBuilder secondSamplePick1 = firstSamplePut2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-32, -50), Math.toRadians(180))
                ;

        TrajectoryActionBuilder secondSamplePick2 = secondSamplePick1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-32, -22), Math.toRadians(180))
                ;

        TrajectoryActionBuilder secondSamplePut = secondSamplePick2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-58, -54), Math.toRadians(45))
                ;

        TrajectoryActionBuilder thirdSamplePick1 = secondSamplePut.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-44, -50), Math.toRadians(180))
                ;

        TrajectoryActionBuilder thirdSamplePick2 = thirdSamplePick1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-44, -23), Math.toRadians(180))
                ;

        TrajectoryActionBuilder thirdSamplePut = thirdSamplePick2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-58, -54), Math.toRadians(45))
                ;

         TrajectoryActionBuilder fourSamplePick1 = thirdSamplePut.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-52, -50), Math.toRadians(180))
                ;

        TrajectoryActionBuilder fourSamplePick2 = fourSamplePick1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-52, -23), Math.toRadians(180))
                ;

        TrajectoryActionBuilder fourSamplePut = fourSamplePick2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-58, -54), Math.toRadians(45))
                ;

        TrajectoryActionBuilder park = fourSamplePut.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-47, -22), Math.toRadians(270))
                ;

        waitForStart();

        Actions.runBlocking(new ParallelAction(
                arm.armUpdate(),
                new SequentialAction(
                        new ParallelAction(
                                firstSamplePut1.build(),
                                arm.armUp(),
                                etesito.wristSpecimen(),
                                etesito.servosClimbing()
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

                        etesito.wristDown(),
                        secondSamplePick1.build(),
                        new ParallelAction(
                                secondSamplePick2.build(),
                                arm.armDown()
                        ),
                        new ParallelAction(
                                arm.rodePickSampleSample1(),
                                etesito.pickSampleAction()
                                ),
                        new SleepAction(0.6),
                        etesito.wristSpecimen(),
                        etesito.pickSampleSlowAction(),
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
                        etesito.mantenerSampleAction(),
                        new SleepAction(0.5),

                        //SECOND_SAMPLE_PUT


                        thirdSamplePick1.build(),
                        new ParallelAction(
                                thirdSamplePick2.build(),
                                arm.armDown(),
                                etesito.wristDown()
                        ),
                        new ParallelAction(
                                arm.rodePickSampleSample2(),
                                etesito.pickSampleAction()
                        ),
                        new SleepAction(0.6),
                        arm.rodeDown(),
                        etesito.wristSpecimen(),
                        etesito.pickSampleSlowAction(),
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

                        fourSamplePick1.build(),
                        new ParallelAction(
                                fourSamplePick2.build(),
                                arm.armDown()
                        ),
                        new SleepAction(0.1),
                        new ParallelAction(
                                arm.rodePickSampleSample2(),
                                etesito.pickSampleAction()
                        ),
                        new SleepAction(0.6),
                        arm.rodeDown(),
                        etesito.wristSpecimen(),
                        etesito.pickSampleSlowAction(),
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
                        new ParallelAction(
                                park.build(),
                                arm.armDown()
                        )



                        /*thirdSamplePick.build(),
                        thirdSamplePut.build(),
                        fourSamplePick.build(),
                        fourSamplePut.build(),
                fiveSamplePick.build(),
                fiveSamplePut.build(),

                park.build()


*/
                        //ALL_SAMPLES TO HUMAN

                )));


    }
}
