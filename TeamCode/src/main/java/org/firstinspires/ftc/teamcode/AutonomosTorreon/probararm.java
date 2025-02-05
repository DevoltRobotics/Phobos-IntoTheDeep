package org.firstinspires.ftc.teamcode.AutonomosTorreon;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Comands.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Comands.Etesito;
import org.firstinspires.ftc.teamcode.RR.PinpointDrive;

@Autonomous
public class probararm extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(10, -60, 0);

        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        Etesito etesito = new Etesito();
        ArmSubsystem arm = new ArmSubsystem(hardwareMap);

        etesito.init(hardwareMap);

        TrajectoryActionBuilder firstSpecimenPut = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-10, -32), Math.toRadians(270))
                ;

        TrajectoryActionBuilder firstSampleMove1 = firstSpecimenPut.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(30, -40), Math.toRadians(40))

                ;

        TrajectoryActionBuilder firstSampleMove2 = firstSampleMove1.endTrajectory().fresh()
                .turnTo(Math.toRadians(-50))

                ;

        TrajectoryActionBuilder secondSampleMove = firstSampleMove2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(40, -40), Math.toRadians(40))
                .turnTo(Math.toRadians(-60))
                ;

        TrajectoryActionBuilder thirdSampleMove = secondSampleMove.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(46, -40), Math.toRadians(40))
                .strafeToLinearHeading(new Vector2d(34, -40), Math.toRadians(20))
                .turnTo(Math.toRadians(290))
                ;

        TrajectoryActionBuilder secondSpecimenPick = thirdSampleMove.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(40, -40), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(40, -54), Math.toRadians(270))
                ;

        TrajectoryActionBuilder secondSpecimenPut = secondSpecimenPick.endTrajectory().fresh()

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(6, -32, Math.toRadians(270)), Math.toRadians(90))
                ;

        TrajectoryActionBuilder thirdSpecimenPick = secondSpecimenPut.endTrajectory().fresh()

                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(40, -54, Math.toRadians(270)), Math.toRadians(270))
                ;

        TrajectoryActionBuilder thirdSpecimenPut = thirdSpecimenPick.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(2, -32), Math.toRadians(270))
                ;

        TrajectoryActionBuilder fourSpecimenPick = thirdSpecimenPut.endTrajectory().fresh()

                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(40, -54, Math.toRadians(270)), Math.toRadians(270))
                ;

        TrajectoryActionBuilder fourSpecimenPut = fourSpecimenPick.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(2, -32), Math.toRadians(270))
                ;

        TrajectoryActionBuilder fiveSpecimenPick = fourSpecimenPut.endTrajectory().fresh()

                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(40, -54, Math.toRadians(270)), Math.toRadians(270))
                ;

        TrajectoryActionBuilder fiveSpecimenPut = fiveSpecimenPick.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(6, -32), Math.toRadians(270))
                ;


        /*while (opModeInInit()){
            arm.ArmUpdateVoid();

        }

        Actions.runBlocking(new SequentialAction(
                etesito.pickSpecimenAction(),
                new SleepAction(0.4),
                arm.armInit(),
                new SleepAction(0.2),
                etesito.wristInit(),
                etesito.servosInit()

        ));*/


        waitForStart();

        Actions.runBlocking(new ParallelAction(
                arm.armUpdate(),
                new SequentialAction(
                        arm.armSpecimen()


                        /*,
                        arm.rodeSpecimen(),
                        new SleepAction(0.4),
                        etesito.dropSpecimenAction(),
                        new SleepAction(0.2),
                        etesito.wristDownM(),
                        new SleepAction(0.2),
                        arm.rodeDown(),
                        new SleepAction(0.5),
                        arm.armDown(),

                        //PUT_FIRST_SPECIMEN

                        new ParallelAction(
                                firstSampleMove1.build(),
                                new SleepAction(1),
                                arm.rodePickSample1()

                                ),

                        arm.samplePickRedSpecimen(),

                        new ParallelAction(
                                arm.rodePutSample1(),
                                etesito.mantenerSampleAction(),
                                etesito.wristContract(),
                                firstSampleMove2.build()

                                ),

                        etesito.dropSampleAction(),
                        new SleepAction(0.1),



                        //FIRST SAMPLE MOVE

                        secondSampleMove.build(),
                        thirdSampleMove.build(),
                        secondSpecimenPick.build(),
                        secondSpecimenPut.build(),
                        thirdSpecimenPick.build(),
                        thirdSpecimenPut.build(),
                        fourSpecimenPick.build(),
                        fourSpecimenPut.build(),
                        fiveSpecimenPick.build(),
                        fiveSpecimenPut.build()*/

                ))

                /*
                new ParallelAction(
                etesito.armUpdate(),
                new SequentialAction(

                        ////////////////////////////////////////////////////////////////////////////////////////////////////
                new ParallelAction(
                        etesito.armUp(),
                        etesito.rodeSpecimenPrevious(),
                        etesito.wristSpecimen(),
                        firstSpecimenPut.build()
                ),
                        etesito.rodeSpecimen(),
                        new SleepAction(0.3),
                        etesito.dropAction(),
                        new SleepAction(0.1),
                        etesito.rodeDown(),
                        etesito.armDown(),

                        //FIRST_SPECIMEN_PUT

                        new SleepAction(0.1),
                        new ParallelAction(
                                new SequentialAction(
                                        new SleepAction(0.8),
                                        etesito.pickSampleAction(),
                                        new SleepAction(0.3),
                                        etesito.mantenerSampleAction(),
                                        new SleepAction(0.2),
                                        etesito.dropSampleAction(),
                                        new SleepAction(0.2),
                                        etesito.mantenerSampleAction()
                                ),
                                firstSampleMove.build()
                        ),

                        new ParallelAction(
                                new SequentialAction(
                                        new SleepAction(0.4),
                                        etesito.pickSampleAction(),
                                        new SleepAction(0.3),
                                        etesito.mantenerSampleAction(),
                                        new SleepAction(0.2),
                                        etesito.dropSampleAction(),
                                        new SleepAction(0.2),
                                        etesito.mantenerSampleAction()
                                ),
                                secondSampleMove.build()
                        ),

                        new ParallelAction(
                                new SequentialAction(
                                        new SleepAction(0.4),
                                        etesito.pickSampleAction(),
                                        new SleepAction(0.3),
                                        etesito.mantenerSampleAction(),
                                        new SleepAction(0.2),
                                        etesito.dropSampleAction(),
                                        new SleepAction(0.2),
                                        etesito.mantenerSampleAction()
                                ),
                                thirdSampleMove.build()
                        )

                        //ALL_SAMPLES TO HUMAN





                ))*/
        );


    }
}
