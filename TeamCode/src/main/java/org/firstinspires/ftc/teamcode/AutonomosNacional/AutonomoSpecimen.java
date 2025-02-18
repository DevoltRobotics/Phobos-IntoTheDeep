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
import org.firstinspires.ftc.teamcode.Comands.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Comands.Etesito;
import org.firstinspires.ftc.teamcode.RR.PinpointDrive;

@Autonomous
public class AutonomoSpecimen extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(10, -60, Math.toRadians(0));

        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        Etesito etesito = new Etesito();
        ArmSubsystem arm = new ArmSubsystem(hardwareMap);
        etesito.init(hardwareMap);

        ChassisSubsystem chassis = new ChassisSubsystem(hardwareMap);

        Actions.runBlocking(new SequentialAction(
                etesito.pickSpecimenAction(),
                new SleepAction(0.2),
                arm.armInit(),
                new SleepAction(0.7),
                etesito.wristInit(),
                etesito.servosInit()


        ));

        while (opModeInInit()) {
            arm.ArmUpdateVoid();
            if(isStarted()) break;
        }

        TrajectoryActionBuilder firstSpecimenPut = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(2, -24), Math.toRadians(270));

        TrajectoryActionBuilder firstSampleMove1 = firstSpecimenPut.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(29, -31), Math.toRadians(26));

        //TrajectoryActionBuilder firstSampleMove2 = firstSampleMove1.endTrajectory().fresh()
          //      .turnTo(Math.toRadians(24));

        TrajectoryActionBuilder firstSamplePut = firstSampleMove1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(29, -35), Math.toRadians(320));

        TrajectoryActionBuilder secondSampleMove1 = firstSamplePut.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(39, -31), Math.toRadians(26));

        //TrajectoryActionBuilder secondSampleMove2 = secondSampleMove1.endTrajectory().fresh()
          //      .turnTo(Math.toRadians(17));

        TrajectoryActionBuilder secondSamplePut = secondSampleMove1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(39, -35), Math.toRadians(315));

        TrajectoryActionBuilder thirdSampleMove1 = secondSamplePut.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(49, -31), Math.toRadians(27));

        //TrajectoryActionBuilder thirdSampleMove2 = thirdSampleMove1.endTrajectory().fresh()
          //      .turnTo(Math.toRadians(17));

        TrajectoryActionBuilder thirdSamplePut = thirdSampleMove1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(37, -42), Math.toRadians(300));

        TrajectoryActionBuilder secondSpecimenPick1 = thirdSamplePut.endTrajectory().fresh()
                .turnTo(Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(37, -53), Math.toRadians(270));

        TrajectoryActionBuilder secondSpecimenPut = secondSpecimenPick1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(2, -24));

        TrajectoryActionBuilder thirdSpecimenPick1 = secondSpecimenPut.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(38, -49));

        TrajectoryActionBuilder thirdSpecimenPut = thirdSpecimenPick1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(4, -22.3));

        TrajectoryActionBuilder fourSpecimenPick = thirdSpecimenPut.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(38, -49));

        TrajectoryActionBuilder fourSpecimenPut = fourSpecimenPick.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(6, -25, Math.toRadians(270)), Math.toRadians(90));


        TrajectoryActionBuilder park = fourSpecimenPut.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(50, -55), Math.toRadians(270));

        /*TrajectoryActionBuilder fiveSpecimenPick = fourSpecimenPut.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(34.5, -52), Math.toRadians(270))
                ;

        TrajectoryActionBuilder fiveSpecimenPut = fiveSpecimenPick.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(6, -28), Math.toRadians(270))
                ;
*/

        waitForStart();

        Actions.runBlocking(new ParallelAction(
                arm.armUpdate(),

                new SequentialAction(
                        new ParallelAction(
                                firstSpecimenPut.build(),
                                arm.armSpecimen(),
                                etesito.wristSpecimen(),
                                etesito.servosClimbing()
                        ),
                        new SleepAction(0.05),

                        arm.rodeSpecimen(),
                        new SleepAction(0.2),
                        arm.armPostSpecimen(),
                        new SleepAction(0.2),
                        etesito.dropSpecimenAction(),
                        new SleepAction(0.2),
                        arm.rodeDown(),
                        new SleepAction(0.2),

                        //FIRST_SPECIMEN_PUT

                        new ParallelAction(
                                etesito.wristDown(),
                                arm.armDown(),
                                firstSampleMove1.build(),
                                etesito.pickSampleSlowAction(),
                                arm.rodeDown(),
                                etesito.pickSampleAction()

                                ),

                                arm.rodePickSampleSpecimen2(),

                        new SleepAction(0.5),

                        new ParallelAction(
                                etesito.pickSampleSlowAction(),
                                etesito.wristContract(),
                                firstSamplePut.build(),
                                arm.rodePutSample1()

                        ),
                        etesito.dropSampleAction(),
                        new SleepAction(0.25),

                        //MOVE_FIRST_TO_HUMAN

                        new ParallelAction(
                                etesito.wristDown(),
                                secondSampleMove1.build(),
                                etesito.pickSampleSlowAction(),
                                arm.rodeDown(),
                                etesito.pickSampleAction()
                                ),

                                arm.rodePickSampleSpecimen2(),

                        new SleepAction(0.5),

                        new ParallelAction(
                                etesito.pickSampleSlowAction(),
                                etesito.wristContract(),
                                secondSamplePut.build(),
                                arm.rodePutSample2()

                        ),
                        etesito.dropSampleAction(),
                        new SleepAction(0.25),


                        //SECOND_TOHUMAN


                        new ParallelAction(
                                thirdSampleMove1.build(),
                                etesito.pickSampleSlowAction(),
                                arm.rodeDown(),
                                etesito.wristDown(),
                                etesito.pickSampleAction()
                                ),

                        arm.rodePickSampleSpecimen1(),
                        new SleepAction(0.5),

                        new ParallelAction(
                                etesito.pickSampleSlowAction(),
                                etesito.wristContract(),
                                thirdSamplePut.build(),
                                arm.rodePutSample3()

                        ),
                        etesito.dropSampleAction(),
                        new SleepAction(0.25),

                        //THIRD_TO_HUMAN

                        new ParallelAction(
                                etesito.mantenerSampleAction(),
                                arm.rodeDown(),
                                etesito.pickSpecimenWrist(),
                                secondSpecimenPick1.build()
                        ),
                        etesito.pickSpecimenAction(),
                        new SleepAction(0.3), // TODO wait time to grab the specimen
                        arm.armSpecimen(),
                        new SleepAction(0.15), // TODO wait time to grab the specimen
                        new ParallelAction(
                                etesito.wristContract(),
                                secondSpecimenPut.build()
                        ),
                        etesito.wristSpecimen(),
                        new SleepAction(0.05),
                        arm.rodeSpecimen(),
                        new SleepAction(0.2),
                        arm.armPostSpecimen(),
                        new SleepAction(0.2),
                        etesito.dropSpecimenAction(),
                        new SleepAction(0.2),
                        arm.rodeDown(),
                        new SleepAction(0.2),

                        //SECOND_SPECIMENPUT

                        new ParallelAction(
                                etesito.pickSpecimenWrist(),
                                arm.armDown(),
                                thirdSpecimenPick1.build()
                        ),
                        arm.rodePickSpecimen(),
                        new SleepAction(0.15),

                        etesito.pickSpecimenAction(),
                        new SleepAction(0.3), // TODO wait time to grab the specimen
                        arm.armSpecimen(),
                        new SleepAction(0.15), // TODO wait time to grab the specimen
                        new ParallelAction(
                                etesito.wristContract(),
                                thirdSpecimenPut.build()
                        ),
                        etesito.wristSpecimen(),
                        new SleepAction(0.05),
                        arm.rodeSpecimen(),
                        new SleepAction(0.2),
                        arm.armPostSpecimen(),
                        new SleepAction(0.2),
                        etesito.dropSpecimenAction(),
                        new SleepAction(0.2),
                        arm.rodeDown(),
                        new SleepAction(0.2),

                        //THIRD_SPECIMENPUT

                        new ParallelAction(
                                etesito.pickSpecimenWrist(),
                                arm.armDown(),
                                fourSpecimenPick.build()
                        ),
                        arm.rodePickSpecimen(),
                        new SleepAction(0.15),

                        etesito.pickSpecimenAction(),

                        new SleepAction(0.3), // TODO wait time to grab the specimen
                        arm.armSpecimen(),
                        new SleepAction(0.15), // TODO wait time to grab the specimen
                        new ParallelAction(
                                etesito.wristContract(),
                                fourSpecimenPut.build()
                        ),
                        etesito.wristSpecimen(),
                        new SleepAction(0.05),
                        arm.rodeSpecimen(),
                        new SleepAction(0.2),
                        arm.armPostSpecimen(),
                        new SleepAction(0.2),
                        etesito.dropSpecimenAction(),
                        new SleepAction(0.2),
                        arm.rodeDown(),
                        new SleepAction(0.2),
                        //FOUR_SPECIMENPUT
                        arm.armDownLast(),
                        etesito.wristDown()




                ))


        );


    }
}