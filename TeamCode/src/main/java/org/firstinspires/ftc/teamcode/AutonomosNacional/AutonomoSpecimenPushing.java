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
public class AutonomoSpecimenPushing extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(10, -60, Math.toRadians(0));

        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        Etesito etesito = new Etesito();
        ArmSubsystem arm = new ArmSubsystem(hardwareMap);
        etesito.init(hardwareMap);

        ChassisSubsystem chassis = new ChassisSubsystem(hardwareMap);

        etesito.setLight("orange");

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
        }

        TrajectoryActionBuilder firstSpecimenPut = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(0, -24), Math.toRadians(270));

        TrajectoryActionBuilder moveSamples = firstSpecimenPut.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(37, -30), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(37, -5))

                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(46, -45), Math.toRadians(270))

                //FIRST_SAMPLE_MOVE

                .strafeToConstantHeading(new Vector2d(46, -5))

                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(54, -45), Math.toRadians(270))

                //SECOND_SAMPLE_PUT

                .strafeToLinearHeading(new Vector2d(54, -5), Math.toRadians(180))

                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(58, -45), Math.toRadians(270))

                ;

        TrajectoryActionBuilder secondSpecimenPick = moveSamples.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(37, -35), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(37, -55), Math.toRadians(270));

        TrajectoryActionBuilder secondSpecimenPut = secondSpecimenPick.endTrajectory().fresh()
                .setTangent(Math.toRadians(100))
                .splineToConstantHeading(new Vector2d(-8, -24), Math.toRadians(180));

        TrajectoryActionBuilder thirdSpecimenPick1 = secondSpecimenPut.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(34, -50));

        TrajectoryActionBuilder thirdSpecimenPut = thirdSpecimenPick1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-5, -24));

        TrajectoryActionBuilder fourSpecimenPick = thirdSpecimenPut.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(34, -50));

        TrajectoryActionBuilder fourSpecimenPut = fourSpecimenPick.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-2, -24));

        TrajectoryActionBuilder fiveSpecimenPick = fourSpecimenPut.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(34, -50));

        TrajectoryActionBuilder fiveSpecimenPut = fiveSpecimenPick.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-2, -24));

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
                                etesito.wristInit(),
                                arm.armInit(),
                                moveSamples.build()
                                ),

                        //THIRD_TO_HUMAN
                        etesito.pickSpecimenWrist(),
                        new SleepAction(0.1),

                        new ParallelAction(
                                arm.armDown(),
                                secondSpecimenPick.build()
                        ),
                        etesito.pickSpecimenAction(),
                        new SleepAction(0.3), // TODO wait time to grab the specimen
                        arm.armSpecimen(),
                        new SleepAction(0.15), // TODO wait time to grab the specimen
                        new ParallelAction(
                                etesito.wristSpecimen(),
                                secondSpecimenPut.build()
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

                        //SECOND_SPECIMENPUT

                        new ParallelAction(
                                etesito.pickSpecimenWrist(),
                                arm.armDown(),
                                thirdSpecimenPick1.build()
                        ),
                        new SleepAction(0.15), // TODO wait time to grab the specimen

                        etesito.pickSpecimenAction(),
                        new SleepAction(0.3), // TODO wait time to grab the specimen
                        arm.armSpecimen(),
                        new SleepAction(0.15), // TODO wait time to grab the specimen
                        new ParallelAction(
                                etesito.wristSpecimen(),
                                thirdSpecimenPut.build()
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

                        //THIRD_SPECIMENPUT

                        new ParallelAction(
                                etesito.pickSpecimenWrist(),
                                arm.armDown(),
                                fourSpecimenPick.build()
                        ), 
                        new SleepAction(0.10), // TODO wait time to grab the specimen

                        etesito.pickSpecimenAction(),
                        new SleepAction(0.3), // TODO wait time to grab the specimen
                        arm.armSpecimen(),
                        new SleepAction(0.15), // TODO wait time to grab the specimen
                        new ParallelAction(
                                etesito.wristSpecimen(),
                                fourSpecimenPut.build()
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

                        //FOUR_SPECIMENPUT

                        new ParallelAction(
                                etesito.pickSpecimenWrist(),
                                arm.armDown(),
                                fiveSpecimenPick.build()
                        ),
                        new SleepAction(0.1), // TODO wait time to grab the specimen

                        etesito.pickSpecimenAction(),
                        new SleepAction(0.3), // TODO wait time to grab the specimen
                        arm.armSpecimen(),
                        new SleepAction(0.15), // TODO wait time to grab the specimen
                        new ParallelAction(
                                etesito.wristSpecimen(),
                                fiveSpecimenPut.build()
                        ),
                        new SleepAction(0.05),
                        arm.rodeSpecimen(),
                        new SleepAction(0.2),
                        arm.armPostSpecimen(),
                        new SleepAction(0.2),
                        etesito.dropSpecimenAction(),
                        new SleepAction(0.2),
                        arm.rodeDown(),
                        new SleepAction(0.2)




                ))


        );


    }
}