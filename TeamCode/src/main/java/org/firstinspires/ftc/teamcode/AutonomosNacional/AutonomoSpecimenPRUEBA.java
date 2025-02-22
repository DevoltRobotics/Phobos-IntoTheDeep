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
public class AutonomoSpecimenPRUEBA extends LinearOpMode {
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
                etesito.wristContract(),
                new SleepAction(0.1),
                arm.armInit(),
                new SleepAction(0.66),
                etesito.wristInit(),
                etesito.servosInit()

        ));

        while (opModeInInit()) {
            arm.ArmUpdateVoid();
        }

        TrajectoryActionBuilder firstSpecimenPut = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-8, -28), Math.toRadians(270));

        TrajectoryActionBuilder firstSampleMove1 = firstSpecimenPut.endTrajectory().fresh()
                .setTangent(Math.toRadians(260))
                .splineToLinearHeading(new Pose2d(32.5, -25, Math.toRadians(356)), Math.toRadians(70));

        TrajectoryActionBuilder firstSampleMove2 = firstSampleMove1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(43, -34), Math.toRadians(290));

        TrajectoryActionBuilder secondSampleMove1 = firstSampleMove2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(43, -24), Math.toRadians(359));

        TrajectoryActionBuilder secondSampleMove2 = secondSampleMove1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(35, -40), Math.toRadians(290));

        /*TrajectoryActionBuilder thirdSampleMove1 = secondSampleMove2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(52, -24), Math.toRadians(0));

        TrajectoryActionBuilder thirdSampleMove2 = thirdSampleMove1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(35, -40), Math.toRadians(300));
*/
        TrajectoryActionBuilder secondSpecimenPick1 = secondSampleMove2.endTrajectory().fresh()
                .turnTo(Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(35, -55), Math.toRadians(270));

        TrajectoryActionBuilder secondSpecimenPut = secondSpecimenPick1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-5, -28))
                .strafeToLinearHeading(new Vector2d(-5, -23.3), Math.toRadians(270));

        TrajectoryActionBuilder thirdSpecimenPick1 = secondSpecimenPut.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(34, -50), Math.toRadians(270));

        TrajectoryActionBuilder thirdSpecimenPut = thirdSpecimenPick1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(2, -35))
                .splineToSplineHeading(new Pose2d(2, -22.3, Math.toRadians(270)), Math.toRadians(180));

        TrajectoryActionBuilder fourSpecimenPick = thirdSpecimenPut.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(34, -50), Math.toRadians(270));

        TrajectoryActionBuilder fourSpecimenPut = fourSpecimenPick.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(8, -25, Math.toRadians(270)), Math.toRadians(90));


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
                                arm.armSpecimenPrueba1(),
                                etesito.wristDownM(),
                                etesito.servosClimbing()
                        ),
                        new SleepAction(0.1),

                        arm.rodeSpecimenPrueba(),
                        new SleepAction(0.3),
                        arm.armPostSpecimen(),
                        new SleepAction(0.4),
                        etesito.dropSpecimenAction(),
                        new SleepAction(0.2),
                        arm.rodeDown(),
                        new SleepAction(0.2),

                        //FIRST_SPECIMEN_PUT

                        new ParallelAction(
                                etesito.wristDown(),
                                arm.armDown(),
                                firstSampleMove1.build(),
                                etesito.pickSampleSlowAction()
                        ),

                        new ParallelAction(
                                arm.rodePickSample(),
                                etesito.pickSampleAction()
                        ),
                        new SleepAction(0.05),

                        new ParallelAction(
                                etesito.pickSampleSlowAction(),
                                etesito.wristContract()
                        ),
                        new ParallelAction(
                                firstSampleMove2.build(),
                                arm.rodePutSample1()

                        ),
                        etesito.dropSampleAction(),
                        new SleepAction(0.2),

                        //MOVE_FIRST_TO_HUMAN

                        new ParallelAction(
                                etesito.wristDown(),
                                arm.rodeDown()
                        ),
                        new SleepAction(0.1),
                        secondSampleMove1.build(),


                        new ParallelAction(
                                arm.rodePickSample(),
                                etesito.pickSampleAction()
                        ),
                        new SleepAction(0.05),

                        new ParallelAction(
                                etesito.pickSampleSlowAction(),
                                etesito.wristContract()
                        ),
                        new SleepAction(0.05),
                        new ParallelAction(
                                secondSampleMove2.build(),
                                arm.rodePutSample2()
                        ),

                        etesito.dropSampleAction(),
                        new SleepAction(0.2),

                        //SECOND_TOHUMAN

                        new ParallelAction(
                                etesito.mantenerSampleAction(),
                                arm.rodSemiDown(),
                                etesito.pickSpecimenWrist(),
                                secondSpecimenPick1.build()
                        ),
                        chassis.moveChassis(),
                        etesito.pickSpecimenAction(),
                        new SleepAction(0.45), // TODO wait time to grab the specimen
                        new ParallelAction(
                                arm.armSpecimenPrueba2(),
                                etesito.wristDownM(),
                                secondSpecimenPut.build()
                        ),
                        new SleepAction(0.05),
                        arm.rodeSpecimenPrueba(),
                        new SleepAction(0.3),
                        arm.armPostSpecimen(),
                        new SleepAction(0.4),
                        etesito.dropSpecimenAction(),
                        new SleepAction(0.2),
                        arm.rodSemiDown(),
                        new SleepAction(0.2),

                        //SECOND_SPECIMENPUT

                        new ParallelAction(
                                etesito.pickSpecimenWrist(),
                                arm.armDown(),
                                thirdSpecimenPick1.build()
                        ),
                        chassis.moveChassis(),
                        new SleepAction(0.15), // TODO wait time to grab the specimen

                        etesito.pickSpecimenAction(),
                        new SleepAction(0.45), // TODO wait time to grab the specimen
                        new ParallelAction(
                                arm.armSpecimenPrueba2(),
                                etesito.wristDownM(),
                                thirdSpecimenPut.build()
                        ),
                        new SleepAction(0.05),

                        arm.rodeSpecimenPrueba(),
                        new SleepAction(0.3),
                        arm.armPostSpecimen(),
                        new SleepAction(0.4),
                        etesito.dropSpecimenAction(),
                        new SleepAction(0.2),
                        arm.rodSemiDown(),
                        new SleepAction(0.2),

                        //THIRD_SPECIMENPUT

                        new ParallelAction(
                                etesito.pickSpecimenWrist(),
                                arm.armDown(),
                                fourSpecimenPick.build()
                        ),
                        chassis.moveChassis(),

                        new SleepAction(0.10), // TODO wait time to grab the specimen

                        etesito.pickSpecimenAction(),
                        new SleepAction(0.45), // TODO wait time to grab the specimen
                        new ParallelAction(
                                arm.armSpecimenPrueba2(),
                                etesito.wristDownM(),
                                fourSpecimenPut.build()
                        ),
                        new SleepAction(0.05),
                        arm.rodeSpecimenPrueba(),
                        new SleepAction(0.3),
                        arm.armPostSpecimen(),
                        new SleepAction(0.4),
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
