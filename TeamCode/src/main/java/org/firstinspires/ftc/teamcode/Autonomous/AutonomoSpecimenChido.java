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
                .strafeToLinearHeading(new Vector2d(15, -25), Math.toRadians(270))
                ;

        TrajectoryActionBuilder firstSpecimenAccommodate = firstSpecimenPut.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(15, -30), Math.toRadians(270))
                ;

        TrajectoryActionBuilder secondSpecimenMove = firstSpecimenAccommodate.endTrajectory().fresh()
                .setTangent(Math.toRadians(60))
                .strafeToLinearHeading(new Vector2d(45, -38), 0)
                .strafeToLinearHeading(new Vector2d(45, 26), 0)
                .setTangent(Math.toRadians(320))
                .splineToLinearHeading(new Pose2d(47, -32, 0), Math.toRadians(270))

                ;

        TrajectoryActionBuilder secondSpecimenPick1 = secondSpecimenMove.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(35, -21), Math.toRadians(270))
                ;

        TrajectoryActionBuilder secondSpecimenPick2 = secondSpecimenPick1.endTrajectory().fresh()
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(35, -32), Math.toRadians(270))
                ;

        TrajectoryActionBuilder secondSpecimenPut = secondSpecimenPick2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-20, -7), Math.toRadians(270))
                ;

        TrajectoryActionBuilder secondSpecimenAccomodate = secondSpecimenPut.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-20, -9), Math.toRadians(270))
                ;

        TrajectoryActionBuilder thirdSpecimenPick1 = secondSpecimenAccomodate.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(24, -21), Math.toRadians(270))
                ;

        TrajectoryActionBuilder thirdSpecimenPick2 = thirdSpecimenPick1.endTrajectory().fresh()
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(24, -32), Math.toRadians(270))
                ;

        TrajectoryActionBuilder thirdSpecimenPut = thirdSpecimenPick2.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(-55, -6), Math.toRadians(270))

                ;

        TrajectoryActionBuilder thirdSpecimenAccommodate = thirdSpecimenPut .endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-55, -8), Math.toRadians(270))
                ;

        TrajectoryActionBuilder park = thirdSpecimenAccommodate .endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(10, -30), 0)
                ;


        Actions.runBlocking(new SequentialAction(
                claw.pick(),
                new SleepAction(0.8),
                arm.armInit(),
                new SleepAction(0.1),
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
                new SleepAction(0.2),
                arm.rodeSpecimen(),
                new SleepAction(0.6),
                                firstSpecimenAccommodate.build(),
                                wrist.wristInit(),
                                new SleepAction(0.1),
                                claw.drop(),
                new SleepAction(0.2),
                wrist.wristInit(),
                new SleepAction(0.3),
                arm.rodeDown(),
                new SleepAction(0.8),
                arm.armInit(),
                                //FIRST_PUT

                                secondSpecimenMove.build(),
                                new ParallelAction(
                                        secondSpecimenPick1.build(),
                                        wrist.wristMedium()
                                        ),
                                arm.armDown(),
                                new SleepAction(0.5),
                                secondSpecimenPick2.build(),
                                claw.pick(),
                                new SleepAction(0.2),
                                wrist.wristSpecimen(),
                                arm.armSpecimen(),
                                new ParallelAction(
                                        secondSpecimenPut.build()
                                        ),
                                new SleepAction(0.2),
                                arm.rodeSpecimen(),
                                new SleepAction(0.6),
                                secondSpecimenAccomodate.build(),
                                claw.drop(),
                                new SleepAction(0.2),
                                wrist.wristMedium(),
                                new SleepAction(0.1),
                                arm.rodeDown(),
                                new SleepAction(0.4),

                                //SECOND_PUT

                                new ParallelAction(
                                        arm.armDown(),
                                        thirdSpecimenPick1.build()

                                        ),
                                thirdSpecimenPick2.build(),
                                new SleepAction(0.1),
                                claw.pick(),
                                new SleepAction(0.2),
                                wrist.wristSpecimen(),
                                arm.armSpecimen(),

                                new ParallelAction(
                                        thirdSpecimenPut.build()

                                        ),
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
                                arm.armDown()

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


