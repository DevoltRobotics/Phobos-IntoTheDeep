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
                .strafeToLinearHeading(new Vector2d(15, -26.5), Math.toRadians(273))
                ;

        TrajectoryActionBuilder firstSpecimenAccommodate = firstSpecimenPut.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(15, -27), Math.toRadians(273))
                ;

        TrajectoryActionBuilder secondSpecimenMove = firstSpecimenAccommodate.endTrajectory().fresh()
                .setTangent(Math.toRadians(50))
                .splineToLinearHeading(new Pose2d(37.5, -38, 0), Math.toRadians(90))
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(42, 19, 0), Math.toRadians(70))
                .setTangent(Math.toRadians(290))
                .splineToLinearHeading(new Pose2d(56, -36, 0), Math.toRadians(270))

                ;

        TrajectoryActionBuilder ThirdSpecimenMove = secondSpecimenMove.endTrajectory().fresh()
                .setTangent(Math.toRadians(110))
                .strafeToLinearHeading(new Vector2d(46, 22), 0)
                .setTangent(Math.toRadians(290))
                .splineToLinearHeading(new Pose2d(61, -36, 0), Math.toRadians(270))
                ;

        TrajectoryActionBuilder secondSpecimenPick1 = ThirdSpecimenMove.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(34, -27), Math.toRadians(270))
                ;

        TrajectoryActionBuilder secondSpecimenPick2 = secondSpecimenPick1.endTrajectory().fresh()
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(35, -42), Math.toRadians(275))
                ;

        TrajectoryActionBuilder secondSpecimenPut = secondSpecimenPick1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(0, -28), Math.toRadians(275))
                ;

        TrajectoryActionBuilder secondSpecimenAccomodate = secondSpecimenPut.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(0, -27), Math.toRadians(275))
                ;

        TrajectoryActionBuilder thirdSpecimenPick = secondSpecimenAccomodate.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(56, -35), Math.toRadians(275))
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(56, -43), Math.toRadians(275))
                ;

        TrajectoryActionBuilder thirdSpecimenPut = thirdSpecimenPick.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(-12, -26.5), Math.toRadians(275))

                ;

        TrajectoryActionBuilder thirdSpecimenAccommodate = thirdSpecimenPut .endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-12, -28), Math.toRadians(275))
                ;

        TrajectoryActionBuilder park = thirdSpecimenAccommodate .endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(32, -52), 0)
                ;


        Actions.runBlocking(new SequentialAction(
                claw.pick(),
                new SleepAction(0.8),
                arm.armInit(),
                new SleepAction(1.2),
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
                new SleepAction(0.3),
                arm.rodeSpecimen(),
                new SleepAction(0.6),
                firstSpecimenAccommodate.build(),
                claw.drop(),
                new SleepAction(0.2),
                wrist.wristInit(),
                new SleepAction(0.3),
                arm.rodeDown(),
                new SleepAction(0.8),
                arm.armInit(),
                                //FIRST_PUT

                                secondSpecimenMove.build(),
                                ThirdSpecimenMove.build(),
                                new ParallelAction(
                                        secondSpecimenPick1.build(),
                                        wrist.wristMedium()
                                        ),
                                arm.armDown(),
                                secondSpecimenPick2.build()

                                //MOVE_TO_HUMAN 

                                /*
                                claw.pick(),
                                new SleepAction(0.2),
                                arm.armSpecimen(),
                                new ParallelAction(
                                        secondSpecimenPut.build(),
                                        wrist.wristSpecimen()
                                        ),
                                new SleepAction(0.3),
                                arm.rodeSpecimen(),
                                new SleepAction(0.6),
                                secondSpecimenAccomodate.build(),
                                claw.drop(),
                                new SleepAction(0.2),
                                wrist.wristDownM(),
                                new SleepAction(0.3),
                                arm.rodeDown(),
                                new SleepAction(0.8),
                                arm.armDown(),

                //SECOND_PUT


                                new ParallelAction(
                                        wrist.wristMedium(),
                                        thirdSpecimenPick.build()

                                        ),
                                new SleepAction(0.1),
                                claw.pick(),
                                new SleepAction(0.2),
                                wrist.wristSpecimen(),
                                new SleepAction(0.4),
                                arm.armSpecimen(),
                                thirdSpecimenPut.build(),
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


