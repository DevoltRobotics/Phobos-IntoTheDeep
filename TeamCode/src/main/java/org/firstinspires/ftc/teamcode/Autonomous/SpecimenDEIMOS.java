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


@Autonomous
public class SpecimenDEIMOS extends LinearOpMode {

    boolean initialized = false;

    @Override
    public void runOpMode() throws InterruptedException{

        //Pose2d initialPose = new Pose2d(10, -60, 0);

        Pose2d initialPose = new Pose2d(15, -60, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        ClimbServos climbServos = new ClimbServos(hardwareMap);

        TrajectoryActionBuilder firstSpecimenPut = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(15, -24), Math.toRadians(270))
                ;

        TrajectoryActionBuilder firstSpecimenAccommodate = firstSpecimenPut.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(15, -28), Math.toRadians(270))
                ;

        TrajectoryActionBuilder secondSpecimenMove = firstSpecimenAccommodate.endTrajectory().fresh()
                .setTangent(Math.toRadians(60))
                .strafeToLinearHeading(new Vector2d(48, -38), 0)
                .strafeToLinearHeading(new Vector2d(48, 26), 0)
                .setTangent(Math.toRadians(320))
                .splineToLinearHeading(new Pose2d(56, -42, 0), Math.toRadians(270))
                ;

        TrajectoryActionBuilder secondSpecimenPick1 = secondSpecimenMove.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(48, -36), Math.toRadians(268))
                ;

        TrajectoryActionBuilder secondSpecimenPick2 = secondSpecimenPick1.endTrajectory().fresh()
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(48, -48), Math.toRadians(268))
                ;

        TrajectoryActionBuilder secondSpecimenPut = secondSpecimenPick2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-20, -24), Math.toRadians(268))
                ;

        TrajectoryActionBuilder secondSpecimenAccomodate = secondSpecimenPut.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-20, -20), Math.toRadians(268))
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
                                new SleepAction(6),
                                arm.armSpecimen(),
                                new ParallelAction(
                                firstSpecimenPut.build(),
                                        arm.rodeCompens(),
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
                                new SleepAction(4),
                                secondSpecimenPick2.build(),
                                claw.pick(),
                                new SleepAction(0.2),
                                wrist.wristSpecimen(),
                                arm.armSpecimen(),
                                new ParallelAction(
                                        secondSpecimenPut.build(),
                                        arm.rodeCompens()
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
                                        arm.armDown(),
                                        new SleepAction(0.5)

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


