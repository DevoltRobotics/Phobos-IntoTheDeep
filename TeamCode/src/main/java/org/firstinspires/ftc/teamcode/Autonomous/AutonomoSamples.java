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
public class AutonomoSamples extends LinearOpMode {

    boolean initialized = false;

    @Override
    public void runOpMode() throws InterruptedException{

        Pose2d initialPose = new Pose2d(-40, -60, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        ClimbServos climbServos = new ClimbServos(hardwareMap);

        TrajectoryActionBuilder firstSamplePut = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-64, -50), Math.toRadians(40))
                ;

        TrajectoryActionBuilder firstSampleAccommodate = firstSamplePut.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-66, -52), Math.toRadians(40))
                ;

        TrajectoryActionBuilder secondSamplePick = firstSampleAccommodate.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(-71, -27.5), Math.toRadians(95))
                ;

        TrajectoryActionBuilder secondSamplePut = secondSamplePick.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-67.5, -52), Math.toRadians(45))
                ;

        TrajectoryActionBuilder secondSampleAccommodate = secondSamplePut.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-70, -54), Math.toRadians(45))
                ;

        TrajectoryActionBuilder thirdSamplePick = secondSampleAccommodate.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-91, -30.5), Math.toRadians(95))
                ;

        TrajectoryActionBuilder thirdSamplePut = thirdSamplePick.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-69, -58), Math.toRadians(50))

                ;

        TrajectoryActionBuilder thirdSampleAccommodate = thirdSamplePut .endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-71, -60), Math.toRadians(50))
                ;

        TrajectoryActionBuilder park = thirdSampleAccommodate .endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-75, 10), Math.toRadians(15))
                ;


        Actions.runBlocking(new SequentialAction(
                claw.pick(),
                new SleepAction(0.8),
                arm.armInit(),
                new SleepAction(0.8),
                climbServos.servosInit(),
                wrist.wristInit()
                ));

        while (opModeInInit() ){
            arm.updateArm();
        }

        waitForStart();

        Actions.runBlocking(new ParallelAction(arm.armInPos(),
                        new SequentialAction(
                                arm.armUp(),
                                new SleepAction(0.1),
                                new ParallelAction(
                                firstSamplePut.build(),
                                        climbServos.servosUp(),
                                        wrist.wristUp(),
                                        arm.rodeUp()

                                        ),
                firstSampleAccommodate.build(),
                new SleepAction(0.1),
                claw.drop(),
                new SleepAction(0.6),
                wrist.wristDownM(),
                new SleepAction(0.3),
                arm.rodeDown(),
                new SleepAction(0.6),
                new ParallelAction(
                        arm.armDown(),
                        secondSamplePick.build()
                        ),
                //FIRST_PUT

                                new SleepAction(0.1),
                wrist.wristDown(),
                                new SleepAction(0.3),

                                claw.pick(),
                                new SleepAction(0.3),
                                new ParallelAction(
                                wrist.wristUp(),
                                arm.armUp()
                                ),
                                new SleepAction(0.3),

                                new ParallelAction(
                                secondSamplePut.build(),
                                arm.rodeUp()),

                new SleepAction(0.1),
                secondSampleAccommodate.build(),
                                new SleepAction(0.1),
                                claw.drop(),
                new SleepAction(0.6),
                wrist.wristDownM(),
                new SleepAction(0.3),
                arm.rodeDown(),
                new SleepAction(0.8),
                arm.armDown(),


                                /////SECOND_PUT
                                thirdSamplePick.build(),

                new SleepAction(0.1),
                wrist.wristDown(),
                new SleepAction(0.3),

                claw.pick(),
                new SleepAction(0.3),
                new ParallelAction(
                        wrist.wristUp(),
                        arm.armUp()
                ),
                new SleepAction(0.3),

                new ParallelAction(
                        thirdSamplePut.build(),
                        arm.rodeUp()),

                new SleepAction(0.3),
                thirdSampleAccommodate.build(),
                new SleepAction(0.1),
                claw.drop(),
                new SleepAction(0.6),
                wrist.wristDown(),
                new SleepAction(0.3),
                arm.rodeDown(),
                new SleepAction(0.6),
                wrist.wristDisabled(),
                arm.armDown(),
                                new SleepAction(0.4),
                                park.build()




                        )));





        }

}


