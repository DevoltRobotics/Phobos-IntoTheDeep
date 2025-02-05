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
public class TestTrackColor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));

        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        Etesito etesito = new Etesito();
        ArmSubsystem arm = new ArmSubsystem(hardwareMap);

        etesito.init(hardwareMap);

        TrajectoryActionBuilder firstSamplePut1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(2, 0), Math.toRadians(0))
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

        TrajectoryActionBuilder fiveSpecimenPick = fourSamplePut.endTrajectory().fresh()
                .setTangent(Math.toRadians(100))
                .splineToLinearHeading(new Pose2d(-40, -20, Math.toRadians(0)), Math.toRadians(60))

                ;

        waitForStart();

        Actions.runBlocking(new ParallelAction(
                arm.armUpdate(),
                new SequentialAction(
                        new ParallelAction(
                                firstSamplePut1.build(),
                                etesito.wristContract()
                        ),

                        arm.rodePickSubmersible(),
                        new SleepAction(0.5),
                        etesito.wristDown(),
                        arm.samplePickRedSM(),
                        etesito.wristContract(),
                        etesito.pickSampleSlowAction(),
                        new SleepAction(0.2),
                        arm.rodeDown()



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
