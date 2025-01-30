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
import org.firstinspires.ftc.teamcode.Comands.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.RR.PinpointDrive;

@Autonomous
public class PruebaAutonomoLL extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(10, -60, 0);

        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        Etesito etesito = new Etesito();
        ArmSubsystem arm = new ArmSubsystem(hardwareMap);
        LimelightSubsystem limeLight = new LimelightSubsystem(hardwareMap);

        etesito.init(hardwareMap);

        TrajectoryActionBuilder numero1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(10, -55), Math.toRadians(0))
                ;
        TrajectoryActionBuilder firstSpecimenPut = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-6, -27), Math.toRadians(270))
                ;

        TrajectoryActionBuilder firstSampleMove1 = firstSpecimenPut.endTrajectory().fresh()
                .setTangent(Math.toRadians(260))
                .splineToLinearHeading(new Pose2d(30, -24, Math.toRadians(356)), Math.toRadians(70))
                ;

        TrajectoryActionBuilder firstSampleMove2 = firstSampleMove1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(43, -32), Math.toRadians(290))
                ;

        TrajectoryActionBuilder secondSampleMove1 = firstSampleMove2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(45, -25), Math.toRadians(357))
                ;

        TrajectoryActionBuilder secondSampleMove2 = secondSampleMove1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(52, -40), Math.toRadians(290))
                ;

        TrajectoryActionBuilder thirdSampleMove1 = secondSampleMove2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(55, -25), Math.toRadians(0))
                ;

        TrajectoryActionBuilder thirdSampleMove2 = thirdSampleMove1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(35, -40), Math.toRadians(-60))
                ;

        TrajectoryActionBuilder secondSpecimenPick = thirdSampleMove2.endTrajectory().fresh()
                .turnTo(Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(35.5, -57), Math.toRadians(270))
                ;

        TrajectoryActionBuilder secondSpecimenPut = secondSpecimenPick.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-5, -28), Math.toRadians(270))
                ;

        TrajectoryActionBuilder thirdSpecimenPick = secondSpecimenPut.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(35.5, -57), Math.toRadians(270))
                //.setTangent(Math.toRadians(270))
                //.splineToLinearHeading(new Pose2d(33, -57.5, Math.toRadians(270)), Math.toRadians(290))

                ;

        TrajectoryActionBuilder thirdSpecimenPut = thirdSpecimenPick.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(0, -28), Math.toRadians(270))
                ;

        TrajectoryActionBuilder fourSpecimenPick = thirdSpecimenPut.endTrajectory().fresh()

                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(32.5, -57.5, Math.toRadians(270)), Math.toRadians(290))

                ;

        TrajectoryActionBuilder fourSpecimenPut = fourSpecimenPick.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(2, -28), Math.toRadians(270))
                ;

        TrajectoryActionBuilder fiveSpecimenPick = fourSpecimenPut.endTrajectory().fresh()

                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(40, -54, Math.toRadians(270)), Math.toRadians(270))
                ;

        TrajectoryActionBuilder fiveSpecimenPut = fiveSpecimenPick.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(6, -28), Math.toRadians(270))
                ;

        waitForStart();

        Actions.runBlocking(new ParallelAction(
                arm.armUpdate(),
        new SequentialAction(
                arm.armSemiDown(),
                        etesito.wristContract(),
                        new SleepAction(0.3),
                        numero1.build(),
                        limeLight.trackear()



                        ))
        


        );


    }


}
