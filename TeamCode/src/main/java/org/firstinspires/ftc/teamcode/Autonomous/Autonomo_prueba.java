package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Comands.Arm;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;

@Autonomous
public class Autonomo_prueba extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(25, -58, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Arm arm = new Arm(hardwareMap);

        /*TrajectoryActionBuilder first_specimen = drive.actionBuilder(initialPose)
                .setTangent(0)
                .strafeToLinearHeading(new Vector2d(10, -33), Math.toRadians(270))
                //extend rode
                .strafeToLinearHeading(new Vector2d(10, -35), Math.toRadians(270));
                //open_claw || timer || wrist_down &&  && contract_rode && down_arm

                //arm_down && wrist_medium

                //FIRST_SPECIMEN

        TrajectoryActionBuilder second_specimen = first_specimen.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(32, -52), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(32, -60), Math.toRadians(270))
                //close_claw
                .strafeToLinearHeading(new Vector2d(5, -33), Math.toRadians(270))
                //arm && wrist_specimen
                .strafeToLinearHeading(new Vector2d(5, -35), Math.toRadians(270));
                //open_claw || timer || wrist_down &&  && contract_rode && down_arm

                //SECOND_SPECIMEN

        TrajectoryActionBuilder third_specimen = second_specimen.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(32, -52), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(32, -60), Math.toRadians(270))
                //close_claw
                .strafeToLinearHeading(new Vector2d(0, -33), Math.toRadians(270))
                //arm && wrist_specimen
                .strafeToLinearHeading(new Vector2d(0, -35), Math.toRadians(270));
                //open_claw || timer || wrist_down &&  && contract_rode && down_arm

                //THIRD_SPECIMEN

        TrajectoryActionBuilder estacionar = third_specimen.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(40, -58), Math.toRadians(90));

         */


        waitForStart();

        Actions.runBlocking(new SequentialAction(
                arm.armUp()
        ));




    }





}

