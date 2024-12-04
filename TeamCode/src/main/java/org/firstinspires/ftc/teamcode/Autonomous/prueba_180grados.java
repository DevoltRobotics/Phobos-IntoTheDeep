package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
public class prueba_180grados extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{


        Pose2d initialPose = new Pose2d(10, -60, Math.abs(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);

        ClimbServos climbServos = new ClimbServos(hardwareMap);

        TrajectoryActionBuilder girar = drive.actionBuilder(initialPose)
                .turn(Math.toRadians(180))
                ;

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                        girar.build()


                )

        );


    }

}

