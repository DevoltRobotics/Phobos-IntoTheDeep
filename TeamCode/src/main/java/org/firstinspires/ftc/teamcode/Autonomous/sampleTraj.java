package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Comands.Etesito;
import org.firstinspires.ftc.teamcode.subsystems.PedroSubsystem;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous
public class sampleTraj extends OpMode {

    private Follower follower;
    private Timer actionTimer;
    private Timer opmodeTimer;

    PedroSubsystem pedroSb;

    private Command pathCommand;

    //This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>

    private final Pose startPose = new Pose(8.5, 112, Math.toRadians(0));

    private final Pose putSample1Pose = new Pose(15, 130, Math.toRadians(315));
    private final Pose putSample1ControlPose = new Pose(19, 112, Math.toRadians(340));

    private final Pose pickUp2SamplePose = new Pose(30, 115, Math.toRadians(50));

    private final Pose move2SamplePose = new Pose(pickUp2SamplePose.getX(), pickUp2SamplePose.getY(), Math.toRadians(325));

    private final Pose putSample2Pose = new Pose(15, 130, Math.toRadians(315));

    private final Pose pickUp3SamplePose = new Pose(28, 125, Math.toRadians(50));

    private final Pose move3SamplePose = new Pose(pickUp3SamplePose.getX(), pickUp2SamplePose.getY(), Math.toRadians(35));

    private final Pose put3SamplePose = new Pose(15, 130, Math.toRadians(315));

    private final Pose pickUp4SamplePose = new Pose(38, 135, Math.toRadians(300));

    private final Pose move4SamplePose = new Pose(pickUp4SamplePose.getX(), pickUp4SamplePose.getY(), Math.toRadians(35));

    private final Pose put4SamplePose = new Pose(13, 131, Math.toRadians(315));

    private final Pose pickUp5SamplePose = new Pose(65, 105, Math.toRadians(270));
    private final Pose pickUp5SampleControlPose = new Pose(65, 120, Math.toRadians(275));

    private final Pose put5SamplePos = new Pose(12, 130, Math.toRadians(315));
    private final Pose put5SampleControlPose = new Pose(65, 120, Math.toRadians(275));

    private Path PutSample1, PutSample5;

    private PathChain PickSample2, PutSample2, PickSample3, PutSample3, PickSample4, PutSample4,
            PickSample5;


    public void buildPaths() {
        PutSample1 = new Path(new BezierCurve(new Point(startPose), new Point(putSample1ControlPose), new Point(putSample1Pose)));
        PutSample1.setLinearHeadingInterpolation(startPose.getHeading(), putSample1Pose.getHeading());


        PickSample2 = follower.pathBuilder()
               .addPath(new BezierLine(new Point(putSample1Pose), new Point(pickUp2SamplePose)))
               .setLinearHeadingInterpolation(putSample1Pose.getHeading(), pickUp2SamplePose.getHeading())
               .build();

        PutSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUp2SamplePose), new Point(putSample2Pose)))
                .setLinearHeadingInterpolation(pickUp2SamplePose.getHeading(), putSample2Pose.getHeading())
                .build();

        PickSample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(putSample2Pose), new Point(pickUp3SamplePose)))
                .setLinearHeadingInterpolation(putSample2Pose.getHeading(), pickUp3SamplePose.getHeading())
                .build();

        PutSample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUp3SamplePose), new Point(put3SamplePose)))
                .setLinearHeadingInterpolation(pickUp3SamplePose.getHeading(), put3SamplePose.getHeading())
                .build();

        PickSample4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(put3SamplePose), new Point(pickUp4SamplePose)))
                .setLinearHeadingInterpolation(put3SamplePose.getHeading(), pickUp4SamplePose.getHeading())
                .build();

        PutSample4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUp4SamplePose), new Point(put4SamplePose)))
                .setLinearHeadingInterpolation(pickUp4SamplePose.getHeading(), put4SamplePose.getHeading())
                .build();

        PickSample5 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(put4SamplePose), new Point(pickUp5SampleControlPose), new Point(pickUp5SamplePose)))
                .setLinearHeadingInterpolation(put4SamplePose.getHeading(), pickUp5SamplePose.getHeading())
                .build();

        PutSample5 = new Path(new BezierCurve(new Point(pickUp5SamplePose), new Point(put5SampleControlPose), new Point(put5SamplePos)));
        PutSample5.setLinearHeadingInterpolation(pickUp5SamplePose.getHeading(), put5SamplePos.getHeading());

        pathCommand = new SequentialCommandGroup(
            pedroSb.followPathCmd(PutSample1),
                pedroSb.followPathCmd(PickSample2),
                pedroSb.followPathCmd(PutSample2),
                pedroSb.followPathCmd(PickSample3),
                pedroSb.followPathCmd(PutSample3),
                pedroSb.followPathCmd(PickSample4),
                pedroSb.followPathCmd(PutSample4),
                pedroSb.followPathCmd(PickSample5)



        );
    }

    @Override
    public void init() {

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        pedroSb = new PedroSubsystem(follower);
        CommandScheduler.getInstance().registerSubsystem(pedroSb);

        buildPaths();
    }

    @Override
    public void init_loop() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathCommand.schedule();
    }

    @Override
    public void loop() {
        // These loop te movements of the robot

        CommandScheduler.getInstance().run();

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {
        pathCommand.cancel();
    }
}