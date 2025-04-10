package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
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

import org.firstinspires.ftc.teamcode.Comands.Etesito;
import org.firstinspires.ftc.teamcode.subsystems.PedroSb;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous
public class specimeneTraj extends OpMode {

    private Follower follower;
    private Timer actionTimer;
    private Timer opmodeTimer;

    Etesito etesito = new Etesito();

    PedroSb pedroSb;

    private Command pathCommand;

    //This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>

    private final Pose startPose = new Pose(8.5, 62, Math.toRadians(0));

    private final Pose putSpecimen1Pose = new Pose(23, 62, Math.toRadians(0));

    private final Pose pickUpSample1Pose = new Pose(32, 42, Math.toRadians(320));

    private final Pose putSample1Pose = new Pose(27, 34, Math.toRadians(230));

    private final Pose pickUpSample2Pose = new Pose(30, 32, Math.toRadians(320));

    private final Pose putSample2Pose = new Pose(27, 32, Math.toRadians(230));

    private final Pose pickUpSample3Pose = new Pose(30, 22, Math.toRadians(320));

    private final Pose pickSpecimen2yPutSample3Pose = new Pose(16, 34, Math.toRadians(180));
    private final Pose pickSpecimen2yPutSample3ControlPose = new Pose(34, 28, Math.toRadians(180));

    private final Pose putSpecimen2Pose = new Pose(41, 75, Math.toRadians(180));

    private final Pose pickSpecimen3Pose = new Pose(12, 28, Math.toRadians(180));

    private final Pose putSpecimen3Pose = new Pose(41, 72, Math.toRadians(180));

    private final Pose pickSpecimen4Pose = new Pose(12, 28, Math.toRadians(180));

    private final Pose putSpecimen4Pose = new Pose(41, 69, Math.toRadians(180));

    private final Pose pickSpecimen5Pose = new Pose(12, 28, Math.toRadians(180));

    private final Pose putSpecimen5Pose = new Pose(41, 66, Math.toRadians(180));

    private Path PutSpecimen1;

    private PathChain PickSample1, PutSample1, PickSample2, PutSample2, PickSample3, PickSpecimen2yPutSample3,
            PutSpecimen2, PickSpecimen3, PutSpecimen3, PickSpecimen4, PutSpecimen4, PickSpecimen5, PutSpecimen5;

    public void buildPaths() {
        PutSpecimen1 = new Path(new BezierLine(new Point(startPose), new Point(putSpecimen1Pose)));
        PutSpecimen1.setConstantHeadingInterpolation(putSpecimen1Pose.getHeading());


        PickSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(putSpecimen1Pose), new Point(pickUpSample1Pose)))
                .setLinearHeadingInterpolation(putSpecimen1Pose.getHeading(), pickUpSample1Pose.getHeading())
                .build();

        PutSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUpSample1Pose), new Point(putSample1Pose)))
                .setLinearHeadingInterpolation(pickUpSample1Pose.getHeading(), putSample1Pose.getHeading())
                .build();

        PickSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(putSample1Pose), new Point(pickUpSample2Pose)))
                .setLinearHeadingInterpolation(putSample1Pose.getHeading(), pickUpSample2Pose.getHeading())
                .build();

        PutSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUpSample2Pose), new Point(putSample2Pose)))
                .setLinearHeadingInterpolation(pickUpSample2Pose.getHeading(), putSample2Pose.getHeading())
                .build();

        PickSample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(putSample2Pose), new Point(pickUpSample3Pose)))
                .setLinearHeadingInterpolation(putSample2Pose.getHeading(), pickUpSample3Pose.getHeading())
                .build();

        PickSpecimen2yPutSample3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickUpSample3Pose), new Point(pickSpecimen2yPutSample3Pose)))
                .setLinearHeadingInterpolation(pickUpSample3Pose.getHeading(), pickSpecimen2yPutSample3Pose.getHeading())
                .build();

        PutSpecimen2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickSpecimen2yPutSample3Pose), new Point(putSpecimen2Pose)))
                .setConstantHeadingInterpolation(putSpecimen2Pose.getHeading())
                .build();

        PickSpecimen3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(putSpecimen2Pose), new Point(pickSpecimen3Pose)))
                .setConstantHeadingInterpolation(pickSpecimen3Pose.getHeading())
                .build();

        PutSpecimen3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickSpecimen3Pose), new Point(putSpecimen3Pose)))
                .setConstantHeadingInterpolation(putSpecimen3Pose.getHeading())
                .build();

        PickSpecimen4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(putSpecimen3Pose), new Point(pickSpecimen4Pose)))
                .setConstantHeadingInterpolation(pickSpecimen4Pose.getHeading())
                .build();

        PutSpecimen4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickSpecimen4Pose), new Point(putSpecimen4Pose)))
                .setConstantHeadingInterpolation(putSpecimen4Pose.getHeading())
                .build();

        PickSpecimen5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(putSpecimen4Pose), new Point(pickSpecimen5Pose)))
                .setConstantHeadingInterpolation(pickSpecimen5Pose.getHeading())
                .build();

        PutSpecimen5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickSpecimen5Pose), new Point(putSpecimen5Pose)))
                .setConstantHeadingInterpolation(putSpecimen5Pose.getHeading())
                .build();

        pathCommand = new SequentialCommandGroup(
                pedroSb.followPathCmd(PutSpecimen1),
                new WaitCommand(300),
                pedroSb.followPathCmd(PickSample1),
                new WaitCommand(300),

                pedroSb.followPathCmd(PutSample1),
                new WaitCommand(300),

                pedroSb.followPathCmd(PickSample2),
                new WaitCommand(300),

                pedroSb.followPathCmd(PutSample2),
                new WaitCommand(300),

                pedroSb.followPathCmd(PickSample3),
                new WaitCommand(300),

                pedroSb.followPathCmd(PickSpecimen2yPutSample3),
                new WaitCommand(300)

                /*
                pedroSb.followPathCmd(PutSpecimen2),

                pedroSb.followPathCmd(PickSpecimen3),
                pedroSb.followPathCmd(PutSpecimen3),

                pedroSb.followPathCmd(PickSpecimen4),
                pedroSb.followPathCmd(PutSpecimen4),

                pedroSb.followPathCmd(PickSpecimen5),
                pedroSb.followPathCmd(PutSpecimen5)

                 */

        );
    }

    @Override
    public void init() {
        etesito.init(hardwareMap,true, true);

        etesito.wristDown();

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        pedroSb = new PedroSb(follower);
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