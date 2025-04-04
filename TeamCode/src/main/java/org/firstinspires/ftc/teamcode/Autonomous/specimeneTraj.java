package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.PedroSubsystem;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous
public class specimeneTraj extends OpMode {

    private Follower follower;
    private Timer actionTimer;
    private Timer opmodeTimer;

    PedroSubsystem pedroSb;

    private Command pathCommand;

    //This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>

    private final Pose startPose = new Pose(8.5, 60, Math.toRadians(0));

    private final Pose putSpecimen1 = new Pose(23, 60, Math.toRadians(0));

    private final Pose pickUp1SamplePose = new Pose(36, 38, Math.toRadians(320));

    private final Pose put1SamplePose = new Pose(25, 33, Math.toRadians(230));

    private final Pose pickUp2SamplePose = new Pose(36, 28, Math.toRadians(320));

    private final Pose put2SamplePose = new Pose(25, 28 , Math.toRadians(230));

    private final Pose pickUp3SamplePose = new Pose(36, 18, Math.toRadians(320));

    private final Pose pick2specimenyput3sample = new Pose(12, 24, Math.toRadians(180));

    private final Pose put2specimen = new Pose(41, 75, Math.toRadians(180));

    private final Pose pick3specimen = new Pose(12, 24, Math.toRadians(180));

    private final Pose put3specimen = new Pose(41, 72, Math.toRadians(180));

    private final Pose pick4specimen = new Pose(12, 24, Math.toRadians(180));

    private final Pose put4specimen = new Pose(41, 69, Math.toRadians(180));

    private final Pose pick5specimen = new Pose(12, 24, Math.toRadians(180));

    private final Pose put5specimen = new Pose(41, 66, Math.toRadians(180));

    private Path PutSpecimen1, PickSample1, pickSpecimen2, PutSample1, PickSample2, PutSample2, PickSample3, PutSample3,
            PutSpecimen2, PickSpecimen3, PutSpecimen3, PickSpecimen4, PutSpecimen4, pickSpecimen5, PutSpecimen5;

    public void buildPaths() {
        PutSpecimen1 = new Path(new BezierLine(new Point(startPose), new Point(putSpecimen1)));
        PutSpecimen1.setLinearHeadingInterpolation(startPose.getHeading(), putSpecimen1.getHeading());

        PickSample1 = new Path(new BezierLine(new Point(putSpecimen1), new Point(pickUp1SamplePose)));
        PickSample1.setLinearHeadingInterpolation(putSpecimen1.getHeading(), pickUp1SamplePose.getHeading());

        PutSample1 = new Path(new BezierLine(new Point(pickUp1SamplePose), new Point(put1SamplePose)));
        PutSample1.setLinearHeadingInterpolation(pickUp1SamplePose.getHeading(), put1SamplePose.getHeading());

        PickSample2 = new Path(new BezierLine(new Point(put1SamplePose), new Point(pickUp2SamplePose)));
        PickSample2.setLinearHeadingInterpolation(put1SamplePose.getHeading(), pickUp2SamplePose.getHeading());

        PutSample2 = new Path(new BezierLine(new Point(pickUp2SamplePose), new Point(put2SamplePose)));
        PutSample2.setLinearHeadingInterpolation(pickUp2SamplePose.getHeading(), put2SamplePose.getHeading());

        PickSample3 = new Path(new BezierLine(new Point(put2SamplePose), new Point(pickUp3SamplePose)));
        PickSample3.setLinearHeadingInterpolation(put2SamplePose.getHeading(), pickUp3SamplePose.getHeading());

        PutSample3 = new Path(new BezierLine(new Point(pickUp3SamplePose), new Point(pick2specimenyput3sample)));
        PutSample3.setLinearHeadingInterpolation(pickUp3SamplePose.getHeading(), pick2specimenyput3sample.getHeading());

        PutSpecimen2 = new Path(new BezierLine(new Point(pick2specimenyput3sample), new Point(put2specimen)));
        PutSpecimen2.setConstantHeadingInterpolation(put2specimen.getHeading());

        PickSpecimen3 = new Path(new BezierLine(new Point(put2specimen), new Point(pick3specimen)));
        PickSpecimen3.setConstantHeadingInterpolation(pick3specimen.getHeading());

        PutSpecimen3 = new Path(new BezierLine(new Point(pick3specimen), new Point(put3specimen)));
        PutSpecimen3.setConstantHeadingInterpolation(put3specimen.getHeading());

        PickSpecimen4 = new Path(new BezierLine(new Point(put3specimen), new Point(pick4specimen)));
        PickSpecimen4.setConstantHeadingInterpolation(pick4specimen.getHeading());

        PutSpecimen4 = new Path(new BezierLine(new Point(pick4specimen), new Point(put4specimen)));
        PutSpecimen4.setConstantHeadingInterpolation(put4specimen.getHeading());

        pickSpecimen5 = new Path(new BezierLine(new Point(put4specimen), new Point(pick5specimen)));
        pickSpecimen5.setConstantHeadingInterpolation(pick5specimen.getHeading());

        PutSpecimen5 = new Path(new BezierLine(new Point(pick5specimen), new Point(put5specimen)));
        PutSpecimen5.setConstantHeadingInterpolation(put5specimen.getHeading());

        pathCommand = new SequentialCommandGroup(
            pedroSb.followPathCmd(PutSpecimen1),
                pedroSb.followPathCmd(PickSample1),
                pedroSb.followPathCmd(PutSample2)

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