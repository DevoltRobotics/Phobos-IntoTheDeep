package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous
public class SpecimenTrajectory extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    //This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>

    private final Pose startPose = new Pose(8.5, 79.5, Math.toRadians(0));

    private final Pose putSpecimen1 = new Pose(25, 79, Math.toRadians(0));

    private final Pose pickup1SamplePose = new Pose(38, 39, Math.toRadians(300));

    private final Pose pickup1PoseControl = new Pose(22, 53, Math.toRadians(260));

    private final Pose put1SamplePose = new Pose(27, 37, Math.toRadians(260));

    private final Pose pickup2SamplePose = new Pose(38, 29, Math.toRadians(300));

    private final Pose put2SamplePose = new Pose(27, 27, Math.toRadians(260));

    private final Pose pickup3SamplePose = new Pose(30, 27, Math.toRadians(295));

    private final Pose pick2specimenyput3sample = new Pose(16, 35, Math.toRadians(180));

    private final Pose put2specimen = new Pose(41, 75, Math.toRadians(180));

    private final Pose pick3specimen = new Pose(10, 30, Math.toRadians(180));

    private final Pose put3specimen = new Pose(41, 72, Math.toRadians(180));

    private final Pose pick4specimen = new Pose(10, 30, Math.toRadians(180));

    private final Pose put4specimen = new Pose(41, 69, Math.toRadians(180));

    private final Pose pick5specimen = new Pose(10, 30, Math.toRadians(180));

    private final Pose put5specimen = new Pose(41, 66, Math.toRadians(180));

    private Path scorePreload, PickSample1, pickSpecimen2, PutSample1, PickSample2, PutSample2, PickSample3, PutSample3,
            putSpecimen2, pickSpecimen3, putSpecimen3, pickSpecimen4, putSpecimen4, pickSpecimen5, putSpecimen5;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(putSpecimen1)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), putSpecimen1.getHeading());

        PickSample1 = new Path(new BezierCurve(new Point(putSpecimen1), new Point(pickup1PoseControl), new Point(pickup1SamplePose)));
        PickSample1.setLinearHeadingInterpolation(putSpecimen1.getHeading(), pickup1SamplePose.getHeading());


        PutSample1 = new Path(new BezierLine(new Point(pickup1SamplePose), new Point(put1SamplePose)));
        PutSample1.setLinearHeadingInterpolation(pickup1SamplePose.getHeading(), put1SamplePose.getHeading());

        PickSample2 = new Path(new BezierLine(new Point(put1SamplePose), new Point(pickup2SamplePose)));
        PickSample2.setLinearHeadingInterpolation(put1SamplePose.getHeading(), pickup2SamplePose.getHeading());

        PutSample2 = new Path(new BezierLine(new Point(pickup2SamplePose), new Point(put2SamplePose)));
        PutSample2.setLinearHeadingInterpolation(pickup2SamplePose.getHeading(), put2SamplePose.getHeading());

        PickSample3 = new Path(new BezierLine(new Point(put2SamplePose), new Point(pickup3SamplePose)));
        PickSample3.setLinearHeadingInterpolation(put2SamplePose.getHeading(), pickup3SamplePose.getHeading());

        PutSample3 = new Path(new BezierLine(new Point(pickup3SamplePose), new Point(pick2specimenyput3sample)));
        PutSample3.setLinearHeadingInterpolation(pickup3SamplePose.getHeading(), pick2specimenyput3sample.getHeading());

        putSpecimen2 = new Path(new BezierLine(new Point(pick2specimenyput3sample), new Point(put2specimen)));
        putSpecimen2.setConstantHeadingInterpolation(put2specimen.getHeading());

        pickSpecimen3 = new Path(new BezierLine(new Point(put2specimen), new Point(pick3specimen)));
        pickSpecimen3.setConstantHeadingInterpolation(pick3specimen.getHeading());

        putSpecimen3 = new Path(new BezierLine(new Point(pick3specimen), new Point(put3specimen)));
        pickSpecimen3.setConstantHeadingInterpolation(put3specimen.getHeading());

        pickSpecimen4 = new Path(new BezierLine(new Point(put3specimen), new Point(pick4specimen)));
        pickSpecimen3.setConstantHeadingInterpolation(pick4specimen.getHeading());

        putSpecimen4 = new Path(new BezierLine(new Point(pick4specimen), new Point(put4specimen)));
        pickSpecimen3.setConstantHeadingInterpolation(put4specimen.getHeading());

        pickSpecimen5 = new Path(new BezierLine(new Point(put4specimen), new Point(pick5specimen)));
        pickSpecimen3.setConstantHeadingInterpolation(pick5specimen.getHeading());

        putSpecimen5 = new Path(new BezierLine(new Point(pick5specimen), new Point(put5specimen)));
        putSpecimen5.setConstantHeadingInterpolation(put5specimen.getHeading());


    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                     follower.followPath(PickSample1,true);
                    setPathState(2);
                }
                break;

            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(PutSample1,true);
                    setPathState(3);
                }
                break;



            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(PickSample2,true);
                    setPathState(4);
                }
                break;

            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(PutSample2,true);
                    setPathState(5);
                }
                break;


            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(PickSample3,true);
                    setPathState(6);
                }
                break;

            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(PutSample3, true);
                    setPathState(7);
                }
                break;

            case 7:
                if(!follower.isBusy()) {
                    setPathState(-1);
                }
                break;

        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        follower.setStartingPose(startPose);
        buildPaths();
    }
    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}