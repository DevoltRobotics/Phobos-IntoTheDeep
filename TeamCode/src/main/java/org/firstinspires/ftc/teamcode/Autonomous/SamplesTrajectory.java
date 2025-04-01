package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous
public class SamplesTrajectory extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    //This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>

    private final Pose startPose = new Pose(8.5, 112, Math.toRadians(0));

    private final Pose putSample1Pose = new Pose(12, 130, Math.toRadians(315));
    private final Pose putSample1ControlPose = new Pose(19, 112, Math.toRadians(340));

    private final Pose pickUp2SamplePose = new Pose(25, 130, Math.toRadians(340));

    private final Pose move2SamplePose = new Pose(25, 130, Math.toRadians(325));

    private final Pose putSample2Pose = new Pose(12, 130, Math.toRadians(315));

    private final Pose pickUp3SamplePose = new Pose(25, 120, Math.toRadians(25));

    private final Pose move3SamplePose = new Pose(25, 120, Math.toRadians(40));

    private final Pose put3SamplePose = new Pose(12, 130, Math.toRadians(315));

    private final Pose pickUp4SamplePose = new Pose(25, 130, Math.toRadians(25));

    private final Pose move4SamplePose = new Pose(25, 130, Math.toRadians(35));

    private final Pose put4SamplePose = new Pose(12, 130, Math.toRadians(315));

    private final Pose pickUp5SamplePose = new Pose(65, 100, Math.toRadians(270));
    private final Pose pickUp5SampleControlPose = new Pose(65, 120, Math.toRadians(275));

    private final Pose put5SamplePos = new Pose(12, 130, Math.toRadians(315));
    private final Pose put5SampleControlPose = new Pose(65, 120, Math.toRadians(275));

    private Path scorePreload, PickSample2, MoveSample2, PutSample2, PickSample3, MoveSample3, PutSample3, PickSample4, MoveSample4, PutSample4,
            PickSample5, PutSample5;
    private PathChain MoveSample2Chain;

    public void buildPaths() {
        scorePreload = new Path(new BezierCurve(new Point(startPose), new Point(putSample1ControlPose), new Point(putSample1Pose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), putSample1Pose.getHeading());

        PickSample2 = new Path(new BezierLine(new Point(putSample1Pose), new Point(pickUp2SamplePose)));
        PickSample2.setLinearHeadingInterpolation(putSample1Pose.getHeading(), pickUp2SamplePose.getHeading());

        MoveSample2 = new Path(new BezierLine(new Point(pickUp2SamplePose), new Point(move2SamplePose)));
        MoveSample2.setLinearHeadingInterpolation(pickUp2SamplePose.getHeading(), move2SamplePose.getHeading());

        MoveSample2Chain = new PathChain(PickSample2, MoveSample2);

        PutSample2 = new Path(new BezierLine(new Point(move2SamplePose), new Point(putSample2Pose)));
        PutSample2.setLinearHeadingInterpolation(move2SamplePose.getHeading(), putSample2Pose.getHeading());

        PickSample3 = new Path(new BezierLine(new Point(putSample2Pose), new Point(pickUp3SamplePose)));
        PickSample3.setLinearHeadingInterpolation(putSample2Pose.getHeading(), pickUp3SamplePose.getHeading());

        MoveSample3 = new Path(new BezierLine(new Point(pickUp3SamplePose), new Point(move3SamplePose)));
        MoveSample3.setLinearHeadingInterpolation(pickUp3SamplePose.getHeading(), move3SamplePose.getHeading());

        PutSample3 = new Path(new BezierLine(new Point(move3SamplePose), new Point(put3SamplePose)));
        PutSample3.setLinearHeadingInterpolation(move3SamplePose.getHeading(), put3SamplePose.getHeading());

        PickSample4 = new Path(new BezierLine(new Point(put3SamplePose), new Point(pickUp4SamplePose)));
        PickSample4.setLinearHeadingInterpolation(put3SamplePose.getHeading(), pickUp4SamplePose.getHeading());

        MoveSample4 = new Path(new BezierLine(new Point(pickUp4SamplePose), new Point(move4SamplePose)));
        MoveSample4.setLinearHeadingInterpolation(pickUp4SamplePose.getHeading(), move4SamplePose.getHeading());

        PutSample4 = new Path(new BezierLine(new Point(move4SamplePose), new Point(put4SamplePose)));
        PutSample4.setLinearHeadingInterpolation(move4SamplePose.getHeading(), put4SamplePose.getHeading());

        PickSample5 = new Path(new BezierCurve(new Point(put4SamplePose), new Point(pickUp5SampleControlPose), new Point(pickUp5SamplePose)));
        PickSample5.setLinearHeadingInterpolation(put4SamplePose.getHeading(), pickUp5SamplePose.getHeading());

        PutSample5 = new Path(new BezierCurve(new Point(pickUp5SamplePose), new Point(put5SampleControlPose), new Point(put5SamplePos)));
        PutSample5.setLinearHeadingInterpolation(pickUp5SamplePose.getHeading(), put5SamplePos.getHeading());


    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                     follower.followPath(PickSample2,true);

                    setPathState(2);
                }
                break;

            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(MoveSample2Chain,true);
                    setPathState(-1);
                }
                break;

            /*case 3:
                if(!follower.isBusy()) {
                    follower.followPath(PutSample2,true);
                    setPathState(4);
                }
                break;


            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(PickSample3,true);
                    setPathState(5);
                }
                break;

            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(MoveSample3,true);
                    setPathState(6);
                }
                break;

            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(PutSample3,true);
                    setPathState(7);
                }
                break;


            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(PickSample4,true);
                    setPathState(8);
                }
                break;

            case 8:
                if(!follower.isBusy()) {
                    follower.followPath(MoveSample4,true);
                    setPathState(9);
                }
                break;

            case 9:
                if(!follower.isBusy()) {
                    follower.followPath(PutSample4, true);
                    setPathState(10);
                }
                break;

            case 10:
                if(!follower.isBusy()) {
                    follower.followPath(PickSample5,true);
                    setPathState(11);
                }
                break;

            case 11:
                if(!follower.isBusy()) {
                    follower.followPath(PutSample5,true);
                    setPathState(11);
                }
                break;


             */

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

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
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