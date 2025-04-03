package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Comands.Constants.preSubmRodePos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.preSubWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.rodeInToTicks;
import static org.firstinspires.ftc.teamcode.Comands.Constants.rodeTicksToIn;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Comands.Etesito;
import org.firstinspires.ftc.teamcode.subsystems.Vision.CrosshairVision;
import org.opencv.core.RotatedRect;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous
public class PedroOpenCv extends OpMode {
    double ScaleFactor = 200;

    Etesito etesito;

    CrosshairVision vision;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    //This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>

    private final Pose startPose = new Pose(0, 01, Math.toRadians(0));

    private final Pose putSpecimen1 = new Pose(25, 78, Math.toRadians(0));

    private Path scorePreload;
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(putSpecimen1)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), putSpecimen1.getHeading());

    }

    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();

        RotatedRect[] rects = vision.getLastRects();

        for(int i = 0 ; i < rects.length ; i++) {
            telemetry.addData("detection #" + i, rects[i].center);
        }

        vision.updateExposure();

        double tY = vision.pipeline.vectorY / ScaleFactor;
        double tX = vision.pipeline.vectorX / ScaleFactor;

        double targetAngle = Math.atan2(tY, tX);

        follower.holdPoint(new BezierPoint (new Point(tX, startPose.getY())), targetAngle);

        double armCurrentIn = -etesito.rodeMotor.getCurrentPosition() * rodeTicksToIn;

        double tYRode = tY * rodeInToTicks;

        int rodeTarget = (int)(armCurrentIn + tYRode);

        if (gamepad2.dpad_up){
            etesito.rodeSb.rodeToPos(rodeTarget).schedule();
        } else if (gamepad2.dpad_down) {
            etesito.rodeSb.rodeToPos(preSubmRodePos).schedule();
        }

        telemetry.addData("tX", tX);
        telemetry.addData("tY", tY);
        telemetry.addData("tYRode", tYRode);
        telemetry.addData("tAngle", targetAngle);
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {

        etesito.init(hardwareMap, true, true);

        etesito.rodeSb.setDefaultCommand(etesito.rodeSb.rodeUpdate());

        etesito.rodeSb.rodeToPos(preSubmRodePos).schedule();

        etesito.wristSb.servoPosCMD(preSubWristPos).schedule();

        vision = new CrosshairVision(etesito.webcam);
        vision.init();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        follower.setStartingPose(startPose);
        buildPaths();
    }
    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
    }

    @Override
    public void stop() {
    }
}