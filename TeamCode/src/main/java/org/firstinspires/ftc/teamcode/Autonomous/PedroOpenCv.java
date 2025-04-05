package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Comands.Constants.preSubmRodePos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.preSubWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.rodeInToTicks;
import static org.firstinspires.ftc.teamcode.Comands.Constants.rodeTicksToIn;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Comands.Etesito;
import org.firstinspires.ftc.teamcode.subsystems.Vision.CrosshairVision;
import org.opencv.core.RotatedRect;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
@Config
@TeleOp
public class PedroOpenCv extends OpMode {
    public static double ScaleFactor = 10;
    public static double focalLenghtXMid = 160;

    public static boolean move = false;

    double cX;


    Etesito etesito = new Etesito();

    CrosshairVision vision;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    //This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    private final Pose putSpecimen1 = new Pose(25, 78, Math.toRadians(0));

    private Path scorePreload;
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(putSpecimen1)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), putSpecimen1.getHeading());

    }

    double beforeHeading;
    double beforeXPos;
    double beforeRodeIn;

    @Override
    public void loop() {
        // These loop the movements of the robot

        etesito.rodeMotor.setPower(etesito.rodeController.update(etesito.rodeMotor.getCurrentPosition()) * 0.09);

        follower.update();

        RotatedRect[] rects = vision.getLastRects();

        for(int i = 0 ; i < rects.length ; i++) {
            telemetry.addData("detection #" + i, rects[i].center);
        }

        RotatedRect rect = new RotatedRect();
        if(rects.length > 0) {
            rect = rects[0];
        }

        vision.updateExposure();


        if (rect.center.x >= focalLenghtXMid){
            cX = rect.center.x - focalLenghtXMid;

        } else if (rect.center.x < focalLenghtXMid){
            cX = -(focalLenghtXMid - rect.center.x);

        }else {
            cX = 0;
        }

        double tY = rect.center.y / ScaleFactor;
        double tX = cX / ScaleFactor;

        //double targetX = Range.clip(beforeXPos - tX, -4, 4);
        double targetAngle = beforeHeading - tX;

        if (move) {
            follower.holdPoint(new BezierPoint(new Point(startPose.getX(), startPose.getY())), Math.toRadians(targetAngle));
        }else {
            follower.holdPoint(new BezierPoint(new Point(0,0)), 0);
        }
        double tYRode = tY * rodeInToTicks;

        int rodeTarget = (int)(beforeRodeIn + tYRode);

        if (gamepad2.dpad_up){
            etesito.rodeSb.rodeToPos(rodeTarget).schedule();
        } else if (gamepad2.dpad_down) {
            etesito.rodeSb.rodeToPos(preSubmRodePos).schedule();
        }

        beforeRodeIn = -etesito.rodeMotor.getCurrentPosition() * rodeTicksToIn;
        beforeXPos = follower.getPose().getX();
        beforeHeading = Math.toDegrees(follower.getPose().getHeading());

        //telemetry.addData("tX", tX);
        //telemetry.addData("tY", tY);
        //telemetry.addData("tYRode", tYRode);
        telemetry.addData("tY", rect.center.y);
        telemetry.addData("tX", tX);

        telemetry.addData("tAngle", targetAngle);
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();

        CommandScheduler.getInstance().run();

    }

    @Override
    public void init() {
        etesito.init(hardwareMap, true, true);

        //FollowerConstants.headingPIDFCoefficients.setCoefficients(0.55, 0, 0.009, 0);
        //FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(1, 0, 0.05, 0);


        etesito.rodeSb.rodeToPos(preSubmRodePos).schedule();
        etesito.wristSb.servoPosCMD(preSubWristPos).schedule();

        vision = new CrosshairVision(etesito.webcam);
        vision.init();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        follower.setStartingPose(startPose);

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        buildPaths();
    }
    @Override
    public void init_loop() {
        etesito.rodeMotor.setPower(etesito.rodeController.update(etesito.rodeMotor.getCurrentPosition()) * 0.09);
        CommandScheduler.getInstance().run();

    }

    @Override
    public void start() {
        etesito.rodeSb.rodeToPos(preSubmRodePos).schedule();
        etesito.wristSb.servoPosCMD(preSubWristPos).schedule();
        opmodeTimer.resetTimer();
    }

    @Override
    public void stop() {
    }
}