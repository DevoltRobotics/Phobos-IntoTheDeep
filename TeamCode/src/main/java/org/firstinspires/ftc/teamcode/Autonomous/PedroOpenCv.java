package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Comands.Constants.ScaleFactor;
import static org.firstinspires.ftc.teamcode.Comands.Constants.contractWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.degreesPerPixel;
import static org.firstinspires.ftc.teamcode.Comands.Constants.downWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.preSubmRodePos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.preSubWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.rodeInToTicks;
import static org.firstinspires.ftc.teamcode.Comands.Constants.rodeTicksToIn;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Comands.Etesito;
import org.firstinspires.ftc.teamcode.subsystems.PedroSb;
import org.firstinspires.ftc.teamcode.subsystems.Vision.CrosshairVision;
import org.opencv.core.RotatedRect;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
@Config
@TeleOp
public class PedroOpenCv extends OpMode {
    PedroSb pedroSb;

    Etesito etesito = new Etesito();

    CrosshairVision vision;
    private Follower follower;
    private Timer opmodeTimer;

    private Command pathcommand;

    //This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>

    private final Pose startPose = new Pose(0, 10, Math.toRadians(270));

    private final Pose putSpecimen1 = new Pose(0, 0, Math.toRadians(270));

    private Path scorePreload, back;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(putSpecimen1)));
        scorePreload .setConstantHeadingInterpolation(putSpecimen1.getHeading());

        back = new Path(new BezierLine(new Point(putSpecimen1), new Point(startPose)));
        back .setConstantHeadingInterpolation(startPose.getHeading());

        pathcommand = new SequentialCommandGroup(
                new WaitCommand(150),

                pedroSb.turnChassis(0.5, etesito.imu),

                new WaitCommand(50),

                pedroSb.turnChassis(0.3, etesito.imu),

                new WaitCommand(50),

                etesito.rodeSb.rodeToPosVision(vision, telemetry),
                new WaitCommand(50),

                etesito.intakeSb.crservoCMD(1),

                new WaitCommand(200),
                etesito.wristSb.servoSmootrCMD(downWristPos, 0.2),

                new WaitCommand(50),

                etesito.wristSb.servoPosCMD(contractWristPos)

                /*
                new ParallelDeadlineGroup(
                        new WaitCommand(150),
                        pedroSb.returnPath()
                ),



                pedroSb.followPathCmd(back)

                 */
        );
    }

    double beforeHeading;
    double beforeRodeIn;

    @Override
    public void loop() {

        // These loop the movements of the robot

        etesito.rodeMotor.setPower(etesito.rodeController.update(etesito.rodeMotor.getCurrentPosition()) * 0.09);

        RotatedRect[] rects = vision.getLastRects();

        for (int i = 0; i < rects.length; i++) {
            telemetry.addData("detection #" + i, rects[i].center);
        }
        //double targetX = Range.clip(beforeXPos - tX, -4, 4);

        /*

        if (gamepad2.y) {
            pedroSb.turnChassis(0.3, 1).schedule();
        }

        if (gamepad2.dpad_up) {
            etesito.rodeSb.rodeToPosVision(vision, telemetry).schedule();
        } else if (gamepad2.dpad_down) {
            etesito.rodeSb.rodeToPos(preSubmRodePos).schedule();
        }

        if (gamepad2.dpad_right) {
            etesito.wristSb.servoSmootrCMD(downWristPos, 0.3).schedule();

        } else if (gamepad2.dpad_left) {
            etesito.wristSb.servoPosCMD(preSubWristPos).schedule();
        }

        if(gamepad2.a){
            follower.setPose(startPose);

        }

        if (gamepad2.right_bumper) {
            etesito.intakeSb.crservoCMD(1).schedule();

        } else if (gamepad2.left_bumper) {
            etesito.intakeSb.crservoCMD(-1).schedule();

        } else {
            etesito.intakeSb.crservoCMD(0).schedule();
        }

        beforeRodeIn = -etesito.rodeMotor.getCurrentPosition() * rodeTicksToIn;
        beforeHeading = Math.toDegrees(follower.getPose().getHeading());
        */


        vision.updateExposure();

        /*telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());

        telemetry.addData("heading", beforeHeading);

        telemetry.update();*/

        CommandScheduler.getInstance().run();
    }

    @Override
    public void init() {
        etesito.init(hardwareMap, true, true);

        etesito.rodeSb.rodeToPos(preSubmRodePos).schedule();
        etesito.wristSb.servoPosCMD(preSubWristPos).schedule();

        vision = new CrosshairVision(etesito.webcam);
        vision.init();

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        follower.setStartingPose(startPose);

        pedroSb = new PedroSb(follower, etesito.chassiscontroller, vision, telemetry, etesito.fl, etesito.bl, etesito.br, etesito.fr);

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        CommandScheduler.getInstance().registerSubsystem(pedroSb);

        buildPaths();

        etesito.rodeSb.rodeToPos(preSubmRodePos).schedule();
    }
    @Override
    public void init_loop() {
        etesito.rodeMotor.setPower(etesito.rodeController.update(etesito.rodeMotor.getCurrentPosition()) * 0.09);
        CommandScheduler.getInstance().run();

    }

    @Override
    public void start() {
        pathcommand.schedule();
        opmodeTimer.resetTimer();
    }

    @Override
    public void stop() {
        follower.breakFollowing();
        CommandScheduler.getInstance().cancelAll();

    }
}