package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Comands.Constants.SubWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.contractWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.downRodePos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.downWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.pickSubWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.postSubmRodePos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.preSubmRodePos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.preSubWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.submArmPos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Comands.Etesito;
import org.firstinspires.ftc.teamcode.subsystems.PedroSb;
import org.firstinspires.ftc.teamcode.subsystems.Vision.CrosshairVision;
import org.opencv.core.RotatedRect;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
@Config
@Autonomous
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

    int currentRode = 0;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(putSpecimen1)));
        scorePreload .setConstantHeadingInterpolation(putSpecimen1.getHeading());

        back = new Path(new BezierLine(new Point(putSpecimen1), new Point(startPose)));
        back .setConstantHeadingInterpolation(startPose.getHeading());

        pathcommand = new SequentialCommandGroup(
                pedroSb.breakPath(),
                new WaitCommand(150),

                pedroSb.turnChassis(1, etesito.imu),
                new WaitCommand(150),

                pedroSb.reTurnChassis(1, etesito.imu),

                new WaitCommand(150),

                new ParallelCommandGroup(
                etesito.intakeSb.crservoCMD(1),
                etesito.rodeSb.rodeToPosVision(vision, telemetry, 0, etesito.wristSb),
                        pedroSb.breakPath()
                ),

                        new WaitCommand(150),
                etesito.intakeSb.crservoCMD(0)



                /*
                    new WaitCommand(150),
                    pedroSb.returnPath()




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

        currentRode = etesito.rodeMotor.getCurrentPosition();
        RotatedRect rect = vision.getRect();

        if(rect != null) {
            telemetry.addData("detection", rect);
        }

        if (gamepad2.a){
            pathcommand.schedule();
        }

        if (gamepad2.y){
            etesito.rodeSb.rodeToPos(preSubmRodePos).schedule();
            etesito.wristSb.servoPosCMD(SubWristPos).schedule();
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
        etesito.wristSb.servoPosCMD(SubWristPos).schedule();
        opmodeTimer.resetTimer();
    }

    @Override
    public void stop() {
        follower.breakFollowing();
        CommandScheduler.getInstance().cancelAll();

    }
}