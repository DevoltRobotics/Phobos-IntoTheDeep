package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Comands.Constants.basketArmPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.basketWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.contractWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.downWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.highRodePos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.preSubWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.preSubmRodePos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.servosHangingPos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
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
import org.firstinspires.ftc.teamcode.subsystems.Vision.CrosshairVision;
import org.opencv.core.RotatedRect;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous
public class AutonomousSample extends OpMode {

    private Follower follower;
    private Timer actionTimer;
    private Timer opmodeTimer;

    PedroSb pedroSb;

    Etesito etesito = new Etesito();

    CrosshairVision vision;

    double xPose;
    double yPose;
    double heading;

    private int rodePick2 = -1000;
    private int rodePick3 = -800;

    private int rodePick4 = -350;

    private Command pathCommand;

    private final Pose startPose = new Pose(8.5, 112, Math.toRadians(270));

    private final Pose putSample1Pose = new Pose(13, 129, Math.toRadians(319));
    //private final Pose putSample1ControlPose = new Pose(20, 112, Math.toRadians(340));

    private final Pose pickUp2SamplePose = new Pose(30, 117, Math.toRadians(49));

    private final Pose putSample2Pose = new Pose(17, 133, Math.toRadians(320));
    private final Pose putSample2ControlPose = new Pose(23, 132, Math.toRadians(340));

    private final Pose pickUp3SamplePose = new Pose(31, 128, Math.toRadians(46));

    private final Pose put3SamplePose = new Pose(16, 132, Math.toRadians(319));
    private final Pose putSample3ControlPose = new Pose(26, 132, Math.toRadians(340));

    private final Pose pickUp4SamplePose = new Pose(34, 137, Math.toRadians(310));

    private final Pose put4SamplePose = new Pose(16, 131, Math.toRadians(318));
    private final Pose put4SampleControlPose = new Pose(23, 125, Math.toRadians(318));

    private final Pose pickUp5SamplePose = new Pose(65, 104, Math.toRadians(270));
    private final Pose pickUp5SampleControlPose = new Pose(65, 120, Math.toRadians(275));

    private final Pose put5OneSamplePose = new Pose(55, 124, Math.toRadians(290));
    private final Pose put5TwoSamplePose = new Pose(16, 130, Math.toRadians(315));

    private final Pose pickUp6SamplePose = new Pose(65, 104, Math.toRadians(270));
    private final Pose pickUp6SampleControlPose = new Pose(65, 125, Math.toRadians(275));

    private final Pose put6OneSamplePose = new Pose(55, 124, Math.toRadians(290));
    private final Pose put6TwoSamplePose = new Pose(16, 130, Math.toRadians(315));

    private Path PutSample1;

    private PathChain PickSample2, PutSample2, PickSample3, PutSample3, PickSample4, PutSample4,
            PickSample5, PutSample5One, PutSample5Two, PickSample6, PutSample6One, PutSample6Two;


    public void buildPaths() {
        PutSample1 = new Path(new BezierLine(new Point(startPose), new Point(putSample1Pose)));
        PutSample1.setPathEndTimeoutConstraint(1);
        PutSample1.setLinearHeadingInterpolation(startPose.getHeading(), putSample1Pose.getHeading());

        PickSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(putSample1Pose), new Point(pickUp2SamplePose)))
                .setLinearHeadingInterpolation(putSample1Pose.getHeading(), pickUp2SamplePose.getHeading())
                .setPathEndTimeoutConstraint(100)
                .build();

        PutSample2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickUp2SamplePose), new Point(putSample2ControlPose), new Point(putSample2Pose)))
                .setPathEndTimeoutConstraint(1)
                .setLinearHeadingInterpolation(pickUp2SamplePose.getHeading(), putSample2Pose.getHeading())
                .build();

        PickSample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(putSample2Pose), new Point(pickUp3SamplePose)))
                .setLinearHeadingInterpolation(putSample2Pose.getHeading(), pickUp3SamplePose.getHeading())
                .setPathEndTimeoutConstraint(1)
                .build();

        PutSample3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickUp3SamplePose), new Point(putSample3ControlPose), new Point(put3SamplePose)))
                .setLinearHeadingInterpolation(pickUp3SamplePose.getHeading(), put3SamplePose.getHeading())
                .setPathEndTimeoutConstraint(1)
                .build();

        PickSample4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(put3SamplePose), new Point(pickUp4SamplePose)))
                .setPathEndTimeoutConstraint(1)
                .setLinearHeadingInterpolation(put3SamplePose.getHeading(), pickUp4SamplePose.getHeading())
                .build();

        PutSample4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickUp4SamplePose), new Point(put4SampleControlPose), new Point(put4SamplePose)))
                .setLinearHeadingInterpolation(pickUp4SamplePose.getHeading(), put4SamplePose.getHeading())
                .setPathEndTimeoutConstraint(1)
                .build();

        PickSample5 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(put4SamplePose), new Point(pickUp5SampleControlPose), new Point(pickUp5SamplePose)))
                .setLinearHeadingInterpolation(put4SamplePose.getHeading(), pickUp5SamplePose.getHeading())
                .setPathEndTimeoutConstraint(100)
                .build();

        Pose pre5 = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
        PutSample5One = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pre5), new Point(put5OneSamplePose)))
                .setLinearHeadingInterpolation(pre5.getHeading(), put5OneSamplePose.getHeading())
                .setPathEndTimeoutConstraint(1)
                .build();

        PutSample5Two = follower.pathBuilder()
                .addPath(new BezierLine(new Point(put5OneSamplePose), new Point(put5TwoSamplePose)))
                .setLinearHeadingInterpolation(put5OneSamplePose.getHeading(), put5TwoSamplePose.getHeading())
                .setPathEndTimeoutConstraint(1)
                .build();

        PickSample6 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(put5TwoSamplePose), new Point(pickUp6SampleControlPose), new Point(pickUp6SamplePose)))
                .setLinearHeadingInterpolation(put5TwoSamplePose.getHeading(), pickUp6SamplePose.getHeading())
                .setPathEndTimeoutConstraint(100)
                .build();

        Pose pre6 = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
        PutSample6One = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pre6), new Point(put6OneSamplePose)))
                .setLinearHeadingInterpolation(pre6.getHeading(), put6OneSamplePose.getHeading())
                .setPathEndTimeoutConstraint(1)
                .build();

        PutSample6Two = follower.pathBuilder()
                .addPath(new BezierLine(new Point(put6OneSamplePose), new Point(put6TwoSamplePose)))
                .setLinearHeadingInterpolation(put6OneSamplePose.getHeading(), put6TwoSamplePose.getHeading())
                .setPathEndTimeoutConstraint(100)
                .build();

        pathCommand =
                new SequentialCommandGroup(
                        etesito.armSb.armToPos(basketArmPos),
                        new WaitCommand(150),
                        new ParallelDeadlineGroup(
                                pedroSb.followPathCmd(PutSample1),
                                etesito.rodeSb.rodeToPos(highRodePos),
                                etesito.wristSb.servoPosCMD(basketWristPos),
                                etesito.servosSb.mirrorServoPosCMD(servosHangingPos)
                        ),

                        etesito.intakeSb.crservoCMD(-1),

                        new WaitCommand(300),

                        etesito.wristSb.servoPosCMD(downWristPos),
                        new WaitCommand(150),

                        etesito.rodeSb.rodeToPos(0),

                        new WaitCommand(200),

                        //PUT_FIRST_SAMPLE

                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(PickSample2),
                                etesito.armSb.armToPosSmooth(0, 0.7),
                                etesito.intakeSb.crservoCMD(1)

                        ),

                        etesito.rodeSb.rodeToPosSmooth(rodePick2, 0.73),

                        etesito.wristSb.servoPosCMD(basketWristPos),
                        etesito.rodeSb.rodeToPos(-350),

                        new WaitCommand(150),

                        etesito.armSb.armToPos(basketArmPos),

                        new WaitCommand(150),
                        etesito.rodeSb.rodeToPos(highRodePos),
                        new WaitCommand(200),

                        etesito.intakeSb.crservoCMD(0),
                        pedroSb.followPathCmd(PutSample2),

                        etesito.intakeSb.crservoCMD(-1),

                        new WaitCommand(200),

                        etesito.wristSb.servoPosCMD(downWristPos),
                        new WaitCommand(150),

                        etesito.rodeSb.rodeToPos(-250),

                        new WaitCommand(150),

                        //PUT_SECOND_SAMPLE

                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(PickSample3),
                                etesito.armSb.armToPosSmooth(0, 0.7),
                                etesito.intakeSb.crservoCMD(1)

                        ),

                        etesito.rodeSb.rodeToPosSmooth(rodePick3, 0.7),

                        etesito.wristSb.servoPosCMD(basketWristPos),
                        etesito.rodeSb.rodeToPos(-350),

                        new WaitCommand(150),

                        etesito.armSb.armToPos(basketArmPos),

                        new WaitCommand(150),
                        etesito.rodeSb.rodeToPos(highRodePos),
                        new WaitCommand(200),


                        etesito.intakeSb.crservoCMD(0),

                                pedroSb.followPathCmd(PutSample3),



                        etesito.intakeSb.crservoCMD(-1),
                        new WaitCommand(200),

                        etesito.wristSb.servoPosCMD(downWristPos),
                        new WaitCommand(150),

                        etesito.rodeSb.rodeToPos(-150),

                        new WaitCommand(200),

                        //PUT_THIRD_SAMPLE

                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(PickSample4),
                                etesito.armSb.armToPosSmooth(0, 0.7),
                                etesito.intakeSb.crservoCMD(1)
                        ),

                        etesito.rodeSb.rodeToPosSmooth(rodePick4, 0.4),

                        etesito.wristSb.servoPosCMD(basketWristPos),

                        etesito.armSb.armToPos(basketArmPos),

                        new WaitCommand(150),
                        etesito.rodeSb.rodeToPos(highRodePos),
                        new WaitCommand(120),

                        etesito.intakeSb.crservoCMD(0),
                                pedroSb.followPathCmd(PutSample4),


                        etesito.intakeSb.crservoCMD(-1),

                        new WaitCommand(150),

                        etesito.intakeSb.crservoCMD(-1),

                        new WaitCommand(200),

                        etesito.wristSb.servoPosCMD(preSubWristPos),
                        new WaitCommand(150),

                        etesito.rodeSb.rodeToPos(preSubmRodePos),
                        new WaitCommand(100),

                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(PickSample5),
                                etesito.armSb.armToPosSmooth(0, 0.7)
                        ),

                        pedroSb.breakPath(),
                        new WaitCommand(80),

                        pedroSb.turnChassis(1, etesito.imu),
                        new WaitCommand(80),

                        pedroSb.reTurnChassis(0.7, etesito.imu),

                        new WaitCommand(115),

                        new ParallelCommandGroup(
                                etesito.intakeSb.crservoCMD(1),
                                etesito.rodeSb.rodeToPosVision(vision, telemetry, 0, etesito.wristSb),
                                pedroSb.breakPath()
                        ),

                        etesito.intakeSb.crservoCMD(0),
                        etesito.wristSb.servoPosCMD(contractWristPos),
                        new WaitCommand(50),

                        etesito.rodeSb.rodeToPos(0),
                        new WaitCommand(150),

                        pedroSb.returnPath(),

                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(PutSample5One),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        etesito.wristSb.servoPosCMD(basketWristPos),

                                        etesito.armSb.armToPos(basketArmPos)

                                )),


                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(PutSample5Two),
                                etesito.rodeSb.rodeToPos(highRodePos)

                        ),

                        etesito.intakeSb.crservoCMD(-1),
                        new WaitCommand(200),

                        etesito.wristSb.servoPosCMD(preSubWristPos),
                        new WaitCommand(150),

                        etesito.rodeSb.rodeToPos(preSubmRodePos),
                        new WaitCommand(200),

                        //  PUT_SAMPLE_5

                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(PickSample6),
                                etesito.armSb.armToPosSmooth(0, 0.7)
                        ),

                        pedroSb.breakPath(),
                        new WaitCommand(80),

                        pedroSb.turnChassis(1, etesito.imu),
                        new WaitCommand(80),

                        pedroSb.reTurnChassis(0.7, etesito.imu),

                        new WaitCommand(115),

                        new ParallelCommandGroup(
                                etesito.intakeSb.crservoCMD(1),
                                etesito.rodeSb.rodeToPosVision(vision, telemetry, 0, etesito.wristSb),
                                pedroSb.breakPath()
                        ),

                        etesito.intakeSb.crservoCMD(0),
                        etesito.wristSb.servoPosCMD(contractWristPos),
                        new WaitCommand(50),

                        etesito.rodeSb.rodeToPos(0),
                        new WaitCommand(150),

                        pedroSb.returnPath(),

                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(PutSample6One),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        etesito.wristSb.servoPosCMD(basketWristPos),

                                        etesito.armSb.armToPos(basketArmPos)

                                )),


                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(PutSample6Two),
                                etesito.rodeSb.rodeToPos(highRodePos)

                        ),

                        etesito.intakeSb.crservoCMD(-1),
                        new WaitCommand(200),

                        etesito.wristSb.servoPosCMD(contractWristPos),
                        new WaitCommand(150),

                        etesito.rodeSb.rodeToPos(0),
                        new WaitCommand(200),

                        etesito.armSb.armToPosSmooth(0, 0.4)

                );
    }

    @Override
    public void init() {
        etesito.init(hardwareMap, true, true);

        vision = new CrosshairVision(etesito.webcam);
        vision.init();

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        pedroSb = new PedroSb(follower, etesito.chassiscontroller, vision, telemetry, etesito.fl, etesito.bl, etesito.br, etesito.fr);

        buildPaths();

        etesito.initCmd().schedule();
    }

    @Override
    public void init_loop() {
        etesito.armMotor.setPower(-etesito.armController.update(etesito.armMotor.getCurrentPosition()) * 0.4);
        etesito.rodeMotor.setPower(etesito.rodeController.update(etesito.rodeMotor.getCurrentPosition()) * 0.09);
        CommandScheduler.getInstance().run();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        CommandScheduler.getInstance().registerSubsystem(pedroSb);
        pathCommand.schedule();
    }

    @Override
    public void loop() {
        // These loop te movements of the robot

        RotatedRect rect = vision.getRect();

        if (rect != null) {
            telemetry.addData("detection", rect);
        }


        vision.updateExposure();

        etesito.armMotor.setPower(-etesito.armController.update(etesito.armMotor.getCurrentPosition()) * 0.4);
        etesito.rodeMotor.setPower(etesito.rodeController.update(etesito.rodeMotor.getCurrentPosition()) * 0.09);

        CommandScheduler.getInstance().run();

        xPose = follower.getPose().getX();
        yPose = follower.getPose().getY();
        heading = follower.getPose().getHeading();

        telemetry.addData("x", xPose);
        telemetry.addData("y", yPose);
        telemetry.addData("heading", heading);
        telemetry.update();


        Etesito.headingAuto = follower.getPose().getHeading();
    }

    @Override
    public void stop() {
        etesito.wristSb.servoPosCMD(0.5);
        pathCommand.cancel();
    }
}