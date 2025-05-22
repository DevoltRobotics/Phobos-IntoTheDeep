package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Comands.Constants.basketArmPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.basketWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.contractWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.downWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.highRodePos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.highRodePosAuto;
import static org.firstinspires.ftc.teamcode.Comands.Constants.parkArmPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.parkRodePos;
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

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous
public class AuSmlpMedio extends OpMode {

    private Follower follower;
    private Timer actionTimer;
    private Timer opmodeTimer;

    PedroSb pedroSb;

    Etesito etesito = new Etesito();

    CrosshairVision vision;

    double xPose;
    double yPose;
    double heading;

    //435 rpm
    /*private int rodePick2 = -700;

    private int rodePrePick3 = -250;

    private int rodePick3 = -750;

    private int rodePrePick4 = -150;
    private int rodePick4 = -600;

    private int finalRodePose = -400;

     */

    private int rodePick2 = -900;

    private int rodePrePick3 = -350;

    private int rodePick3 = -1050;

    private int rodePrePick4 = -210;
    private int rodePick4 = -840;

    private int finalRodePose = -560;

    private Command pathCommand;

    private final Pose startPose = new Pose(8.5, 112, Math.toRadians(270));

    private final Pose putSample1Pose = new Pose(15, 129, Math.toRadians(319));
    //private final Pose putSample1ControlPose = new Pose(20, 112, Math.toRadians(340));

    private final Pose pickUp2SamplePose = new Pose(39, 108, Math.toRadians(70));

    private final Pose putSample2Pose = new Pose(17, 133, Math.toRadians(320));
        private final Pose putSample2ControlPose = new Pose(30, 123, Math.toRadians(340));

    private final Pose pickUp3SamplePose = new Pose(44, 115, Math.toRadians(90));

    private final Pose put3SamplePose = new Pose(18, 132, Math.toRadians(321));
    private final Pose putSample3ControlPose = new Pose(26, 130, Math.toRadians(340));

    private final Pose pickUp4SamplePose = new Pose(44, 125, Math.toRadians(90));

    private final Pose put4SamplePose = new Pose(18, 131, Math.toRadians(320));
    private final Pose put4SampleControlPose = new Pose(26, 133, Math.toRadians(318));

    private final Pose pickUp5SamplePose = new Pose(65, 104, Math.toRadians(270));
    private final Pose pickUp5SampleControlPose = new Pose(68, 125, Math.toRadians(275));

    private final Pose put5OneSamplePose = new Pose(55, 121, Math.toRadians(315));
    private final Pose put5TwoSamplePose = new Pose(16, 127, Math.toRadians(315));

    private final Pose parkPose = new Pose(68, 103, Math.toRadians(270));
    private final Pose parkControlPose = new Pose(68, 125, Math.toRadians(275));

    private Path PutSample1;

    private PathChain PickSample2, PutSample2, PickSample3, PutSample3, PickSample4, PutSample4,
            PickSample5, PutSample5One, PutSample5Two, Park, PutSample6One, PutSample6Two;


    public void buildPaths() {
        PutSample1 = new Path(new BezierLine(new Point(startPose), new Point(putSample1Pose)));
        PutSample1.setLinearHeadingInterpolation(startPose.getHeading(), putSample1Pose.getHeading());

        PickSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(putSample1Pose), new Point(pickUp2SamplePose)))
                .setLinearHeadingInterpolation(putSample1Pose.getHeading(), pickUp2SamplePose.getHeading())
                .build();

        PutSample2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickUp2SamplePose), new Point(putSample2ControlPose), new Point(putSample2Pose)))
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
                .addPath(new BezierCurve(new Point(pickUp4SamplePose), new Point(put4SampleControlPose), new Point(put4SamplePose)))
                .setLinearHeadingInterpolation(pickUp4SamplePose.getHeading(), put4SamplePose.getHeading())
                .build();

        PickSample5 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(put4SamplePose), new Point(pickUp5SampleControlPose), new Point(pickUp5SamplePose)))
                .setLinearHeadingInterpolation(put4SamplePose.getHeading(), pickUp5SamplePose.getHeading())
                .build();

        Pose pre5 = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
        PutSample5One = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pre5), new Point(put5OneSamplePose)))
                .setPathEndTimeoutConstraint(1)
                .setConstantHeadingInterpolation(pre5.getHeading())
                .build();

        PutSample5Two = follower.pathBuilder()
                .addPath(new BezierLine(new Point(put5OneSamplePose), new Point(put5TwoSamplePose)))
                .setLinearHeadingInterpolation(put5OneSamplePose.getHeading(), put5TwoSamplePose.getHeading())
                .build();

        Park = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(put5TwoSamplePose), new Point(parkControlPose), new Point(parkPose)))
                .setLinearHeadingInterpolation(put5TwoSamplePose.getHeading(), parkPose.getHeading())
                .build();

        pathCommand =
                new SequentialCommandGroup(
                        etesito.armSb.armToPos(basketArmPos),
                        new WaitCommand(150),
                        new ParallelDeadlineGroup(
                                pedroSb.followPathCmd(PutSample1),
                                etesito.rodeSb.rodeToPos(highRodePosAuto),
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
                                etesito.armSb.armToPosSmooth(0, 1),
                                etesito.intakeSb.crservoCMD(1)

                        ),

                        etesito.rodeSb.rodeToPosSmooth(rodePick2, 0.73),

                        etesito.wristSb.servoPosCMD(contractWristPos),
                        etesito.rodeSb.rodeToPos(-350),

                        new WaitCommand(150),

                        etesito.armSb.armToPos(basketArmPos),

                        new WaitCommand(150),
                        etesito.rodeSb.rodeToPos(highRodePosAuto),
                        new WaitCommand(200),

                        etesito.intakeSb.crservoCMD(0),
                        pedroSb.followPathCmd(PutSample2),
                        etesito.wristSb.servoPosCMD(basketWristPos),
                        new WaitCommand(200),

                        etesito.intakeSb.crservoCMD(-1),

                        new WaitCommand(200),

                        etesito.wristSb.servoPosCMD(downWristPos),
                        new WaitCommand(150),

                        etesito.rodeSb.rodeToPos(rodePrePick3),

                        new WaitCommand(150),

                        //PUT_SECOND_SAMPLE

                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(PickSample3),
                                etesito.armSb.armToPosSmooth(0, 1),
                                etesito.intakeSb.crservoCMD(1)

                        ),

                        etesito.rodeSb.rodeToPosSmooth(rodePick3, 0.7),

                        etesito.wristSb.servoPosCMD(contractWristPos),
                        etesito.rodeSb.rodeToPos(-350),

                        new WaitCommand(150),

                        etesito.armSb.armToPos(basketArmPos),

                        new WaitCommand(150),
                        etesito.rodeSb.rodeToPos(highRodePosAuto),
                        new WaitCommand(200),


                        etesito.intakeSb.crservoCMD(0),

                        pedroSb.followPathCmd(PutSample3),
                        etesito.wristSb.servoPosCMD(basketWristPos),
                        new WaitCommand(200),

                        etesito.intakeSb.crservoCMD(-1),
                        new WaitCommand(200),

                        etesito.wristSb.servoPosCMD(downWristPos),
                        new WaitCommand(150),

                        etesito.rodeSb.rodeToPos(rodePrePick4),

                        new WaitCommand(200),

                        //PUT_THIRD_SAMPLE

                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(PickSample4),
                                etesito.armSb.armToPosSmooth(0, 1),
                                etesito.intakeSb.crservoCMD(1)
                        ),

                        etesito.rodeSb.rodeToPosSmooth(rodePick4, 0.4),

                        etesito.rodeSb.rodeToPos(0),

                        etesito.wristSb.servoPosCMD(contractWristPos),

                        etesito.armSb.armToPos(basketArmPos),

                        new WaitCommand(150),
                        etesito.rodeSb.rodeToPos(highRodePosAuto),
                        new WaitCommand(120),

                        etesito.intakeSb.crservoCMD(0),
                        pedroSb.followPathCmd(PutSample4),
                        etesito.wristSb.servoPosCMD(basketWristPos),
                        new WaitCommand(200),

                        etesito.intakeSb.crservoCMD(-1),

                        new WaitCommand(150),

                        etesito.intakeSb.crservoCMD(-1),

                        new WaitCommand(200),

                        etesito.wristSb.servoPosCMD(preSubWristPos),
                        new WaitCommand(150),

                        etesito.rodeSb.rodeToPos(preSubmRodePos),
                        new WaitCommand(100),

                        //PUT_FOUR_SAMPLE


                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(PickSample5),
                                etesito.armSb.armToPosSmooth(0, 1)
                        ),

                        pedroSb.breakPath(),
                        new WaitCommand(250),

                        pedroSb.turnChassis(1, etesito.imu),

                        new WaitCommand(250),

                        new ParallelCommandGroup(
                                etesito.intakeSb.crservoCMD(1),
                                etesito.rodeSb.rodeToPosVision(vision, telemetry, etesito.wristSb),
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
                                        new WaitCommand(550),
                                        etesito.wristSb.servoPosCMD(downWristPos),

                                        etesito.armSb.armToPos(basketArmPos)

                                )),


                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(PutSample5Two),
                                etesito.wristSb.servoPosCMD(basketWristPos),
                                etesito.rodeSb.rodeToPos(highRodePosAuto)

                        ),

                        etesito.intakeSb.crservoCMD(-1),
                        new WaitCommand(200),

                        etesito.wristSb.servoPosCMD(basketWristPos),
                        new WaitCommand(150),

                        etesito.rodeSb.rodeToPos(parkRodePos),
                        new WaitCommand(200),

                        //  PUT_SAMPLE_5

                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(Park),
                                etesito.armSb.armToPos(parkArmPos)
                        ),

                        etesito.wristSb.servoPosCMD(downWristPos)

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

        etesito.initCmdSample().schedule();
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
        pathCommand.cancel();
    }
}