package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Comands.Constants.basketArmPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.basketWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.downWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.highRodePos;

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
import org.firstinspires.ftc.teamcode.subsystems.PedroSubsystem;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous
public class AutonomousSample extends OpMode {

    private Follower follower;
    private Timer actionTimer;
    private Timer opmodeTimer;

    PedroSubsystem pedroSb;

    Etesito etesito = new Etesito();

    private int rodePick2 = -950;
    private int rodePick3 = -850;

    private int rodePick4 = -350;

    private Command pathCommand;

    private final Pose startPose = new Pose(8.5, 112, Math.toRadians(0));

    private final Pose putSample1Pose = new Pose(16, 130, Math.toRadians(319));
    private final Pose putSample1ControlPose = new Pose(20, 112, Math.toRadians(340));

    private final Pose pickUp2SamplePose = new Pose(30, 117, Math.toRadians(50));

    private final Pose putSample2Pose = new Pose(17, 135, Math.toRadians(320));
    private final Pose putSample2ControlPose = new Pose(23, 136, Math.toRadians(340));

    private final Pose pickUp3SamplePose = new Pose(30, 128, Math.toRadians(50));

    private final Pose put3SamplePose = new Pose(17, 131, Math.toRadians(319));
    private final Pose putSample3ControlPose = new Pose(26, 136, Math.toRadians(340));

    private final Pose pickUp4SamplePose = new Pose(37, 135, Math.toRadians(300));

    private final Pose put4SamplePose = new Pose(17, 131, Math.toRadians(318));
    private final Pose put4SampleControlPose = new Pose(23, 125, Math.toRadians(318));

    private final Pose pickUp5SamplePose = new Pose(65, 105, Math.toRadians(270));
    private final Pose pickUp5SampleControlPose = new Pose(65, 120, Math.toRadians(275));

    private final Pose put5SamplePos = new Pose(12, 130, Math.toRadians(315));
    private final Pose put5SampleControlPose = new Pose(25, 120, Math.toRadians(275));

    private Path PutSample1, PutSample5;

    private PathChain PickSample2, PutSample2, PickSample3, PutSample3, PickSample4, PutSample4,
            PickSample5;


    public void buildPaths() {
        PutSample1 = new Path(new BezierCurve(new Point(startPose), new Point(putSample1ControlPose), new Point(putSample1Pose)));
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
                .addPath(new BezierCurve(new Point(pickUp3SamplePose), new Point(putSample3ControlPose), new Point(put3SamplePose)))
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

        PutSample5 = new Path(new BezierCurve(new Point(pickUp5SamplePose), new Point(put5SampleControlPose), new Point(put5SamplePos)));
        PutSample5.setLinearHeadingInterpolation(pickUp5SamplePose.getHeading(), put5SamplePos.getHeading());

        pathCommand =
                new SequentialCommandGroup(
                        etesito.armSb.armToPos(basketArmPos),
                        new WaitCommand(150),
                        new ParallelDeadlineGroup(
                                pedroSb.followPathCmd(PutSample1),
                                etesito.rodeSb.rodeToPos(highRodePos),
                                etesito.wristSb.servoPosCMD(basketWristPos)
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
                                etesito.armSb.armToPosSmooth(0, 0.55),
                                etesito.intakeSb.crservoCMD(1)

                        ),

                        etesito.rodeSb.rodeToPosSmooth(rodePick2, 0.7),

                        etesito.wristSb.servoPosCMD(basketWristPos),
                        etesito.rodeSb.rodeToPos(-350),

                        new WaitCommand(150),

                        etesito.armSb.armToPos(basketArmPos),

                        new WaitCommand(250),
                        etesito.rodeSb.rodeToPos(highRodePos),
                        new WaitCommand(200),

                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(PutSample2),
                                etesito.intakeSb.crservoCMD(0)
                        ),

                        etesito.intakeSb.crservoCMD(-1),

                        new WaitCommand(200),

                        etesito.wristSb.servoPosCMD(downWristPos),
                        new WaitCommand(150),

                        etesito.rodeSb.rodeToPos(-350),

                        new WaitCommand(150),

                        //PUT_SECOND_SAMPLE

                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(PickSample3),
                                etesito.armSb.armToPosSmooth(0, 0.6),
                                etesito.intakeSb.crservoCMD(1)

                        ),

                        etesito.rodeSb.rodeToPosSmooth(rodePick3, 0.7),

                        etesito.wristSb.servoPosCMD(basketWristPos),
                        etesito.rodeSb.rodeToPos(-350),

                        new WaitCommand(150),

                        etesito.armSb.armToPos(basketArmPos),

                        new WaitCommand(250),
                        etesito.rodeSb.rodeToPos(highRodePos),
                        new WaitCommand(200),


                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(PutSample3),
                                etesito.intakeSb.crservoCMD(0)
                        ),

                        etesito.intakeSb.crservoCMD(-1),

                        new WaitCommand(200),

                        etesito.wristSb.servoPosCMD(downWristPos),
                        new WaitCommand(150),

                        etesito.rodeSb.rodeToPos(0),

                        new WaitCommand(300),

                        //PUT_THIRD_SAMPLE

                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(PickSample4),
                                etesito.armSb.armToPosSmooth(0, 0.6),
                                etesito.intakeSb.crservoCMD(1)
                        ),

                        etesito.rodeSb.rodeToPosSmooth(rodePick4, 0.4),

                        etesito.wristSb.servoPosCMD(basketWristPos),

                        etesito.armSb.armToPos(basketArmPos),

                        new WaitCommand(250),
                        etesito.rodeSb.rodeToPos(highRodePos),
                        new WaitCommand(120),

                        new ParallelCommandGroup(
                                pedroSb.followPathCmd(PutSample4),
                                etesito.intakeSb.crservoCMD(0)
                        ),

                        new WaitCommand(100),

                        etesito.intakeSb.crservoCMD(-1),

                        new WaitCommand(200),

                        etesito.wristSb.servoPosCMD(downWristPos),
                        new WaitCommand(150),

                        etesito.rodeSb.rodeToPos(0),

                        new WaitCommand(300)


                        //PUT_SECOND_SAMPLE


                );
    }

    @Override
    public void init() {
        etesito.init(hardwareMap, true, true);

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        pedroSb = new PedroSubsystem(follower);

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

        etesito.armMotor.setPower(-etesito.armController.update(etesito.armMotor.getCurrentPosition()) * 0.4);
        etesito.rodeMotor.setPower(etesito.rodeController.update(etesito.rodeMotor.getCurrentPosition()) * 0.09);

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