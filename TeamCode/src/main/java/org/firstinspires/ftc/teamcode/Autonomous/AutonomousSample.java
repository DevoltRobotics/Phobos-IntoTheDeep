package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Comands.Constants.basketWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.contractAbramPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.midOpenAbramPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.openAbramPos;

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
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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

    private Command pathCommand;

    //This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>

    private final Pose startPose = new Pose(8.5, 112, Math.toRadians(0));

    private final Pose putSample1Pose = new Pose(17, 126, Math.toRadians(315));
    private final Pose putSample1ControlPose = new Pose(19, 112, Math.toRadians(340));

    private final Pose pickUp2SamplePose = new Pose(25, 126, Math.toRadians(335));

    private final Pose move2SamplePose = new Pose(25, 126, Math.toRadians(325));

    private final Pose putSample2Pose = new Pose(17, 126, Math.toRadians(315));

    private final Pose pickUp3SamplePose = new Pose(15, 130, Math.toRadians(25));

    private final Pose move3SamplePose = new Pose(15, 130, Math.toRadians(35));

    private final Pose put3SamplePose = new Pose(17, 126, Math.toRadians(315));

    private final Pose pickUp4SamplePose = new Pose(25, 130, Math.toRadians(25));

    private final Pose move4SamplePose = new Pose(25, 130, Math.toRadians(35));

    private final Pose put4SamplePose = new Pose(17, 126, Math.toRadians(315));

    private final Pose pickUp5SamplePose = new Pose(65, 100, Math.toRadians(270));
    private final Pose pickUp5SampleControlPose = new Pose(65, 120, Math.toRadians(275));

    private final Pose put5SamplePos = new Pose(15, 130, Math.toRadians(315));
    private final Pose put5SampleControlPose = new Pose(65, 120, Math.toRadians(275));

    private Path scorePreload, PickSample2, MoveSample2, PutSample2, PickSample3, MoveSample3, PutSample3, PickSample4, MoveSample4, PutSample4,
            PickSample5, PutSample5;

    public void buildPaths() {
        scorePreload = new Path(new BezierCurve(new Point(startPose), new Point(putSample1ControlPose), new Point(putSample1Pose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), putSample1Pose.getHeading());

        PickSample2 = new Path(new BezierLine(new Point(putSample1Pose), new Point(pickUp2SamplePose)));
        PickSample2.setLinearHeadingInterpolation(putSample1Pose.getHeading(), pickUp2SamplePose.getHeading());

        MoveSample2 = new Path(new BezierLine(new Point(pickUp2SamplePose), new Point(move2SamplePose)));
        MoveSample2.setLinearHeadingInterpolation(pickUp2SamplePose.getHeading(), move2SamplePose.getHeading());

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

        pathCommand = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        pedroSb.followPathCmd(scorePreload),
                        etesito.extendArmHighBasketCmd()
                ),

                etesito.intakeSb.crservoCMD(-1),

                new WaitCommand(500),

                etesito.contractArmBasketCmd(),

                //PUT_FIRST_SAMPLE

                new ParallelDeadlineGroup(
                        pedroSb.followPathCmd(PickSample2),
                        etesito.armSb.armToPosSmooth(0, 0.7),
                        etesito.rodeSb.rodeToPos(-400)

                        ),

                etesito.intakeSb.crservoCMD(1),
                new WaitCommand(50),

                pedroSb.followPathCmd(MoveSample2),
                etesito.rodeSb.rodeToPos(-500),

                new WaitCommand(100),

                etesito.wristSb.servoPosCMD(basketWristPos),
                etesito.rodeSb.rodeToPos(0),

                new WaitCommand(200),

                new ParallelCommandGroup(
                        pedroSb.followPathCmd(PutSample2),
                        etesito.extendArmHighBasketCmd()
                ),

                etesito.intakeSb.crservoCMD(-1),

                new WaitCommand(500),

                etesito.contractArmBasketCmd(),

                //PUT_SECOND_SAMPLE

                new ParallelDeadlineGroup(
                        pedroSb.followPathCmd(PickSample3),
                        etesito.armSb.armToPosSmooth(0, 0.7),
                        etesito.rodeSb.rodeToPos(-400)

                ),

                etesito.intakeSb.crservoCMD(1),
                new WaitCommand(50),

                pedroSb.followPathCmd(MoveSample3),
                etesito.rodeSb.rodeToPos(-500),

                new WaitCommand(100),

                etesito.wristSb.servoPosCMD(basketWristPos),
                etesito.rodeSb.rodeToPos(0),

                new WaitCommand(200),

                new ParallelCommandGroup(
                        pedroSb.followPathCmd(PutSample3),
                        etesito.extendArmHighBasketCmd()
                ),

                etesito.intakeSb.crservoCMD(-1),

                new WaitCommand(500),

                etesito.contractArmBasketCmd(),

                //PUT_THIRD_SAMPLE

                new ParallelDeadlineGroup(
                        pedroSb.followPathCmd(PickSample4),
                        etesito.armSb.armToPosSmooth(0, 0.7),
                        etesito.rodeSb.rodeToPos(-400)

                ),
                etesito.intakeSb.crservoCMD(1),
                new WaitCommand(50),

                pedroSb.followPathCmd(MoveSample4),
                etesito.rodeSb.rodeToPos(-500),

                new WaitCommand(100),

                etesito.wristSb.servoPosCMD(basketWristPos),
                etesito.rodeSb.rodeToPos(0),

                new WaitCommand(200),

                new ParallelCommandGroup(
                        pedroSb.followPathCmd(PutSample4),
                        etesito.extendArmHighBasketCmd()
                ),

                etesito.intakeSb.crservoCMD(-1),

                new WaitCommand(500),

                etesito.subContractArmBasketCmd(),

                //PUT_FOUR_SAMPLE

                new ParallelDeadlineGroup(
                    pedroSb.followPathCmd(PickSample5),
                        etesito.armSb.armToPosSmooth(0, 0.7)
                ),

                etesito.intakeSb.crservoCMD(1)














        );
    }

    @Override
    public void init() {
        etesito.init(hardwareMap, true, true);

        etesito.armSb.setDefaultCommand(etesito.armSb.armUpdate());
        etesito.rodeSb.setDefaultCommand(etesito.rodeSb.rodeUpdate());

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        pedroSb = new PedroSubsystem(follower);
        CommandScheduler.getInstance().registerSubsystem(pedroSb);

        buildPaths();

        etesito.initCmd().schedule();
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
        etesito.headingAuto = etesito.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        pathCommand.cancel();
    }
}