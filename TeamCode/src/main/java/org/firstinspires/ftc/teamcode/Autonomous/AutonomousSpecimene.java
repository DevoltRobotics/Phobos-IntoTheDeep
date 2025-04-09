package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Comands.Constants.contractWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.firstSpecimenArmPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.firstSpecimenRodePos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.firstSpecimenWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.openClawPos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Comands.Etesito;
import org.firstinspires.ftc.teamcode.subsystems.PedroSb;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous
public class AutonomousSpecimene extends OpMode {

    private Follower follower;
    private Timer actionTimer;
    private Timer opmodeTimer;

    PedroSb pedroSb;

    Etesito etesito = new Etesito();

    private Command pathCommand;

    private int pathState;

    //This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>

    private final Pose startPose = new Pose(8.5, 79.5, Math.toRadians(0));

    private final Pose putSpecimen1 = new Pose(28, 79, Math.toRadians(0));

    private final Pose pickup1SamplePose = new Pose(36, 38, Math.toRadians(300));

    private final Pose pickup1PoseControl = new Pose(22, 53, Math.toRadians(260));

    private final Pose put1SamplePose = new Pose(27, 37, Math.toRadians(260));

    private final Pose pickup2SamplePose = new Pose(38, 28, Math.toRadians(300));

    private final Pose put2SamplePose = new Pose(27, 27, Math.toRadians(260));

    private final Pose pickup3SamplePose = new Pose(30, 27, Math.toRadians(295));

    private final Pose pick2specimenyput3samplePose = new Pose(16, 35, Math.toRadians(180));

    private final Pose put2specimenPose = new Pose(41, 75, Math.toRadians(180));

    private final Pose pick3specimenPose = new Pose(10, 30, Math.toRadians(180));

    private final Pose put3specimenPose = new Pose(41, 72, Math.toRadians(180));

    private final Pose pick4specimenPose = new Pose(10, 30, Math.toRadians(180));

    private final Pose put4specimenPose = new Pose(41, 69, Math.toRadians(180));

    private final Pose pick5specimenPose = new Pose(10, 30, Math.toRadians(180));

    private final Pose put5specimenPose = new Pose(41, 66, Math.toRadians(180));

    private Path scorePreload, PickSample1, PutSample1, PickSample2, PutSample2, PickSample3, PutSample3yPickSpecimen2,
            PutSpecimen2, PickSpecimen3, putSpecimen3, pickSpecimen4, putSpecimen4, pickSpecimen5, putSpecimen5;

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

        PutSample3yPickSpecimen2 = new Path(new BezierLine(new Point(pickup3SamplePose), new Point(pick2specimenyput3samplePose)));
        PutSample3yPickSpecimen2.setLinearHeadingInterpolation(pickup3SamplePose.getHeading(), pick2specimenyput3samplePose.getHeading());

        PutSpecimen2 = new Path(new BezierLine(new Point(pick2specimenyput3samplePose), new Point(put2specimenPose)));
        PutSpecimen2.setConstantHeadingInterpolation(put2specimenPose.getHeading());

        PickSpecimen3 = new Path(new BezierLine(new Point(put2specimenPose), new Point(pick3specimenPose)));
        PickSpecimen3.setConstantHeadingInterpolation(pick3specimenPose.getHeading());

        putSpecimen3 = new Path(new BezierLine(new Point(pick3specimenPose), new Point(put3specimenPose)));
        putSpecimen3.setConstantHeadingInterpolation(put3specimenPose.getHeading());

        pickSpecimen4 = new Path(new BezierLine(new Point(put3specimenPose), new Point(pick4specimenPose)));
        pickSpecimen4.setConstantHeadingInterpolation(pick4specimenPose.getHeading());

        putSpecimen4 = new Path(new BezierLine(new Point(pick4specimenPose), new Point(put4specimenPose)));
        putSpecimen4.setConstantHeadingInterpolation(put4specimenPose.getHeading());

        pickSpecimen5 = new Path(new BezierLine(new Point(put4specimenPose), new Point(pick5specimenPose)));
        pickSpecimen5.setConstantHeadingInterpolation(pick5specimenPose.getHeading());

        putSpecimen5 = new Path(new BezierLine(new Point(pick5specimenPose), new Point(put5specimenPose)));
        putSpecimen5.setConstantHeadingInterpolation(put5specimenPose.getHeading());


        pathCommand = new SequentialCommandGroup(

                new ParallelDeadlineGroup(
                        pedroSb.followPathCmd(scorePreload),
                        etesito.wristSb.servoPosCMD(firstSpecimenWristPos),
                        etesito.armSb.armToPos(firstSpecimenArmPos),
                        etesito.rodeSb.rodeToPos(firstSpecimenRodePos)
                ),
                etesito.clawSb.servoPosCMD(openClawPos),
                new WaitCommand(100),
                etesito.wristSb.servoPosCMD(contractWristPos),
                etesito.rodeSb.rodeToPos(0),

                //PUT_FIRST_SPECIMEN

                new WaitCommand(500),

                new ParallelDeadlineGroup(
                        pedroSb.followPathCmd(PickSample1),
                        etesito.armSb.armToPosSmooth(0,1),
                        etesito.intakeSb.crservoCMD(1)
                ),

                etesito.rodeSb.rodeToPos(-500),

                new WaitCommand(300),
                etesito.wristSb.servoPosCMD(contractWristPos),
                etesito.rodeSb.rodeToPos(-800),
                pedroSb.followPathCmd(PutSample1),

                etesito.intakeSb.crservoCMD(-1),
                new WaitCommand(200)

                //FIRST_TO_HUMAN

                /*pedroSb.followPathCmd(PickSample2),
                etesito.abrahamSb.servoPosCMD(contractAbramPos),

                etesito.abrahamSb.servoPosCMD(midOpenAbramPos),
                new WaitCommand(100),

                pedroSb.followPathCmd(PutSample2),
                etesito.abrahamSb.servoPosCMD(openAbramPos),
                new WaitCommand(200),

                //SECOND_TO_HUMAN

                new ParallelDeadlineGroup(
                        pedroSb.followPathCmd(PickSample3),
                        etesito.downArm3rdSample()

                        ),

                etesito.rodeSb.rodeToPos(-900),
                new WaitCommand(500),

                new ParallelDeadlineGroup(
                        pedroSb.followPathCmd(PutSample3yPickSpecimen2),
                        etesito.rodeSb.rodeToPos(0)
                        ),

                //THIRD_TO_HUMAN && PICK_SECOND

                new ParallelDeadlineGroup(
                        etesito.pickSpecimenSeqCmd(),
                        etesito.intakeSb.crservoCMD(-1)
                        ),

                etesito.intakeSb.crservoCMD(0),
                pedroSb.followPathCmd(PutSpecimen2),

                etesito.putSpecimenSeqCmd(),

                new ParallelDeadlineGroup(
                        pedroSb.followPathCmd(PickSpecimen3),
                        etesito.rodeSb.rodeToPos(0),
                        etesito.armSb.armToPosSmooth(0, 0.5)
                )*/

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

        pedroSb = new PedroSb(follower);
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

        telemetry.addData("path state", pathState);
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