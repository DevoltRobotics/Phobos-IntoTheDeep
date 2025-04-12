package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Comands.Constants.contractWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.downWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.firstSpecimenArmPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.firstSpecimenRodePos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.firstSpecimenWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.openClawPos;
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

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous
public class AutonomousSpecimene extends OpMode {

    private Follower follower;
    private Timer actionTimer;
    private Timer opmodeTimer;

    int pickSample1RodePos = -1000;
    int putSample1RodePos = -600;

    int pickSample2RodePos = -1000;
    int putSample2RodePos = -600;

    int pickSample3RodePos = -900;
    int putSample3RodePos = -800;

    PedroSb pedroSb;

    Etesito etesito = new Etesito();

    private Command pathCommand;

    private int pathState;

    //This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>

    private final Pose startPose = new Pose(8.5, 64, Math.toRadians(0));

    private final Pose putSpecimen1Pose = new Pose(26, 63, Math.toRadians(0));

    private final Pose pickUpSample1Pose = new Pose(30, 46, Math.toRadians(310));

    private final Pose putSample1Pose = new Pose(27, 38, Math.toRadians(230));

    private final Pose pickUpSample2Pose = new Pose(30, 36, Math.toRadians(305));

    private final Pose putSample2Pose = new Pose(27, 32, Math.toRadians(230));

    private final Pose pickUpSample3Pose = new Pose(32, 26, Math.toRadians(303));

    private final Pose pickSpecimen2yPutSample3Pose = new Pose(18.5, 37, Math.toRadians(180));
    private final Pose pickSpecimen2yPutSample3ControlPose = new Pose(34, 28, Math.toRadians(180));

    private final Pose putSpecimen2Pose = new Pose(41, 75, Math.toRadians(180));

    private final Pose pickSpecimen3Pose = new Pose(18, 38, Math.toRadians(180));

    private final Pose putSpecimen3Pose = new Pose(41, 72, Math.toRadians(180));

    private final Pose pickSpecimen4Pose = new Pose(12, 28, Math.toRadians(180));

    private final Pose putSpecimen4Pose = new Pose(41, 69, Math.toRadians(180));

    private final Pose pickSpecimen5Pose = new Pose(12, 28, Math.toRadians(180));

    private final Pose putSpecimen5Pose = new Pose(41, 66, Math.toRadians(180));

    private Path PutSpecimen1;

    private PathChain PickSample1, PutSample1, PickSample2, PutSample2, PickSample3, PickSpecimen2yPutSample3,
            PutSpecimen2, PickSpecimen3, PutSpecimen3, PickSpecimen4, PutSpecimen4, PickSpecimen5, PutSpecimen5;

    public void buildPaths() {
        PutSpecimen1 = new Path(new BezierLine(new Point(startPose), new Point(putSpecimen1Pose)));
        PutSpecimen1.setConstantHeadingInterpolation(putSpecimen1Pose.getHeading());


        PickSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(putSpecimen1Pose), new Point(pickUpSample1Pose)))
                .setLinearHeadingInterpolation(putSpecimen1Pose.getHeading(), pickUpSample1Pose.getHeading())
                .build();

        PutSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUpSample1Pose), new Point(putSample1Pose)))
                .setLinearHeadingInterpolation(pickUpSample1Pose.getHeading(), putSample1Pose.getHeading())
                .build();

        PickSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(putSample1Pose), new Point(pickUpSample2Pose)))
                .setLinearHeadingInterpolation(putSample1Pose.getHeading(), pickUpSample2Pose.getHeading())
                .build();

        PutSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUpSample2Pose), new Point(putSample2Pose)))
                .setLinearHeadingInterpolation(pickUpSample2Pose.getHeading(), putSample2Pose.getHeading())
                .build();

        PickSample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(putSample2Pose), new Point(pickUpSample3Pose)))
                .setLinearHeadingInterpolation(putSample2Pose.getHeading(), pickUpSample3Pose.getHeading())
                .build();

        PickSpecimen2yPutSample3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickUpSample3Pose), new Point(pickSpecimen2yPutSample3Pose)))
                .setLinearHeadingInterpolation(pickUpSample3Pose.getHeading(), pickSpecimen2yPutSample3Pose.getHeading())
                .build();

        PutSpecimen2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickSpecimen2yPutSample3Pose), new Point(putSpecimen2Pose)))
                .setConstantHeadingInterpolation(putSpecimen2Pose.getHeading())
                .build();

        PickSpecimen3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(putSpecimen2Pose), new Point(pickSpecimen3Pose)))
                .setConstantHeadingInterpolation(pickSpecimen3Pose.getHeading())
                .build();

        PutSpecimen3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickSpecimen3Pose), new Point(putSpecimen3Pose)))
                .setConstantHeadingInterpolation(putSpecimen3Pose.getHeading())
                .build();

        PickSpecimen4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(putSpecimen3Pose), new Point(pickSpecimen4Pose)))
                .setConstantHeadingInterpolation(pickSpecimen4Pose.getHeading())
                .build();

        PutSpecimen4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickSpecimen4Pose), new Point(putSpecimen4Pose)))
                .setConstantHeadingInterpolation(putSpecimen4Pose.getHeading())
                .build();

        PickSpecimen5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(putSpecimen4Pose), new Point(pickSpecimen5Pose)))
                .setConstantHeadingInterpolation(pickSpecimen5Pose.getHeading())
                .build();

        PutSpecimen5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickSpecimen5Pose), new Point(putSpecimen5Pose)))
                .setConstantHeadingInterpolation(putSpecimen5Pose.getHeading())
                .build();


        pathCommand = new SequentialCommandGroup(

                new ParallelDeadlineGroup(
                        pedroSb.followPathCmd(PutSpecimen1),
                        etesito.servosSb.mirrorServoPosCMD(servosHangingPos),
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

                new ParallelCommandGroup(
                        pedroSb.followPathCmd(PickSample1),
                        etesito.wristSb.servoPosCMD(downWristPos),
                        etesito.armSb.armToPosSmooth(0,1),
                        etesito.intakeSb.crservoCMD(1)
                ),

                etesito.rodeSb.rodeToPosSmooth(pickSample1RodePos, 0.5),

                new WaitCommand(100),

                new ParallelCommandGroup(
                        etesito.intakeSb.crservoCMD(0),
                        etesito.wristSb.servoPosCMD(contractWristPos),
                etesito.rodeSb.rodeToPos(putSample1RodePos),
                pedroSb.followPathCmd(PutSample1)
                        ),

                etesito.intakeSb.crservoCMD(-1),
                new WaitCommand(200),

                //FIRST_TO_HUMAN

                new ParallelDeadlineGroup(
                pedroSb.followPathCmd(PickSample2),
                etesito.rodeSb.rodeToPos(pickSample2RodePos + 700),
                etesito.intakeSb.crservoCMD(0)
        ),

                etesito.intakeSb.crservoCMD(1),
                etesito.wristSb.servoPosCMD(downWristPos),

                etesito.rodeSb.rodeToPosSmooth(pickSample2RodePos, 0.3),

                new WaitCommand(100),
                new ParallelCommandGroup(
                        etesito.intakeSb.crservoCMD(0),
                        etesito.wristSb.servoPosCMD(contractWristPos),
                        etesito.rodeSb.rodeToPos(putSample2RodePos),
                        pedroSb.followPathCmd(PutSample2)
                ),

                etesito.intakeSb.crservoCMD(-1),
                new WaitCommand(200),

                //SECOND_TO_HUMAN


                new ParallelDeadlineGroup(
                        pedroSb.followPathCmd(PickSample3),
                        etesito.rodeSb.rodeToPos(pickSample3RodePos + 700),
                        etesito.intakeSb.crservoCMD(0)
                ),

                etesito.intakeSb.crservoCMD(1),
                etesito.wristSb.servoPosCMD(downWristPos),

                etesito.rodeSb.rodeToPosSmooth(pickSample3RodePos, 0.3),

                new WaitCommand(100),
                new ParallelCommandGroup(
                        etesito.intakeSb.crservoCMD(0),
                        etesito.rodeSb.rodeToPos(0),
                        pedroSb.followPathCmd(PickSpecimen2yPutSample3)
                ),

                //THIRD_TO_HUMAN

                etesito.intakeSb.crservoCMD(-1),

                new WaitCommand(100),

                etesito.pickSpecimenOneSeqCmd(),


                new ParallelCommandGroup(
                        pedroSb.followPathCmd(PutSpecimen3),
                        etesito.pickSpecimenTwoSeqCmd()
                ),

                etesito.putSpecimenOneSeqCmd(),

                new ParallelCommandGroup(
                        pedroSb.followPathCmd(PickSpecimen3),
                        etesito.armSb.armToPosSmooth(0, 0.7)

                )
                );
    }

    @Override
    public void init() {
        etesito.init(hardwareMap, true, true);

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
        etesito.armMotor.setPower(-etesito.armController.update(etesito.armMotor.getCurrentPosition()) * 0.4);
        etesito.rodeMotor.setPower(etesito.rodeController.update(etesito.rodeMotor.getCurrentPosition()) * 0.09);
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
        etesito.armMotor.setPower(-etesito.armController.update(etesito.armMotor.getCurrentPosition()) * 0.4);
        etesito.rodeMotor.setPower(etesito.rodeController.update(etesito.rodeMotor.getCurrentPosition()) * 0.09);

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