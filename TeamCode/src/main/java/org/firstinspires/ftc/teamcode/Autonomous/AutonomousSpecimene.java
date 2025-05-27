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

//435 RPM
    /*
    int pickSample1RodePos = -1050;
    int putSample1RodePos = -600;

    int pickSample2RodePos = -1000;
    int putSample2RodePos = -600;

    int pickSample3RodePos = -1000;
    int putSample3RodePos = -600;

    int parkRodePos = -1000;

     */

    int pickSample1RodePos = -1470;
    int putSample1RodePos = -600;

    int prePickSample2RodePos = - 420;

    int pickSample2RodePos = -1400;
    int putSample2RodePos = -840;

    int prePickSample3RodePos = -700;

    int pickSample3RodePos = -1400;
    int putSample3RodePos = -840;

    int parkRodePos = -1000;

    PedroSb pedroSb;

    Etesito etesito = new Etesito();

    private Command pathCommand;

    private int pathState;

    //This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>

    private final Pose startPose = new Pose(8.5, 64, Math.toRadians(0));

    private final Pose putSpecimen1Pose = new Pose(32, 72, Math.toRadians(30));

    private final Pose pickUpSample1Pose = new Pose(32, 48, Math.toRadians(301));

    private final Pose putSample1Pose = new Pose(29, 40, Math.toRadians(230));

    private final Pose pickUpSample2Pose = new Pose(39, 38, Math.toRadians(277));

    private final Pose putSample2Pose = new Pose(27, 32, Math.toRadians(228));

    private final Pose pickUpSample3Pose = new Pose(42.5, 31, Math.toRadians(271));

    private final Pose putSample3Pose = new Pose(27, 32, Math.toRadians(230));

    private final Pose pickSpecimen2Pose = new Pose(18.5, 38, Math.toRadians(180));

    private final Pose rePickSpecimen2Pose = new Pose(17, 38, Math.toRadians(180));

    private final Pose putSpecimen2Pose = new Pose(39, 81, Math.toRadians(180));

    private final Pose pickSpecimen3Pose = new Pose(17, 39, Math.toRadians(180));

    private final Pose putSpecimen3Pose = new Pose(39, 80, Math.toRadians(180));

    private final Pose pickSpecimen4Pose = new Pose(17, 39, Math.toRadians(180));

    private final Pose putSpecimen4Pose = new Pose(39, 79, Math.toRadians(180));

    private final Pose pickSpecimen5Pose = new Pose(17, 39, Math.toRadians(180));

    private final Pose putSpecimen5Pose = new Pose(39, 78, Math.toRadians(180));

    private final Pose parkPose = new Pose(26, 64, Math.toRadians(245));

    private Path PutSpecimen1;

    private PathChain PickSample1, PutSample1, PickSample2, PutSample2, PickSample3, PutSample3, PickSpecimen2, RePickSpecimen2,
            PutSpecimen2, PickSpecimen3, PutSpecimen3, PickSpecimen4, PutSpecimen4, PickSpecimen5, PutSpecimen5, Park;

    public void buildPaths() {
        PutSpecimen1 = new Path(new BezierLine(new Point(startPose), new Point(putSpecimen1Pose)));
        PutSpecimen1.setLinearHeadingInterpolation(startPose.getHeading(), putSpecimen1Pose.getHeading());
        
        PickSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(putSpecimen1Pose), new Point(pickUpSample1Pose)))
                .setLinearHeadingInterpolation(putSpecimen1Pose.getHeading(), pickUpSample1Pose.getHeading())
                .build();

        PutSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUpSample1Pose), new Point(putSample1Pose)))
                .setPathEndTimeoutConstraint(1)
                .setLinearHeadingInterpolation(pickUpSample1Pose.getHeading(), putSample1Pose.getHeading())
                .build();

        PickSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(putSample1Pose), new Point(pickUpSample2Pose)))
                .setPathEndTimeoutConstraint(50)
                .setLinearHeadingInterpolation(putSample1Pose.getHeading(), pickUpSample2Pose.getHeading())
                .build();

        PutSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUpSample2Pose), new Point(putSample2Pose)))
                .setPathEndTimeoutConstraint(1)
                .setLinearHeadingInterpolation(pickUpSample2Pose.getHeading(), putSample2Pose.getHeading())
                .build();

        PickSample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(putSample2Pose), new Point(pickUpSample3Pose)))
                .setPathEndTimeoutConstraint(80)
                .setLinearHeadingInterpolation(putSample2Pose.getHeading(), pickUpSample3Pose.getHeading())
                .build();

        PutSample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUpSample3Pose), new Point(putSample3Pose)))
                .setPathEndTimeoutConstraint(1)
                .setLinearHeadingInterpolation(pickUpSample3Pose.getHeading(), putSample3Pose.getHeading())
                .build();

        PickSpecimen2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUpSample3Pose), new Point(pickSpecimen2Pose)))
                .setLinearHeadingInterpolation(putSample3Pose.getHeading(), pickSpecimen2Pose.getHeading())
                .build();

        RePickSpecimen2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickSpecimen2Pose), new Point(rePickSpecimen2Pose)))
                .setPathEndTimeoutConstraint(50)
                .setLinearHeadingInterpolation(pickSpecimen2Pose.getHeading(), rePickSpecimen2Pose.getHeading())
                .build();


        PutSpecimen2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickSpecimen2Pose), new Point(putSpecimen2Pose)))
                .setPathEndTimeoutConstraint(1)
                .setConstantHeadingInterpolation(putSpecimen2Pose.getHeading())
                .build();

        PickSpecimen3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(putSpecimen2Pose), new Point(pickSpecimen3Pose)))
                .setPathEndTimeoutConstraint(50)
                .setConstantHeadingInterpolation(pickSpecimen3Pose.getHeading())
                .build();

        PutSpecimen3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickSpecimen3Pose), new Point(putSpecimen3Pose)))
                .setPathEndTimeoutConstraint(1)
                .setConstantHeadingInterpolation(putSpecimen3Pose.getHeading())
                .build();

        PickSpecimen4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(putSpecimen3Pose), new Point(pickSpecimen4Pose)))
                .setPathEndTimeoutConstraint(50)
                .setConstantHeadingInterpolation(pickSpecimen4Pose.getHeading())
                .build();

        PutSpecimen4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickSpecimen4Pose), new Point(putSpecimen4Pose)))
                .setPathEndTimeoutConstraint(1)
                .setConstantHeadingInterpolation(putSpecimen4Pose.getHeading())
                .build();

        PickSpecimen5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(putSpecimen4Pose), new Point(pickSpecimen5Pose)))
                .setPathEndTimeoutConstraint(50)
                .setConstantHeadingInterpolation(pickSpecimen5Pose.getHeading())
                .build();

        PutSpecimen5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickSpecimen5Pose), new Point(putSpecimen5Pose)))
                .setPathEndTimeoutConstraint(1)
                .setConstantHeadingInterpolation(putSpecimen5Pose.getHeading())
                .build();

        Park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(putSpecimen5Pose), new Point(parkPose)))
                .setPathEndTimeoutConstraint(1)
                .setLinearHeadingInterpolation(putSpecimen5Pose.getHeading(), parkPose.getHeading())
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
                new WaitCommand(200),

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
                etesito.rodeSb.rodeToPos(prePickSample2RodePos),
                etesito.intakeSb.crservoCMD(1)
        ),

                etesito.wristSb.servoPosCMD(downWristPos),
                new WaitCommand(50),

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
                        etesito.rodeSb.rodeToPos(prePickSample3RodePos),
                        etesito.intakeSb.crservoCMD(1)
                ),

                etesito.wristSb.servoPosCMD(downWristPos),
                new WaitCommand(50),

                etesito.rodeSb.rodeToPosSmooth(pickSample3RodePos, 0.35),

                new WaitCommand(100),
                new ParallelCommandGroup(
                        etesito.intakeSb.crservoCMD(0),
                        etesito.rodeSb.rodeToPos(putSample3RodePos),
                        etesito.wristSb.servoPosCMD(contractWristPos),
                        pedroSb.followPathCmd(PutSample3)
                ),

                //THIRD_TO_HUMAN

                etesito.intakeSb.crservoCMD(-1),
                new WaitCommand(200),

                new ParallelCommandGroup(
                        etesito.intakeSb.crservoCMD(0),
                        etesito.rodeSb.rodeToPos(0),
                        etesito.wristSb.servoPosCMD(downWristPos),
                        pedroSb.followPathCmd(PickSpecimen2)
                ),

                pedroSb.followPathCmd(RePickSpecimen2),

                etesito.pickSpecimen1OneSeqCmd(),

                new ParallelCommandGroup(
                        pedroSb.followPathCmd(PutSpecimen2),
                        etesito.pickSpecimenTwoSeqCmd()
                ),

                //PUT_SECOND_SPECIMEN

                etesito.putSpecimenOneSeqCmd(),

                new ParallelCommandGroup(
                        pedroSb.followPathCmd(PickSpecimen3),
                        etesito.armSb.armToPosSmooth(0, 1)
                ),

                etesito.pickSpecimenOneSeqCmd(),

                new ParallelCommandGroup(
                        pedroSb.followPathCmd(PutSpecimen3),
                        etesito.pickSpecimenTwoSeqCmd()
                ),

                etesito.putSpecimenOneSeqCmd(),

                //PUT_THIRD_SPECIMEN

                new ParallelCommandGroup(
                        pedroSb.followPathCmd(PickSpecimen4),
                        etesito.armSb.armToPosSmooth(0, 1)
                ),

                etesito.pickSpecimenOneSeqCmd(),

                new ParallelCommandGroup(
                        pedroSb.followPathCmd(PutSpecimen4),
                        etesito.pickSpecimenTwoSeqCmd()
                ),

                etesito.putSpecimenOneSeqCmd(),

                //PUT_FOUR_SPECIMEN

                new ParallelCommandGroup(
                        pedroSb.followPathCmd(PickSpecimen5),
                        etesito.armSb.armToPosSmooth(0, 1)
                ),

                etesito.pickSpecimenOneSeqCmd(),

                new ParallelCommandGroup(
                        pedroSb.followPathCmd(PutSpecimen5),
                        etesito.pickSpecimenTwoSeqCmd()
                ),

                etesito.putSpecimenLastSeqCmd(),

                etesito.rodeSb.rodeToPos(0),

                new WaitCommand(150),

                etesito.wristSb.servoPosCMD(contractWristPos),
                etesito.armSb.armToPosSmooth(0, 0.6)

                //PUT_FIVE_SPECIMEN


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
        etesito.armMotor.setPower(-etesito.armController.update(etesito.armMotor.getCurrentPosition()) * 0.6);
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
        etesito.armMotor.setPower(-etesito.armController.update(etesito.armMotor.getCurrentPosition()) * 0.6);
        etesito.rodeMotor.setPower(etesito.rodeController.update(etesito.rodeMotor.getCurrentPosition()) * 0.09);

        CommandScheduler.getInstance().run();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

        Etesito.headingAuto = follower.getPose().getHeading();
    }

    @Override
    public void stop() {
        pathCommand.cancel();
    }
}