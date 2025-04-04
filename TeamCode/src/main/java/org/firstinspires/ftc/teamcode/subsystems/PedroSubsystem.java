package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class PedroSubsystem extends SubsystemBase {

    Follower follower;

    public PedroSubsystem(Follower follower) {
        this.follower = follower;
    }

    @Override
    public void periodic() {
        follower.update();
    }

    public Command followPathCmd(Path path) {
        return new FollowPathCmd(path);
    }

    public Command turn(double radians) {
        return new Turn(radians);
    }

    public Command followPathCmd(PathChain path) {
        return new FollowPathChainCmd(path);
    }

    class FollowPathCmd extends CommandBase {
        Path path;

        FollowPathCmd(Path path) {
            this.path = path;
            addRequirements(PedroSubsystem.this);
        }

        @Override
        public void initialize() {
            follower.followPath(path);
        }

        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }

    class FollowPathChainCmd extends CommandBase {
        PathChain path;

        FollowPathChainCmd(PathChain path) {
            this.path = path;
            addRequirements(PedroSubsystem.this);
        }

        @Override
        public void initialize() {
            follower.followPath(path, true);
        }

        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }

    class Turn extends CommandBase {
        double radians;

        Turn(double radians) {
            this.radians = radians;
            addRequirements(PedroSubsystem.this);
        }

        @Override
        public void initialize() {
            follower.turnTo(radians);
        }

        @Override
        public boolean isFinished() {
            return !follower.isTurning();
        }
    }

    class HoldPoint extends CommandBase {
        BezierPoint holdPoint;
        double heading;


        public HoldPoint(BezierPoint holdPoint, double heading) {
            this.holdPoint = holdPoint;
            this.heading = heading;
        }

        public HoldPoint(Point holdPoint, double heading) {
            this(new BezierPoint(holdPoint), heading);
        }

        public HoldPoint(Pose holdPose) {
            this(new BezierPoint(new Point(holdPose)), holdPose.getHeading());
        }

        @Override
        public void initialize() {
            follower.holdPoint(holdPoint, heading);
        }

        @Override
        public void end(boolean interrupted) {
            follower.breakFollowing();
        }
    }

}