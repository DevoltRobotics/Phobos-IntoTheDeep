package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Comands.Constants.ScaleFactor;
import static org.firstinspires.ftc.teamcode.Comands.Constants.downWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.getTargetExtensionFromY;
import static org.firstinspires.ftc.teamcode.Comands.Constants.pickSubWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.preSubmRodePos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.rodeInToTicks;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Comands.PIDFController;
import org.firstinspires.ftc.teamcode.subsystems.Vision.CrosshairVision;
import org.opencv.core.RotatedRect;

public class RodeSb extends SubsystemBase {

    DcMotorEx rodeMotor;
    PIDFController rodeController;

    public RodeSb(DcMotorEx rodeMotor, PIDFController rodeController) {
        this.rodeMotor = rodeMotor;
        this.rodeController = rodeController;

    }

    public Command rodeToPosVision(CrosshairVision vision, Telemetry telemetry, int offset, ServoSb wristSb) {
        return new SequentialCommandGroup(
                new RodeToPosVision(vision, telemetry, offset),
                wristSb.servoSmoothCMD(pickSubWristPos, 0.2),

                new WaitCommand(250),

                new ParallelCommandGroup(
                        rodeToPosPlusSmooth(400, 0.5),
                        new SequentialCommandGroup(
                                new WaitCommand(250),
                                wristSb.servoSmoothCMD(downWristPos, 0.15)

                                )),

                new WaitCommand(100),

                rodeToPosPlusSmooth(-400, 0.5),

                new WaitCommand(200)

        );

    }

    public Command rodeUpdate() {
        return new RodeUpdate();
    }

    public Command rodeToPosSmooth(int target, double timeSeconds) {
        return new RodeToPos(target, true, timeSeconds);
    }

    public Command rodeToPosPlus(int target) {
        return new RodeToPosPlus(target, false);
    }

    public Command rodeToPosPlusSmooth(int target, double timeSeconds) {
        return new RodeToPosPlus(target, true, timeSeconds);
    }

    public Command rodeToPos(int target) {
        return new RodeToPos(target, false);
    }

    class RodeToPosPlus extends CommandBase {

        int targetPos;
        int extra;

        int rodeTarget;

        boolean smooth;

        double timeSeconds;
        int currentTicks;
        ElapsedTime timer = null;

        public RodeToPosPlus(int extra, boolean smooth) {
            this.extra = extra;
            this.smooth = smooth;
            addRequirements(RodeSb.this);
        }

        public RodeToPosPlus(int extra, boolean smooth, double timeSeconds) {
            this.extra = extra;
            this.smooth = smooth;
            this.timeSeconds = timeSeconds;
            addRequirements(RodeSb.this);
        }

        @Override
        public void initialize() {
            currentTicks = rodeMotor.getCurrentPosition();
            timer = new ElapsedTime();
        }

        double lerp(double start, double end, double t) {
            return start * (1 - t) + end * t;
        }

        @Override
        public void execute() {
            targetPos = currentTicks + extra;

            if (smooth) {
                double t = Range.clip(timer.seconds() / timeSeconds, 0, 1);
                rodeTarget = (int) lerp(currentTicks, targetPos, t);

            } else {
                rodeTarget = targetPos;
            }

            rodeController.targetPosition = rodeTarget;
        }

        @Override
        public boolean isFinished() {
            if (smooth) {
                return timer.seconds() >= timeSeconds;
            } else {
                return true;
            }
        }

        @Override
        public void end(boolean interrupted) {
        }
    }

    class RodeToPos extends CommandBase {
        int targetPos;

        int rodeTarget;

        boolean smooth;

        double timeSeconds;
        int currentTicks;
        ElapsedTime timer = null;

        public RodeToPos(int targetPos, boolean smooth) {
            this.targetPos = targetPos;
            this.smooth = smooth;
            addRequirements(RodeSb.this);
        }

        public RodeToPos(int targetPos, boolean smooth, double timeSeconds) {
            this.targetPos = targetPos;
            this.smooth = smooth;
            this.timeSeconds = timeSeconds;
            addRequirements(RodeSb.this);
        }

        double lerp(double start, double end, double t) {
            return start * (1 - t) + end * t;
        }

        @Override
        public void initialize() {
            timer = new ElapsedTime();
            currentTicks = rodeMotor.getCurrentPosition();
        }

        @Override
        public void execute() {
            if (smooth) {
                double t = Range.clip(timer.seconds() / timeSeconds, 0, 1);
                rodeTarget = (int) lerp(currentTicks, targetPos, t);

            } else {
                rodeTarget = targetPos;

            }

            rodeController.targetPosition = rodeTarget;
        }

        @Override
        public boolean isFinished() {
            if (smooth) {
                return timer.seconds() >= timeSeconds;
            } else {
                return true;
            }

        }

        @Override
        public void end(boolean interrupted) {
        }
    }

    class RodeToPosVision extends CommandBase {
        boolean targeteado = false;
        int rodeTarget;

        int beforeTicks;
        ElapsedTime timer = null;

        CrosshairVision vision;

        Telemetry telemetry;

        double error;

        double targetFinalRode;

        int offset;

        public RodeToPosVision(CrosshairVision vision, Telemetry telemetry, int offset) {
            this.vision = vision;
            this.telemetry = telemetry;
            this.offset = offset;

            addRequirements(RodeSb.this);
        }

        @Override
        public void initialize() {
            timer = new ElapsedTime();
            beforeTicks = rodeMotor.getCurrentPosition();
        }

        /*@Override
        public void execute() {
            RotatedRect[] rects = vision.getLastRects();

            for (int i = 0; i < rects.length; i++) {
                telemetry.addData("detection #" + i, rects[i].center);
            }

            int targetY;
            RotatedRect rect = new RotatedRect();
            if (rects.length > 0) {
                rect = rects[0];
                targetY = (int)rect.center.y;
            }else {
                targetY = 220;
            }

            double yError  = 220 - targetY;
            double tagetYIn = yError / ScaleFactor;
            double tYRode = -(tagetYIn * rodeInToTicks);

            if (!targeteado) {
                rodeTarget =  Range.clip((int)(beforeTicks + tYRode), preSubmRodePos - 450, preSubmRodePos);
                rodeController.targetPosition = rodeTarget;

                targeteado = true;
            }

            error = Math.abs(rodeTarget - rodeMotor.getCurrentPosition());

            telemetry.addData("tyR", tYRode);
            telemetry.addData("rdTarget", rodeTarget);
            telemetry.addData("current", rodeMotor.getCurrentPosition());
            telemetry.addData("error", error);

        }

*/

        @Override
        public void execute() {
            RotatedRect rect = vision.getRect();

            int targetY;
            if (rect != null) {
                targetY = (int) rect.center.y + offset;
            } else {
                targetY = 240;
            }

            if (!targeteado) {
                targetFinalRode = getTargetExtensionFromY(targetY, rodeMotor.getCurrentPosition());
                ;
                rodeController.targetPosition = targetFinalRode;

                targeteado = true;
            }

            error = Math.abs(targetFinalRode - rodeMotor.getCurrentPosition());

            telemetry.addData("tyR", targetFinalRode);
            telemetry.addData("rdTarget", rodeTarget);
            telemetry.addData("current", rodeMotor.getCurrentPosition());
            telemetry.addData("error", error);

        }


        @Override
        public boolean isFinished() {
            return timer.seconds() >= 0.3 || error <= 30;

        }

        @Override
        public void end(boolean interrupted) {
        }
    }


    class RodeUpdate extends CommandBase {

        public RodeUpdate() {
            addRequirements(RodeSb.this);
        }

        @Override
        public void execute() {
            rodeMotor.setPower(rodeController.update(rodeMotor.getCurrentPosition()) * 0.09);

        }

    }


}
