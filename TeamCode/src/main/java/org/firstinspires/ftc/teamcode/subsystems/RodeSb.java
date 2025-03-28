package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Comands.PIDFController;

public class RodeSb extends SubsystemBase {


    DcMotorEx rodeMotor;
    PIDFController rodeController;


    public RodeSb(DcMotorEx rodeMotor, PIDFController rodeController){
        this.rodeMotor = rodeMotor;
        this.rodeController = rodeController;

    }

    public Command rodeUpdate() {
        return new RodeUpdate();
    }

    public Command rodeToPosSmooth(int target, double timeSeconds) {
        return new RodeToPos(target, true, timeSeconds);
    }

    public Command rodeToPos(int target) {
        return new RodeToPos(target, false);
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
            if (smooth){
                double t = Range.clip(timer.seconds() / timeSeconds, 0, 1);
                rodeTarget = (int) lerp(currentTicks, targetPos, t);
                rodeMotor.setPower(rodeController.update(rodeMotor.getCurrentPosition()) * 0.09);

            }else {
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
