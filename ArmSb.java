package org.firstinspires.ftc.teamcode.Comands;

import static org.firstinspires.ftc.teamcode.Comands.Constants.ratioArm;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class ArmSb extends SubsystemBase {

    int beforeArmPos = 0;

    DcMotorEx rodeMotor;
    PIDFController rodeController;
    double rodeTarget;

    DcMotorEx arm;
    PIDFController armController;
    double armTarget;

    public ArmSb(DcMotorEx rodeMotor, PIDFController rodeController, DcMotorEx arm, PIDFController armController){
        this.rodeMotor = rodeMotor;
        this.rodeController = rodeController;

        this.arm = arm;
        this.armController = armController;

    }

    public Command rodeToPosSmooth(int target, double timeSeconds) {
        return new RodeToPosSmooth(target, timeSeconds);
    }

    public Command rodeToPos(int target) {
        return new RodeToPos(target);
    }


    public Command armToPosSmooth(int target, double timeSeconds) {
        return new ArmToPosSmooth(target, timeSeconds);
    }

    public Command armToPos(int target) {
        return new ArmToPos(target);
    }

    public Command armUpdate() {
        return new ArmUpdate();
    }

    class ArmUpdate extends CommandBase {

        public ArmUpdate() {
            addRequirements(ArmSb.this);
        }

        @Override
        public void execute() {
            arm.setPower(-armController.update(rodeMotor.getCurrentPosition()) * 0.4);
            armController.targetPosition = armTarget;

            rodeMotor.setPower(rodeController.update(rodeMotor.getCurrentPosition()) * 0.09);
            rodeController.targetPosition = rodeTarget;

            int deltaArmPos = (arm.getCurrentPosition() - beforeArmPos);

            rodeTarget += ((double) deltaArmPos / ratioArm);
            beforeArmPos = arm.getCurrentPosition();

        }

    }

    class RodeToPos extends CommandBase {
        int targetPos;

        public RodeToPos(int targetPos) {
            this.targetPos = targetPos;
            addRequirements(ArmSb.this);
        }

        @Override
        public void execute() {
            rodeTarget = targetPos;

        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    class RodeToPosSmooth extends CommandBase {
        int targetPos;
        double timeSeconds;

        int currentTicks;

        ElapsedTime timer = null;

        public RodeToPosSmooth(int targetPos, double timeSeconds) {
            this.targetPos = targetPos;
            this.timeSeconds = timeSeconds;
            addRequirements(ArmSb.this);
        }

        @Override
        public void initialize() {
            timer = new ElapsedTime();
            currentTicks = rodeMotor.getCurrentPosition();
        }

        @Override
        public void execute() {

            double t = Range.clip(timer.seconds() / timeSeconds, 0, 1);

            rodeTarget = (int) Etesito.lerp(currentTicks, targetPos, t);

        }

        @Override
        public boolean isFinished() {
            return timer.seconds() >= timeSeconds;
        }

        @Override
        public void end(boolean interrupted) {
            rodeTarget = targetPos;
        }
    }

    class ArmToPos extends CommandBase {
        int targetPos;

        public ArmToPos(int targetPos) {
            this.targetPos = targetPos;
            addRequirements(ArmSb.this);
        }

        @Override
        public void execute() {
            armTarget = targetPos;

        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    class ArmToPosSmooth extends CommandBase {
        int targetPos;
        double timeSeconds;

        int currentTicks;

        ElapsedTime timer = null;

        public ArmToPosSmooth(int targetPos, double timeSeconds) {
            this.targetPos = targetPos;
            this.timeSeconds = timeSeconds;
            addRequirements(ArmSb.this);
        }

        @Override
        public void initialize() {
            timer = new ElapsedTime();
            currentTicks = arm.getCurrentPosition();
        }

        @Override
        public void execute() {
            double t = Range.clip(timer.seconds() / timeSeconds, 0, 1);

            armTarget = (int) Etesito.lerp(currentTicks, targetPos, t);

        }

        @Override
        public boolean isFinished() {
            return timer.seconds() <= timeSeconds;
        }

        @Override
        public void end(boolean interrupted) {
            armTarget = targetPos;
        }
    }
}
