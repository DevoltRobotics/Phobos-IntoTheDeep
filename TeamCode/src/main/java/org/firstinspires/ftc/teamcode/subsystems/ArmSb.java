package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Comands.PIDFController;

public class ArmSb extends SubsystemBase {

    DcMotorEx armMotor;
    PIDFController armController;

    public ArmSb(DcMotorEx armMotor, PIDFController armController){
        this.armMotor = armMotor;
        this.armController = armController;
    }

    public Command armToPosSmooth(int target, double timeSecond) {
        return new ArmToPos(target, true, timeSecond);
    }

    public Command armToPos(int target) {
        return new ArmToPos(target, false);
    }

    public Command armUpdate() {
        return new ArmUpdate();
    }

    class ArmToPos extends CommandBase {
        int targetPos;

        int armTarget;

        boolean smooth;

        double timeSeconds;
        int currentTicks;
        ElapsedTime timer = null;

        public ArmToPos(int targetPos, boolean smooth) {
            this.targetPos = targetPos;
            this.smooth = smooth;
            addRequirements(ArmSb.this);
        }

        public ArmToPos(int targetPos, boolean smooth, double timeSeconds) {
            this.targetPos = targetPos;
            this.smooth = smooth;
            this.timeSeconds = timeSeconds;
            addRequirements(ArmSb.this);
        }

        double lerp(double start, double end, double t) {
            return start * (1 - t) + end * t;
        }

        @Override
        public void initialize() {
            timer = new ElapsedTime();
            currentTicks = armMotor.getCurrentPosition();
        }

        @Override
        public void execute() {
            if (smooth){
                double t = Range.clip(timer.seconds() / timeSeconds, 0, 1);
                armTarget = (int) lerp(currentTicks, targetPos, t);

            }else {
                armTarget = targetPos;

            }

            armController.targetPosition = armTarget;
            armMotor.setPower(-armController.update(armMotor.getCurrentPosition()) * 0.4);

        }

        @Override
        public boolean isFinished() {
            if (smooth) {
                return timer.seconds() >= timeSeconds;
            }else return true;

        }

        @Override
        public void end(boolean interrupted) {
            //armMotor.setPower(-armController.update(armMotor.getCurrentPosition()) * 0.4);

        }
    }

    class ArmUpdate extends CommandBase {

        public ArmUpdate() {
            addRequirements(ArmSb.this);
        }

        @Override
        public void execute() {
            armMotor.setPower(-armController.update(armMotor.getCurrentPosition()) * 0.4);

        }
    }


}
