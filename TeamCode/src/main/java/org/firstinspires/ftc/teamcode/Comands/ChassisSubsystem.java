package org.firstinspires.ftc.teamcode.Comands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.ArmSb;

public class ChassisSubsystem extends SubsystemBase {

    private PIDFController controller;

    private int ArmTarget;
    private int RodeTarget;
    private double powerArm;

    private int ratio = 8;

    Etesito etesito = new Etesito();

    ElapsedTime timer = new ElapsedTime();

    DcMotorEx fl, bl, br, fr;
    public ChassisSubsystem(PIDFController controller, DcMotorEx fl, DcMotorEx bl, DcMotorEx br, DcMotorEx fr) {
        this.fl = fl;
        this.bl = bl;
        this.br = br;
        this.fr = fr;

        this.controller = controller;

    }

    public Command moveChassis(double power, double timeTarget, double target, double headingDegrees ) {
        return new MoveChassis(power, timeTarget, target, headingDegrees);
    }

    class MoveChassis extends CommandBase {
        double target;
        double power;

        double error;

        double powerMult;
        ElapsedTime timer;
        double timeTarget;
        double headingDegrees;

        public MoveChassis(double power, double timeTarget, double target, double headingDegrees) {
            this.target = target;
            this.powerMult = power;
            this.timeTarget = timeTarget;
            this.headingDegrees = headingDegrees;

            addRequirements(ChassisSubsystem.this);

        }

        @Override
        public void initialize() {
            controller.targetPosition = target;
        }

        @Override
        public void execute() {
            if (timer == null) {
                timer = new ElapsedTime();
            }

            error = Math.abs(target) - Math.abs(headingDegrees);

            power = controller.update(headingDegrees) * powerMult;
            fl.setPower(power);
            bl.setPower(power);
            fr.setPower(-power);
            br.setPower(-power);

        }

        @Override
        public boolean isFinished() {
            return (timer.seconds() >= 0.4) || Math.abs(error) <= 0.3;
        }

        @Override
        public void end(boolean interrupted) {
            fl.setPower(0);
            bl.setPower(0);
            fr.setPower(0);
            br.setPower(0);

        }
    }

}