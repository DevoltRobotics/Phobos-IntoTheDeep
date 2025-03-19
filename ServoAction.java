package org.firstinspires.ftc.teamcode.Comands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoAction extends SubsystemBase {
    Servo servo1;
    Servo servo2;

    public ServoAction(Servo servo) {
        this.servo1 = servo;
    }

    public ServoAction(Servo servo1, Servo servo2) {
        this.servo1 = servo1;
        this.servo2 = servo2;
    }

    public Command servoPosCMD(double position) {
        return new ServoCMD(position);
    }

    public Command mirrorServoPosCMD(double position) {
        return new MirrorServoCMD(position);
    }

    class ServoCMD extends CommandBase {
        double position;

        public ServoCMD(double position) {
            this.position = position;
            addRequirements(ServoAction.this);
        }

        @Override
        public void execute() {
            servo1.setPosition(position);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    class MirrorServoCMD extends CommandBase {
        double position;

        public MirrorServoCMD(double position) {
            this.position = position;
            addRequirements(ServoAction.this);
        }

        @Override
        public void execute() {
            servo1.setPosition(0.5 + position);
            servo2.setPosition(0.5 - position);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

}
