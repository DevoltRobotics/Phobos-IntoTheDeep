package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class ServoSb extends SubsystemBase {
    Servo servo1;
    Servo servo2;

    public ServoSb(Servo servo) {
        this.servo1 = servo;
    }

    public ServoSb(Servo servo1, Servo servo2) {
        this.servo1 = servo1;
        this.servo2 = servo2;
    }

    public Command colorCmd(String target) {
        return new ColorCMD(target);
    }

    public Command servoPosCMD(double position) {
        return new ServoCMD(position);
    }

    public Command servoSmootrCMD(double position, double timeSeconds) {
        return new ServoSmootCMD(position, timeSeconds);
    }

    public Command mirrorServoPosCMD(double position) {
        return new MirrorServoCMD(position);
    }

    class ServoCMD extends CommandBase {
        double position;

        public ServoCMD(double position) {
            this.position = position;
            addRequirements(ServoSb.this);
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

    class ServoSmootCMD extends CommandBase {
        double timeSeconds;
        double currentPosition;

        double targetPos;
        double servoTarget;

        ElapsedTime timer = null;

        public ServoSmootCMD(double position, double timeSeconds) {
            this.targetPos = position;
            this.timeSeconds = timeSeconds;
            addRequirements(ServoSb.this);
        }

        double lerp(double start, double end, double t) {
            return start * (1 - t) + end * t;
        }

        @Override
        public void initialize() {
            timer = new ElapsedTime();
            currentPosition = servo1.getPosition();
        }

        @Override
        public void execute() {
            double t = Range.clip(timer.seconds() / timeSeconds, 0, 1);
            servoTarget = lerp(currentPosition, targetPos, t);

            servo1.setPosition(servoTarget);
        }

        @Override
        public boolean isFinished() {
            return timer.seconds() >= timeSeconds;
        }

    }

    class ColorCMD extends CommandBase {
        String target;

        public ColorCMD(String target) {
            this.target = target;
            addRequirements(ServoSb.this);
        }

        @Override
        public void execute() {
                switch (target) {
                    case "purple":
                        servo1.setPosition(0.72);
                        break;
                    case "blue":
                        servo1.setPosition(0.62);
                        break;
                    case "green":
                        servo1.setPosition(0.5);
                        break;
                    case "yellow":
                        servo1.setPosition(0.388);
                        break;
                    case "orange":
                        servo1.setPosition(0.33);
                        break;
                    case "red":
                        servo1.setPosition(0.28);
                        break;

            }        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    class MirrorServoCMD extends CommandBase {
        double position;

        public MirrorServoCMD(double position) {
            this.position = position;
            addRequirements(ServoSb.this);
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
