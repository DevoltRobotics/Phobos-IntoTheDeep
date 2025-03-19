package org.firstinspires.ftc.teamcode.Comands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CRservoAction extends SubsystemBase {

        CRServo servo;

        public CRservoAction(CRServo servo) {
            this.servo = servo;
        }

        public Command crservoCMD(double power) {
            return new CRServoCMD(power);
        }

        class CRServoCMD extends CommandBase {
            double power;

            public CRServoCMD(double power) {
                this.power = power;
                addRequirements(CRservoAction.this);
            }

            @Override
            public void execute() {
                servo.setPower(power);

            }

            @Override
            public boolean isFinished() {
                return true;
            }
        }
}
