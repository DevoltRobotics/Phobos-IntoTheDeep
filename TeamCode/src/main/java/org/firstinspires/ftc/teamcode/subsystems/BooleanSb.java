package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;

public class BooleanSb extends SubsystemBase {

    public Command booleanCmd(boolean boleano, boolean target){
        return new BooleanCMD(boleano, target);
    }

    public Command numberCmd(int number, int target){
        return new NumberCMD(number, target);
    }

    class BooleanCMD extends CommandBase {
        boolean boleano;

        boolean target;

        public BooleanCMD(boolean boleano, boolean target) {
            this.boleano = boleano;
            this.target = target;
            addRequirements(BooleanSb.this);
        }

        @Override
        public void execute() {
            boleano = target;
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    class NumberCMD extends CommandBase {
        int number;

        int target;

        public NumberCMD(int number, int target) {
            this.number = number;
            this.target = target;
            addRequirements(BooleanSb.this);
        }

        @Override
        public void execute() {
            number = target;
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }


}
