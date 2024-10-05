package org.firstinspires.ftc.teamcode.Comands;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RODE {

    public static PIDCoefficients testPID = new PIDCoefficients(0, 0 , 0);

    public static ElapsedTime PID_timer = new ElapsedTime();

    static double repetitions = 0, integral = 0, TPR = 1024, pos;

    public static double Max_ext = 17.6;

    public static double Min_ext = 0;

    public static double pulley_cf = 3.3;

    public static void move_Arm_Pos (double TargetPos) {

        double error = Obot.arm_pos;
        double lastError = 0;

        while(Math.abs(error) <= 9){

            error = Obot.arm_pos - TargetPos;
            double changeInError = lastError - error;
            integral += changeInError * PID_timer.time();
            double derivative = changeInError / PID_timer.time();

            double P = testPID.p * error;
            double I = testPID.i * integral;
            double D = testPID.d * derivative;
            Obot.armMotor.setPower(P + I + D);
            error = lastError;
            repetitions ++;
            PID_timer.reset();

        }

    }

    public static void move_Arm_Man (double Power) {

        double error = Obot.extension;
        double lastError = 0;

        while(Math.abs(error) <= 9){

            error = Obot.extension - (Power * TPR / 4);
            double changeInError = lastError - error;

            double derivative = changeInError / PID_timer.time();

            double D = testPID.d * derivative;
            Obot.armMotor.setPower(Power * D);
            error = lastError;
            PID_timer.reset();

        }

    }

    public static double tick_To_Ext (double cm){

        pos = (cm/pulley_cf) * TPR;

        return pos;
    }



}
