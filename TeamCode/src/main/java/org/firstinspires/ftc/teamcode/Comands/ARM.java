package org.firstinspires.ftc.teamcode.Comands;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ARM {

    public static PIDCoefficients testPID = new PIDCoefficients(0, 0 , 0);

    public static ElapsedTime PID_timer = new ElapsedTime();

    static double integral = 0, TPR = 8192, pos = 0;

    static double Max_angle = 90;

    static double Min_angle = 0.1;

    public static double range (double ticks){

        if (tick_To_Angle(ticks) > Max_angle){
            ticks = Max_angle;

        } else if (tick_To_Angle(ticks) < Min_angle){
            ticks = Min_angle;
        }

        return ticks;

    }

    public static void move_Arm_Pos (double TargetPos) {

        double error = Obot.arm_pos;
        double lastError = 0;

        range(TargetPos);

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
            PID_timer.reset();

        }

        if (TargetPos > tick_To_Angle(Max_angle)){

            TargetPos =-1;

        }

    }

    public static void move_Arm_Man (double Power) {

        double error = Obot.arm_pos;
        double lastError = 0;

        while(Math.abs(error) <= 9){

            Obot.ARM_RWE();

            error = Obot.arm_pos - (Power * TPR / 4);
            double changeInError = lastError - error;
            double derivative = changeInError / PID_timer.time();

            double D = testPID.d * derivative;
            Obot.armMotor.setPower(Power * D);
            error = lastError;
            PID_timer.reset();

        }

    }

    public static double tick_To_Angle (double angle){

        pos = angle * (TPR / 360);

        return pos;
    }



    



}
