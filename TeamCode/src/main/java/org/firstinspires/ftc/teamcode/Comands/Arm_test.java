package org.firstinspires.ftc.teamcode.Comands;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

class Arm_test extends OpMode {

    PIDCoefficients testPID = new PIDCoefficients(0, 0 , 0);

    ElapsedTime PID_timer = new ElapsedTime();

    private double repetitions = 0;

    private double integral = 0;

    private void moveTestArm (double TargetPos) {

        double error = Obot.arm_pos;
        double lastError = 0;

        while(Math.abs(error) <= 9 && repetitions < 40){

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

    @Override
    public void init() {

        Obot.init(hardwareMap);

    }

    @Override
    public void loop() {

        if (gamepad2.a){

            moveTestArm(50);

        } else if (gamepad2.b){

            moveTestArm(0);
        }

    }
}
