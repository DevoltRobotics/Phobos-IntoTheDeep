package org.firstinspires.ftc.teamcode.Comands;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import java.util.Set;

public class ArmSubsystem extends OpMode {

    public static PIDCoefficients testPID = new PIDCoefficients(0,0,0);

    double downWristPosition = 0.53;

    private final ArmController controller;

    public ArmSubsystem(ArmController controller) {
        this.controller = controller;
    }

    private ArmController.PIDCoefficients previousCoeffs  = new ArmController.PIDCoefficients();


    @Override
    public void init() {

        ArmController.targetPosition = Obot.arm_pos;

    }

    @Override
    public void loop() {
    }

    void updateController(){

        Obot.armMotor.setPower((int) ArmController.update(Obot.arm_pos));

    }



    void resetArm(){

        Obot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Obot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    private class datos {

        double kV = 0.0;
        double kA = 0.0;
        double kStatic = 0.0;

        double drivingTicksPerSecond = 320;
        double ticksPerSecond = 100;
        double ticksPerPerSecond = 60;

    }
}
