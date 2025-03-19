package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Comands.Etesito;
import org.firstinspires.ftc.teamcode.Comands.PIDFController;
@TeleOp
@Config
public class arm_test extends OpMode {

    Etesito etesito = new Etesito();

    double rodeMin = Double.MIN_VALUE;
    double rodeMax = Double.MAX_VALUE;

    double armMin = Double.MIN_VALUE;
    double armMax = Double.MAX_VALUE;

    public static PIDFController.PIDCoefficients ArmtestPID = new PIDFController.PIDCoefficients(0.0015, 0, 0.017);

    PIDFController armcontroller = new PIDFController(ArmtestPID);

    public static PIDFController.PIDCoefficients RodetestPID = new PIDFController.PIDCoefficients(0.015, 0, 0.007);

    PIDFController rodecontroller = new PIDFController(RodetestPID);

    private boolean extended = false;

    public static int ratio = 8;

    public static int rodeTarget = -1500;

    public double powerRode;

    public double powerArm = 0;


    @Override
    public void init() {

        etesito.init(hardwareMap, false, true);

        armcontroller.reset();
        rodecontroller.reset();

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        etesito.wrist.setPosition(0.5);

        rodeMin = -1300;

        rodeMax = 0;
    }

    int beforeArmPos = 0;

    @Override
    public void loop() {



        etesito.armMotor.setPower(-armcontroller.update(etesito.armMotor.getCurrentPosition()) * powerArm);

        if (gamepad2.y && !extended) {
            powerArm = 0.4;
            armcontroller.targetPosition = -1900;
            rodecontroller.reset();
            etesito.wrist.setPosition(0);

        } else if (gamepad2.a && !extended) {
            powerArm = 0.1;
            armcontroller.targetPosition = 0;
            rodecontroller.reset();
            armcontroller.reset();
            etesito.wrist.setPosition(0.6);

        } else if (gamepad2.b && !extended) {
            powerArm = 0.4;
            armcontroller.targetPosition = -950;
            rodecontroller.reset();
            etesito.wrist.setPosition(0.5);

        }

        if (gamepad2.x){
            etesito.wrist.setPosition(0.5);

        }

        double deltaArmPos = etesito.armMotor.getCurrentPosition() - beforeArmPos;

        ///////////////////////////////////////////////////////////////////////////7

        powerRode = rodecontroller.update(etesito.rodeMotor.getCurrentPosition()) * 0.08;

        etesito.rodeMotor.setPower(Range.clip(powerRode, rodeMin, rodeMax));

        if (gamepad2.dpad_down) {
            rodeMin = -0.5;
            rodecontroller.targetPosition = 0;
        } else if (gamepad2.dpad_up) {
            rodeMin = -1;
            rodecontroller.targetPosition = rodeTarget;

        }

        rodeMax = 1;

        if (gamepad2.left_bumper){
            etesito.claw.setPosition(0.7);

        }else if (gamepad2.right_bumper){
            etesito.claw.setPosition(1);

        }

        if (rodecontroller.targetPosition < 25){
            extended = false;

        } else{
            extended = true;

        }

        rodecontroller.targetPosition += deltaArmPos / ratio;

        rodecontroller.targetPosition += gamepad2.right_stick_y * 20;

        telemetry.addData("rdTarget", rodeTarget);
        telemetry.addData("ratio", ratio);

        telemetry.addLine("-----------");

        telemetry.addData("rodepower", etesito.rodeMotor.getPower());
        telemetry.addData("rodePos", etesito.rodeMotor.getCurrentPosition());
        telemetry.addData("rodeTargetPos", rodecontroller.targetPosition);

        telemetry.addLine("-----------");

        telemetry.addData("armpower", etesito.armMotor.getPower());
        telemetry.addData("armPos", etesito.armMotor.getCurrentPosition());
        telemetry.addData("armTargetPos", armcontroller.targetPosition);

        ///////////////////////////////////////////////////////////////////////////

        beforeArmPos = etesito.armMotor.getCurrentPosition();

    }

}
