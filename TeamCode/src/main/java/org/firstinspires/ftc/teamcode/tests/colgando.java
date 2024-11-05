package org.firstinspires.ftc.teamcode.tests;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Comands.Etesito;
import org.firstinspires.ftc.teamcode.Comands.PIDFController;


@TeleOp(name = "colgando")
public class colgando extends OpMode {

    private final Etesito etesito = new Etesito();

    public static PIDFController.PIDCoefficients armCoefficients = new PIDFController.PIDCoefficients(0.0015, 0, 0.0017);
    public static PIDFController.PIDCoefficients rodeCoefficients = new PIDFController.PIDCoefficients(0.015, 0, 0.007);

    PIDFController armcontroller = new PIDFController(armCoefficients);
    PIDFController rodecontroller = new PIDFController(rodeCoefficients);

    private boolean colgando;

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {

        etesito.init(hardwareMap);

        armcontroller.reset();
        rodecontroller.reset();

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );


        telemetry.addLine("Robot Initialized.");
        telemetry.update();

        colgando = false;


    }


    @Override
    public void loop() {

        etesito.rodeMotor.setPower(rodecontroller.update(etesito.rodeMotor.getCurrentPosition()) * 0.1);

        rodecontroller.targetPosition += gamepad2.right_stick_y * 30;

        //////////////////////////////////////////////////

        if (gamepad2.dpad_down) {
            etesito.C1.setPower(-1);
            etesito.C2.setPower(-1);

            colgando = true;


        } else if (gamepad2.dpad_up) {
            etesito.C1.setPower(1);
            etesito.C2.setPower(1);
            colgando = false;

        } else {
            etesito.C1.setPower(0);
            etesito.C2.setPower(0);

        }

        if (colgando && timer.seconds() > 1.7){
            etesito.servoC1.setPosition(0.1);
            etesito.servoC2.setPosition(0.9);

            colgando = false;

        }

        if (gamepad2.a) {
            etesito.servoC1.setPosition(0.7);
            etesito.servoC2.setPosition(0.3);

        } else if (gamepad2.b) {
            etesito.servoC1.setPosition(0.5);
            etesito.servoC2.setPosition(0.5);

        } else if (gamepad2.y) {
            etesito.servoC1.setPosition(0.1);
            etesito.servoC2.setPosition(0.9);

            //timer.reset();
            //servito = true;

        }

        ///////////////////////////////////////////////////////////////////////////

        if (gamepad2.dpad_left) {
            etesito.wrist.setPosition(0.5);

        } else if (gamepad2.dpad_right) {
            etesito.wrist.setPosition(0.6);

        }

        /////////////////////////////////////////////////////////////////7//////////////////////////////////////////

        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        double botHeading = etesito.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (gamepad1.left_stick_button) {
            etesito.imu.resetYaw();
        }

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        if (gamepad1.right_trigger > 0.5) {
            etesito.FL.setPower(frontLeftPower * 0.4);
            etesito.BL.setPower(backLeftPower * 0.4);
            etesito.FR.setPower(frontRightPower * 0.4);
            etesito.BR.setPower(backRightPower * 0.4);

        } else {
            etesito.FL.setPower(frontLeftPower);
            etesito.BL.setPower(backLeftPower);
            etesito.FR.setPower(frontRightPower);
            etesito.BR.setPower(backRightPower);

        }

    }

}


