package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Comands.Etesito;

@Disabled
@TeleOp
public class test_color extends OpMode {

    Etesito etesito = new Etesito();

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {

        etesito.initSpecimen(hardwareMap);

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

    }

    @Override
    public void loop() {

        if(timer.seconds() > 0.2) {
            etesito.getColors();
            timer.reset();
        }

        etesito.colorTelemetry(telemetry);

        if (gamepad2.right_trigger > 0.5){
            etesito.pick();

        } else if (gamepad2.left_trigger > 0.5){
            etesito.open();

        }

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
