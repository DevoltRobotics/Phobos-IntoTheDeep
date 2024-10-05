package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Comands.Obot;
import org.firstinspires.ftc.teamcode.Comands.ARM;


@TeleOp(name = "Mentita")
public class teleop1 extends OpMode {

    private final Obot robot = Obot.getInstance();

    private double loopTime = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private double endTime = 0;

    private boolean manual;
    private boolean tope;

    private boolean extendIntake = true;

    private double target_pos = 0;


    @Override
    public void init() {

        Obot.init(hardwareMap);

        telemetry.addLine("Robot Initialized.");
        telemetry.update();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));

    }


    @Override
    public void loop() {

        if (gamepad2.dpad_down) {

            target_pos = ARM.tick_To_Angle(0);

        } else if (gamepad2.dpad_right) {

            target_pos = ARM.tick_To_Angle(45);

        } else if (gamepad2.dpad_up) {

            target_pos = ARM.tick_To_Angle(90);

        }

        if (Math.abs(gamepad2.right_trigger) > 0.1 && Math.abs(gamepad2.right_stick_y) < 0.1) {

            manual = true;

        } else if (Math.abs(gamepad2.right_stick_y) > 0.1) {

            manual = false;

        }

        if (!manual) {

            ARM.move_Arm_Pos(target_pos);

        } else if (manual) {

            ARM.move_Arm_Man(gamepad2.right_stick_y);

        }

        if (Obot.tope()){

            Obot.ARM_SRE();
        }

        if (gamepad2.right_bumper){

            Obot.pick();

        } else if (gamepad2.left_bumper){

            Obot.open();

        }

        /////////////////////////////////////////////////////////////////7//////////////////////////////////////////

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double botHeading = Obot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (gamepad1.options) {
            Obot.imu.resetYaw();
        }

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        Obot.FL.setPower(frontLeftPower);
        Obot.BL.setPower(backLeftPower);
        Obot.FR.setPower(frontRightPower);
        Obot.BR.setPower(backRightPower);


    }
}






