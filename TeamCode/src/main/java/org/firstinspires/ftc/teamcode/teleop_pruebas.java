package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Comands.Etesito;
import org.firstinspires.ftc.teamcode.Comands.PIDFController;



@TeleOp(name = "probando_ando")
public class teleop_pruebas extends OpMode {

    private final Etesito etesito = new Etesito();

    public static PIDFController.PIDCoefficients armCoefficients = new PIDFController.PIDCoefficients(0.0015, 0, 0.0017);
    public static PIDFController.PIDCoefficients rodeCoefficients = new PIDFController.PIDCoefficients(0.015, 0, 0.007);

    PIDFController armcontroller = new PIDFController(armCoefficients);
    PIDFController rodecontroller = new PIDFController(rodeCoefficients);

    private double down_ArmPos = 0;
    private double medium_Armpos = -1300;
    private double high_Armpos = -1900;

    private boolean colgando;

    private boolean extended;

    private int ratio = 8;

    private double powerArm;

    private double rdT;

    private boolean arm_Up;

    private boolean arm_Medium;

    private ElapsedTime movimientoBrazoExtenderTimer = new ElapsedTime();
    private boolean movimientoBrazoExtender = false;

    private ElapsedTime clawTimer = new ElapsedTime();
    private boolean claw_close = false;

    private ElapsedTime specimenTimer = new ElapsedTime();
    private boolean specimen = false;

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
        arm_Up = false;

    }

    int beforeArmPos = 0;

    @Override
    public void loop() {

        //////////////////////////////////////////////////

        etesito.armMotor.setPower(-armcontroller.update(etesito.armMotor.getCurrentPosition()) * powerArm);
        etesito.rodeMotor.setPower(rodecontroller.update(etesito.rodeMotor.getCurrentPosition()) * 0.08);

        if (gamepad2.y && !extended) {
            powerArm = 0.4;
            armcontroller.targetPosition = high_Armpos;
            rodecontroller.reset();
            etesito.wrist.setPosition(0.35);

            movimientoBrazoExtenderTimer.reset();
            movimientoBrazoExtender = true;

            arm_Up = true;
            arm_Medium = false;

        } else if (gamepad2.a && !extended) {
            powerArm = 0.1;
            armcontroller.targetPosition = down_ArmPos;
            rodecontroller.reset();
            armcontroller.reset();
            etesito.wrist.setPosition(0.5);
            etesito.claw.setPosition(0.7);

            arm_Up = false;
            arm_Medium = false;

        } else if (gamepad2.b && !extended) {
            powerArm = 0.4;
            armcontroller.targetPosition = medium_Armpos;
            rodecontroller.reset();
            etesito.wrist.setPosition(0.5);

            arm_Up = false;
            arm_Medium = true;

        }

        if(arm_Up) {
            rdT = -2100;

        } else {
            rdT = -1000;

        }

        if(movimientoBrazoExtender && movimientoBrazoExtenderTimer.seconds() >= 0.5) {

            movimientoBrazoExtender = false;
            rodecontroller.targetPosition = rdT;

        }

        if (gamepad2.dpad_down) {
            rodecontroller.targetPosition = 0;
            rodecontroller.reset();

            etesito.wrist.setPosition(0.5);

        } else if (gamepad2.dpad_up) {
            rodecontroller.targetPosition = rdT;

        }

        if (rodecontroller.targetPosition >= -200){
            extended = false;

        } else{
            extended = true;

        }

        double deltaArmPos = etesito.armMotor.getCurrentPosition() - beforeArmPos;

        rodecontroller.targetPosition += (deltaArmPos / ratio) + (gamepad2.right_stick_y * 30);

        ///////////////////////////////////////////////////////////////////////////

        if (gamepad2.dpad_right){
            etesito.wrist.setPosition(0.5);

        }else if (gamepad2.dpad_left){
            etesito.wrist.setPosition(0.6);

        }

        if (!arm_Medium && gamepad2.left_bumper){
            etesito.claw.setPosition(0.8);

            claw_close = false;

            specimenTimer.reset();
            specimen = true;

        }else if (gamepad2.right_bumper){

            etesito.claw.setPosition(1);

            clawTimer.reset();
            claw_close = true;

            specimen = false;

        }

        if(claw_close && clawTimer.seconds() >= 0.5) {

            claw_close = false;
            etesito.wrist.setPosition(0.5);

        }

        if(arm_Medium && specimen && specimenTimer.seconds() >= 0.5) {

            specimen = false;

            rodecontroller.targetPosition = -400;

        }

        beforeArmPos = etesito.armMotor.getCurrentPosition();

        telemetry.addData("ratio", ratio);

        telemetry.addLine("-----------");

        telemetry.addData("rodepower", etesito.rodeMotor.getPower());
        telemetry.addData("rodePos", etesito.rodeMotor.getCurrentPosition());
        telemetry.addData("rodeTargetPos", rodecontroller.targetPosition);

        telemetry.addLine("-----------");

        telemetry.addData("armpower", etesito.armMotor.getPower());
        telemetry.addData("armPos", etesito.armMotor.getCurrentPosition());
        telemetry.addData("armTargetPos", armcontroller.targetPosition);

        telemetry.addData("c1_pos", etesito.servoC1.getPosition());
        telemetry.addData("c2_pos", etesito.servoC2.getPosition());

        telemetry.addData("rdTarget", rdT);

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

        if (gamepad1.right_trigger > 0.5){
            etesito.FL.setPower(frontLeftPower * 0.4);
            etesito.BL.setPower(backLeftPower * 0.4);
            etesito.FR.setPower(frontRightPower * 0.4);
            etesito.BR.setPower(backRightPower * 0.4);

        }else {
            etesito.FL.setPower(frontLeftPower);
            etesito.BL.setPower(backLeftPower);
            etesito.FR.setPower(frontRightPower);
            etesito.BR.setPower(backRightPower);

        }

    }
}








