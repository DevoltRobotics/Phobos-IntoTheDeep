package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Comands.Etesito;
import org.firstinspires.ftc.teamcode.Comands.PIDFController;

@Config
@TeleOp(name = "ETESITO")

public class ET extends OpMode {

    private final Etesito etesito = new Etesito();

    public static PIDFController.PIDCoefficients armCoefficients = new PIDFController.PIDCoefficients(0.0015, 0, 0.0017);
    public static PIDFController.PIDCoefficients rodeCoefficients = new PIDFController.PIDCoefficients(0.015, 0, 0.007);
    public static PIDFController.PIDCoefficients climbCoefficients = new PIDFController.PIDCoefficients(0.015, 0, 0.0017);

    PIDFController armcontroller = new PIDFController(armCoefficients);
    PIDFController rodecontroller = new PIDFController(rodeCoefficients);
    PIDFController climbControllerRight = new PIDFController(climbCoefficients);
    PIDFController climbControllerLeft = new PIDFController(climbCoefficients);

    private double powerArm;

    private double rdT;

    private boolean arm_Up;

    private boolean arm_Medium_High;

    private boolean arm_Medium_Low;

    private boolean arm_Dowm;

    private boolean colgando;

    private boolean escalando;

    private boolean alternar_Garra1;

    private boolean alternarArmMedio;
    private ElapsedTime alternarArmMedioTimer = new ElapsedTime();

    private ElapsedTime movimientoBrazoExtenderTimer = new ElapsedTime();
    private boolean movimientoBrazoExtender = false;

    private ElapsedTime movimientoBrazoContraerDownTimer = new ElapsedTime();
    private boolean movimientoBrazoContraerDown = false;

    private ElapsedTime automatizadoMuñecaDownTimer = new ElapsedTime();
    private boolean automatizadoMuñecaDown = false;

    private boolean bajarMuñecaCanasta = false;

    private ElapsedTime bajarBrazoCanastaTimer = new ElapsedTime();
    private boolean bajarBrazoCanasta = false;

    private ElapsedTime specimenTimer = new ElapsedTime();
    private boolean specimen = false;

    private ElapsedTime escalandoAutomatizadoTimer = new ElapsedTime();
    private boolean escalandoAutomatizado = false;

    private ElapsedTime colgandoAutomatizadoTimer = new ElapsedTime();
    private boolean colgandoAutomatizado = false;

    private ElapsedTime bajarBrazoEscalandoTimer = new ElapsedTime();
    private boolean bajarBrazoEscalando = false;

    private ElapsedTime noForzarServosTimer = new ElapsedTime();
    private boolean noForzarServos = false;

    private ElapsedTime timerEstablecercolgar = new ElapsedTime();

    private ElapsedTime timerColgar = new ElapsedTime();

    private ElapsedTime timerMuñeca1 = new ElapsedTime();

    private ElapsedTime timerColor = new ElapsedTime();

    boolean manualRight;
    boolean manualLeft;

    boolean prerestartVoltage;
    boolean restartVoltage;

    boolean x;

    @Override
    public void init() {


        etesito.init(hardwareMap);

        etesito.imu.resetYaw();

        armcontroller.reset();
        rodecontroller.reset();
        climbControllerRight.reset();
        climbControllerLeft.reset();

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );




        telemetry.addLine("Robot Initialized.");
        telemetry.update();

        colgando = false;
        arm_Up = false;
        arm_Medium_Low = false;
        arm_Medium_High = false;
        arm_Dowm = true;
        alternar_Garra1 = true;

        x = false;


        etesito.servoC1.setPosition(0.5);
        etesito.servoC2.setPosition(0.5);

        escalandoAutomatizado = false;
        colgandoAutomatizado = false;

        escalando = false;

        rdT = etesito.rode_down;

        prerestartVoltage = false;
        restartVoltage = false;

        noForzarServos = false;

        alternarArmMedio = true;

    }

    int beforeArmPos = 0;

    @Override
    public void loop() {


        //colgando = gamepad1.left_trigger > 0.5;

        if (gamepad1.left_trigger > 0.5) {
            colgando = true;

        }/*else if (gamepad2.x && timerEstablecercolgar.seconds() > 0.2) {
            colgando = !colgando;
            timerEstablecercolgar.reset();

        }*/

        etesito.rodeMotor.setPower(rodecontroller.update(etesito.rodeMotor.getCurrentPosition()) * 0.09);

        etesito.armMotor.setPower(-armcontroller.update(etesito.armMotor.getCurrentPosition()) * powerArm);

        if (gamepad2.right_stick_button){
            rodecontroller.reset();

        } else if (gamepad2.left_stick_button) {
            armcontroller.reset();

        }
        //etesito.getColor();

        if (colgando) {

            etesito.wrist_Climb();

            telemetry.addLine("colgando_activado");

            escalando = gamepad1.y;

            if (gamepad2.right_stick_button){
                climbControllerRight.reset();
                telemetry.addLine("Right_Reset");

            }else if (gamepad2.left_stick_button){
                climbControllerLeft.reset();
                telemetry.addLine("Left_Reset");

            }

            if (escalando){
                telemetry.addLine("escalando");

            }else{
                telemetry.addLine("subiendo");

            }

            if (gamepad1.dpad_up || gamepad2.dpad_up) {

                if (escalando) {

                    climbControllerRight.targetPosition = -5300;
                    climbControllerLeft.targetPosition = -5300;
                    escalandoAutomatizadoTimer.reset();
                    escalandoAutomatizado = true;

                } else {
                    climbControllerRight.targetPosition = -4300;
                    climbControllerLeft.targetPosition = -4300;
                    colgandoAutomatizado = true;
                    colgandoAutomatizadoTimer.reset();
                }

                x = false;

                noForzarServos = false;


            } else if (gamepad1.dpad_down || gamepad2.dpad_down) {

                climbControllerRight.targetPosition = 0;
                climbControllerLeft.targetPosition = 0;

                etesito.servoC1.setPosition(0);
                etesito.servoC2.setPosition(1);

                noForzarServos = true;
                noForzarServosTimer.reset();

                if (escalando){
                    powerArm = 0.2;
                    armcontroller.targetPosition = -1200;
                    bajarBrazoEscalando = true;
                    bajarBrazoEscalandoTimer.reset();

                }

                colgandoAutomatizado = false;

            }

            if (bajarBrazoEscalando && bajarBrazoEscalandoTimer.seconds() > 1.5){
                powerArm = 0.2;
                armcontroller.targetPosition = 0;

            }

            if (escalandoAutomatizado && escalandoAutomatizadoTimer.seconds() >= 1.8){

                etesito.servoC1.setPosition(0.6);
                etesito.servoC2.setPosition(0.4);

                escalandoAutomatizado = false;
                escalandoAutomatizadoTimer.reset();

            }

            if (colgandoAutomatizado && colgandoAutomatizadoTimer.seconds() > 1.8) {

                etesito.servoC1.setPosition(0.7);
                etesito.servoC2.setPosition(0.3);

                colgandoAutomatizado = false;
            }


            manualRight = Math.abs(gamepad2.right_stick_y) > 0.5;
            manualLeft = Math.abs(gamepad2.left_stick_y) > 0.5;


            if (manualRight) {
                etesito.cR.setPower(Range.clip(gamepad2.right_stick_y, -0.95, 0.95));
                climbControllerRight.targetPosition = etesito.cR.getCurrentPosition();
                telemetry.addLine("ManualRight");

            }else {
                etesito.cR.setPower(climbControllerRight.update(etesito.cR.getCurrentPosition()));

            }

            if (manualLeft) {
                etesito.cL.setPower(Range.clip(gamepad2.left_stick_y, -0.95, 0.95));
                climbControllerLeft.targetPosition = etesito.cL.getCurrentPosition();
                telemetry.addLine("ManualLeft");

            }else {
                etesito.cL.setPower(climbControllerLeft.update(etesito.cL.getCurrentPosition()));
            }



            if ((Math.abs(etesito.cL.getCurrent(CurrentUnit.AMPS)) > 4 && Math.abs(etesito.cL.getCurrent(CurrentUnit.AMPS)) > 4  && timerColgar.seconds() > 0.2 && x)){
                etesito.servoC1.setPosition(0);
                etesito.servoC2.setPosition(1);

                noForzarServos = true;
                noForzarServosTimer.reset();

                telemetry.addLine("motor_forzado");
            }

            if (manualRight && manualLeft){
                x = true;
                timerColgar.reset();

                noForzarServos = false;

            }
            telemetry.addData("timerColgar", timerColgar);
            telemetry.addData("x", x);

            if (gamepad2.right_stick_button){
                etesito.cL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                etesito.cR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                etesito.cL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                etesito.cR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }

            if (gamepad2.y && !escalando) {
                etesito.servoC1.setPosition(0.7);
                etesito.servoC2.setPosition(0.3);
                noForzarServos = false;

            } else if (gamepad2.b) {
                etesito.servoC1.setPosition(0.6);
                etesito.servoC2.setPosition(0.4);
                noForzarServos = false;

            } else if (gamepad2.a) {
                etesito.servoC1.setPosition(0);
                etesito.servoC2.setPosition(1);

                noForzarServos = true;
                noForzarServosTimer.reset();

            }

            if (noForzarServos && noForzarServosTimer.seconds() > 0.2){
                etesito.servoC1.getController().pwmDisable();
                etesito.servoC2.getController().pwmDisable();

            }

            if (gamepad2.dpad_left) {
                rodecontroller.targetPosition = 0;

            } else if (gamepad2.dpad_right) {
                rodecontroller.targetPosition = -600;

            }


        } else {

            telemetry.addLine("colgando_desactivado");

            if(timerColor.seconds() > 0.2) {
                etesito.getColors();
                timerColor.reset();
            }

            rodecontroller.targetPosition += (gamepad2.right_stick_y * 28);
            armcontroller.targetPosition += (gamepad2.left_stick_y * 30);

            boolean extended = !(rodecontroller.targetPosition >= -150);

            if (gamepad2.y && !extended) {
                powerArm = 0.4;
                armcontroller.targetPosition = etesito.high_Armpos;
                rodecontroller.reset();
                etesito.wrist_up();

                movimientoBrazoExtenderTimer.reset();
                movimientoBrazoExtender = true;

                arm_Up = true;
                arm_Medium_Low = false;
                arm_Medium_High = false;
                arm_Dowm = false;

                telemetry.addLine("Arm_up");


            } else if (gamepad2.x && !extended && alternarArmMedioTimer.seconds() > 0.2 && alternarArmMedio) {
                powerArm = 0.4;
                armcontroller.targetPosition = etesito.mediumHigh_Armpos;
                rodecontroller.reset();

                etesito.wrist_up();

                arm_Up = false;
                arm_Medium_Low = true;
                arm_Medium_High = false;
                arm_Dowm = false;

                alternarArmMedio = false;

                telemetry.addLine("Arm_mediumLow");

            } else if (gamepad2.x && !extended && alternarArmMedioTimer.seconds() > 0.2 && !alternarArmMedio) {
                powerArm = 0.4;
                armcontroller.targetPosition = etesito.mediumLow_Armpos;
                rodecontroller.reset();

                etesito.wrist_Medium();

                arm_Up = false;
                arm_Medium_Low = false;
                arm_Medium_High = true;
                arm_Dowm = false;

                alternarArmMedio = true;

                telemetry.addLine("Arm_mediumHigh");

            }
            else if (gamepad2.a && !extended) {
                powerArm = 0.1;
                armcontroller.targetPosition = etesito.down_ArmPos;
                rodecontroller.reset();
                armcontroller.reset();
                etesito.wrist_Medium();
                etesito.open();

                arm_Up = false;
                arm_Medium_Low = false;
                arm_Medium_High = false;
                arm_Dowm = true;
                movimientoBrazoExtender = false;

                telemetry.addLine("Arm_down");

            }

            if (arm_Up) {
                rdT = etesito.rode_High;
                rodecontroller.setOutputBounds((etesito.rode_High - 50), 50);

            } else if (arm_Medium_Low) {
                rdT = etesito.rode_medium;
                rodecontroller.setOutputBounds((etesito.rode_medium - 50), 50);

            } else {
                rdT = etesito.rode_down;
                rodecontroller.setOutputBounds((etesito.rode_down - 50), 50);

            }

            if (movimientoBrazoExtender && movimientoBrazoExtenderTimer.seconds() >= 0.6) {
                rodecontroller.targetPosition = rdT;
                movimientoBrazoExtender = false;
            }


            if (gamepad2.dpad_down) {
                rodecontroller.targetPosition = 0;

            } else if (gamepad2.dpad_up) {
                rodecontroller.targetPosition = rdT;

            }

            ///////////////////////////////////////////////////////////////////////////

            if (gamepad2.dpad_left && arm_Up) {
                etesito.wrist_up();
                bajarBrazoCanasta = false;

            } else if (gamepad2.dpad_right && arm_Up){
                etesito.wrist_Down_M();

                bajarBrazoCanasta = false;
                bajarBrazoCanastaTimer.reset();

            } else if (gamepad2.dpad_left && arm_Medium_Low) {
                etesito.wrist_Medium();
                bajarBrazoCanasta = false;

            } else if (gamepad2.dpad_right && arm_Medium_Low) {
                etesito.wrist_Down_M();
                bajarBrazoCanasta = false;

            } else if (gamepad2.dpad_left && arm_Dowm) {
                etesito.wrist_Medium();
                bajarBrazoCanasta = false;

                alternar_Garra1 = true;

            } else if (gamepad2.dpad_right && arm_Dowm && alternar_Garra1 && timerMuñeca1.seconds() > 0.2) {
                etesito.wrist_Down_M();
                bajarBrazoCanasta = false;

                alternar_Garra1 = false;

                timerMuñeca1.reset();

            } else if (gamepad2.dpad_right && arm_Dowm && !alternar_Garra1 && timerMuñeca1.seconds() > 0.2){
                etesito.wrist_down();
                bajarBrazoCanasta = false;

                alternar_Garra1 = true;

                timerMuñeca1.reset();
            }


            if (arm_Dowm &&( gamepad1.left_bumper || gamepad1.b)) {
                etesito.open();

                specimen = false;
                bajarBrazoCanasta = false;

                automatizadoMuñecaDown = false;

            }  else if (arm_Medium_Low && (gamepad1.left_bumper || gamepad1.b)) {
                etesito.open();

                automatizadoMuñecaDown = false;
                bajarBrazoCanasta = false;

                specimen = true;
                specimenTimer.reset();

            } else if (arm_Up && (gamepad1.left_bumper || gamepad1.b)) {
                etesito.open();

                automatizadoMuñecaDown = false;

                specimen = false;

                bajarMuñecaCanasta = true;
                bajarBrazoCanasta = true;
                bajarBrazoCanastaTimer.reset();

            } else if (arm_Dowm && (gamepad1.right_bumper || gamepad1.a)){
                etesito.pick();

                automatizadoMuñecaDown = true;
                automatizadoMuñecaDownTimer.reset();

            } else if (gamepad1.right_bumper || gamepad1.a){
                etesito.pick();

            }

            if (bajarMuñecaCanasta && bajarBrazoCanastaTimer.seconds() > 0.4){
                etesito.wrist_Down_M();
                bajarMuñecaCanasta = false;

            }

            if (bajarBrazoCanasta && bajarBrazoCanastaTimer.seconds() > 0.8){
                rodecontroller.targetPosition = 0;
                bajarBrazoCanasta = false;

            }

            if (arm_Dowm && automatizadoMuñecaDown && automatizadoMuñecaDownTimer.seconds() >= 0.5) {
                etesito.wrist_Medium();
                automatizadoMuñecaDown = false;

                alternar_Garra1 = true;

                movimientoBrazoContraerDown = true;
                movimientoBrazoContraerDownTimer.reset();

            }

            if (movimientoBrazoContraerDown && movimientoBrazoContraerDownTimer.seconds() > 0.2  /*&& (etesito.color.green() > etesito.greenTarget || etesito.color.blue() > etesito.blueTarget)*/){
                rodecontroller.targetPosition = 0;
                movimientoBrazoContraerDown = false;

            }

            if (specimen && specimenTimer.seconds() >= 0.5) {

                rodecontroller.targetPosition = 0;
                specimen = false;

            }

        }

        double deltaArmPos = etesito.armMotor.getCurrentPosition() - beforeArmPos;

        rodecontroller.targetPosition += (deltaArmPos / etesito.ratio);

        beforeArmPos = etesito.armMotor.getCurrentPosition();

        etesito.colorTelemetry(telemetry);

        /*telemetry.addData("climbPower1", etesito.cR.getPower());
        telemetry.addData("climbPos1", etesito.cR.getCurrentPosition());
        telemetry.addData("climbTargetPos1", climbControllerRight.targetPosition);

        telemetry.addData("climbPower2", etesito.cL.getPower());
        telemetry.addData("climbPos2", etesito.cL.getCurrentPosition());
        telemetry.addData("climbTargetPos2", climbControllerRight.targetPosition);

        telemetry.addData("rodepower", etesito.rodeMotor.getPower());
        telemetry.addData("rodePos", etesito.rodeMotor.getCurrentPosition());
        telemetry.addData("rodeTargetPos", rodecontroller.targetPosition);

        telemetry.addData("climbPower1", etesito.cR.getPower());
        telemetry.addData("climbPos1", etesito.cR.getCurrentPosition());
        telemetry.addData("climbTargetPos1", climbControllerRight.targetPosition);

        telemetry.addData("climbPower2", etesito.cL.getPower());
        telemetry.addData("climbPos2", etesito.cL.getCurrentPosition());
        telemetry.addData("climbTargetPos2", climbControllerRight.targetPosition);


        telemetry.addLine("-----------");

        telemetry.addData("voltage", etesito.cL.getCurrent(CurrentUnit.AMPS));

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

        telemetry.addData("rdTarget", rdT); */

        /////////////////////////////////////////////////////////////////7//////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////7
        /////////////////////////////////////////////////////////////////////////////////////////////////////////777

        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        double botHeading = etesito.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (gamepad1.dpad_up) {
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

        telemetry.addData("angulo", botHeading);
    }

}

