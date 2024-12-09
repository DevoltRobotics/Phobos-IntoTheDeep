package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

    double rodeMin = Double.MIN_VALUE;
    double rodeMax = Double.MAX_VALUE;

    double armMin = Double.MIN_VALUE;
    double armMax = Double.MAX_VALUE;

    public double red;
    public double green;
    public double blue;
    public double alpha;

    private double powerArm;

    private double rdT;

    private double armT;

    private boolean arm_UpBasket;

    private boolean arm_UpSpecimen;

    private boolean arm_Medium;

    private boolean arm_Dowm;

    private boolean colgando;

    private boolean escalando = false;

    private boolean alternar_Garra1;

    private ElapsedTime timerEstablecercolgar = new ElapsedTime();

    private ElapsedTime movimientoBrazoExtenderCanastaTimer = new ElapsedTime();
    private boolean movimientoBrazoExtenderCanasta = false;

    private ElapsedTime movimientoBrazoContraerDownTimer = new ElapsedTime();
    private boolean movimientoBrazoContraerDown = false;

    private ElapsedTime automatizadoMuñecaDownTimer = new ElapsedTime();
    private boolean automatizadoMuñecaDown = false;

    private boolean bajarMuñecaCanasta = false;

    private ElapsedTime bajarBrazoCanastaTimer = new ElapsedTime();
    private boolean bajarBrazoCanasta = false;

    private ElapsedTime lowBasketMuñecaTimer = new ElapsedTime();
    private boolean lowBasketMuñeca = false;

    private boolean lowBasketArm = false;

    private ElapsedTime escalandoAutomatizadoTimer = new ElapsedTime();
    private boolean escalandoAutomatizado = false;

    private ElapsedTime colgandoAutomatizadoTimer = new ElapsedTime();
    private boolean colgandoAutomatizado = false;

    private ElapsedTime bajarBrazoEscalandoTimer = new ElapsedTime();
    private boolean bajarBrazoEscalando = false;

    private ElapsedTime noForzarServosTimer = new ElapsedTime();
    private boolean noForzarServos = false;

    private ElapsedTime specimenArmDownTimer = new ElapsedTime();
    private boolean specimenArmDown = false;

    private ElapsedTime specimenDownTimer = new ElapsedTime();
    private boolean specimenWristDown = false;

    private boolean specimenRodeDown = false;

    private ElapsedTime timerColgar = new ElapsedTime();

    private ElapsedTime timerMuñeca1 = new ElapsedTime();

    private ElapsedTime timerColor = new ElapsedTime();

    boolean manualRight;
    boolean manualLeft;

    boolean prerestartVoltage;
    boolean restartVoltage;

    boolean voltageIndicatorBoolean;

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
        arm_UpBasket = false;
        arm_Medium = false;
        arm_UpSpecimen = false;
        arm_Dowm = true;
        alternar_Garra1 = true;

        voltageIndicatorBoolean = false;

        etesito.servoC1.setPosition(0.7);
        etesito.servoC2.setPosition(0.3);

        escalandoAutomatizado = false;
        colgandoAutomatizado = false;

        escalando = false;

        rdT = etesito.downRodePos;
        armT = 0;

        prerestartVoltage = false;
        restartVoltage = false;

        noForzarServos = false;


    }

    int beforeArmPos = 0;

    @Override
    public void loop() {


        colgando = gamepad1.left_trigger > 0.3;

        /*if (gamepad1.left_trigger > 0.5 && timerEstablecercolgar.seconds() > 0.2) {
            colgando = !colgando;
            timerEstablecercolgar.reset();

        }*/

        rodecontroller.targetPosition = Range.clip(rodecontroller.targetPosition, rodeMin, rodeMax);
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

            if (gamepad2.right_stick_button){
                climbControllerRight.reset();
                telemetry.addLine("Right_Reset");

            }else if (gamepad2.left_stick_button){
                climbControllerLeft.reset();
                telemetry.addLine("Left_Reset");

            }

            if (gamepad2.dpad_up) {

                    climbControllerRight.targetPosition = -3900;
                    climbControllerLeft.targetPosition = -3900;
                    colgandoAutomatizado = true;
                    colgandoAutomatizadoTimer.reset();


                voltageIndicatorBoolean = false;

                noForzarServos = false;


            } else if (gamepad2.dpad_down) {

                climbControllerRight.targetPosition = -200;
                climbControllerLeft.targetPosition = -200;

                etesito.servoC1.setPosition(0);
                etesito.servoC2.setPosition(1);

                noForzarServos = true;
                noForzarServosTimer.reset();

                colgandoAutomatizado = false;

            }

            /*if (bajarBrazoEscalando && bajarBrazoEscalandoTimer.seconds() > 1.5){
                powerArm = 0.2;
                armcontroller.targetPosition = 0;

            }*/

            if (escalandoAutomatizado && escalandoAutomatizadoTimer.seconds() >= 1.8){

                etesito.servoC1.setPosition(0.6);
                etesito.servoC2.setPosition(0.4);

                escalandoAutomatizado = false;
                escalandoAutomatizadoTimer.reset();

            }

            if (colgandoAutomatizado && colgandoAutomatizadoTimer.seconds() > 1.8) {
                etesito.servos_Uping();

                colgandoAutomatizado = false;
            }


            manualRight = Math.abs(gamepad2.right_stick_y) > 0.5;
            manualLeft = Math.abs(gamepad2.left_stick_y) > 0.5;


            if (manualRight) {
                etesito.cR.setPower(Range.clip(gamepad2.right_stick_y, -0.98, 0.98));
                climbControllerRight.targetPosition = etesito.cR.getCurrentPosition();
                telemetry.addLine("ManualRight");

            }else {
                etesito.cR.setPower(climbControllerRight.update(etesito.cR.getCurrentPosition()));

            }

            if (manualLeft) {
                etesito.cL.setPower(Range.clip(gamepad2.left_stick_y, -0.98, 0.98));
                climbControllerLeft.targetPosition = etesito.cL.getCurrentPosition();
                telemetry.addLine("ManualLeft");

            }else {
                etesito.cL.setPower(climbControllerLeft.update(etesito.cL.getCurrentPosition()));
            }

            if (manualRight && manualLeft){
                voltageIndicatorBoolean = true;
                timerColgar.reset();

                noForzarServos = false;

            } else {
                voltageIndicatorBoolean = false;

            }

            if ((Math.abs(etesito.cL.getCurrent(CurrentUnit.AMPS)) > 4 && Math.abs(etesito.cL.getCurrent(CurrentUnit.AMPS)) > 4  && timerColgar.seconds() > 0.2 && voltageIndicatorBoolean)){
                etesito.servos_down();

                noForzarServos = true;
                noForzarServosTimer.reset();

                telemetry.addLine("motor_forzado");
            }



            telemetry.addData("timerColgar", timerColgar);
            telemetry.addData("x", voltageIndicatorBoolean);

            if (gamepad2.right_stick_button){
                climbControllerRight.reset();
                climbControllerLeft.reset();
                telemetry.addLine("climPosRestarted");

            }

            if (gamepad2.y && !escalando) {
                etesito.servos_Uping();
                noForzarServos = false;

            } else if (gamepad2.b) {
                etesito.servos_Climbing();
                noForzarServos = false;

            } else if (gamepad2.a) {
                etesito.servos_down();

                noForzarServos = true;
                noForzarServosTimer.reset();

            }

            if (gamepad1.dpad_left || gamepad2.dpad_left) {
                rodecontroller.targetPosition = 0;

            } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
                rodecontroller.targetPosition = etesito.climbingRodePos;

            }


        } else {

            telemetry.addLine("colgando_desactivado");


            if(timerColor.seconds() > 0.2) {
                etesito.getColors();

                timerColor.reset();
            }

            etesito.colorTelemetry(telemetry);

            rodecontroller.targetPosition += (gamepad2.right_stick_y * 28);
            armcontroller.targetPosition += (gamepad2.left_stick_y * 30);

            boolean extended = !(rodecontroller.targetPosition >= -150);

            if (gamepad2.dpad_down) {
                rodecontroller.targetPosition = 0;

            } else if (gamepad2.dpad_up) {
                rodecontroller.targetPosition = rdT;

            }

            if (gamepad2.y && !extended) {
                powerArm = 0.4;
                armT = etesito.highArmpos;
                rodecontroller.reset();
                etesito.wrist_up();

                movimientoBrazoExtenderCanastaTimer.reset();
                movimientoBrazoExtenderCanasta = true;

                arm_UpBasket = true;
                arm_Medium = false;
                arm_UpSpecimen = false;
                arm_Dowm = false;

                telemetry.addLine("Arm_up");


            } else if (gamepad2.b && !extended) {
                powerArm = 0.4;
                armT = etesito.lowBasketArmpos;
                rodecontroller.reset();

                etesito.wrist_Medium();

                arm_UpBasket = false;
                arm_Medium = true;
                arm_UpSpecimen = false;
                arm_Dowm = false;

                bajarBrazoCanasta = true;
                bajarBrazoCanastaTimer.reset();

                telemetry.addLine("Arm_mediumLow");

            } else if (gamepad2.x && !extended) {
                powerArm = 0.4;
                armT = etesito.specimenArmPos;
                rodecontroller.reset();

                etesito.wrist_Specimen();

                arm_UpBasket = false;
                arm_Medium = false;
                arm_UpSpecimen = true;
                arm_Dowm = false;

                telemetry.addLine("Arm_mediumHigh");

            }
            else if (gamepad2.a && !extended) {
                powerArm = 0.1;
                armT = etesito.down_ArmPos;
                rodecontroller.reset();
                armcontroller.reset();
                etesito.wrist_Medium();

                arm_UpBasket = false;
                arm_Medium = false;
                arm_UpSpecimen = false;
                arm_Dowm = true;
                movimientoBrazoExtenderCanasta = false;

                telemetry.addLine("Arm_down");

            }

            if (arm_UpBasket) {
                rdT = etesito.highRodePos;
                rodeMax = 50;
                rodeMin = etesito.highRodePos - 150;

            } else if (arm_Medium) {
                rdT = etesito.lowBasketRodePos;
                rodeMax = 50;
                rodeMin = etesito.lowBasketRodePos - 200;

            }
            else if (arm_Dowm){
                rdT = etesito.downRodePos;
                rodeMax = 50;
                rodeMin = etesito.downRodePos - 500;

            } else if (arm_UpSpecimen) {
                rdT = etesito.specimenRodePos;
                rodeMax = 50;
                rodeMin = etesito.downRodePos - 100;

            }

            if (movimientoBrazoExtenderCanasta && movimientoBrazoExtenderCanastaTimer.seconds() >= 0.6) {
                rodecontroller.targetPosition = rdT;
                movimientoBrazoExtenderCanasta = false;
            }


            ///////////////////////////////////////////////////////////////////////////

            if (gamepad2.dpad_left && arm_UpBasket) {
                etesito.wrist_up();
                bajarBrazoCanasta = false;

            } else if (gamepad2.dpad_right && arm_UpBasket){
                etesito.wrist_Down_M();

                bajarBrazoCanasta = false;
                bajarBrazoCanastaTimer.reset();

            } else if (gamepad2.dpad_left && arm_Medium) {
                etesito.wrist_Medium();
                bajarBrazoCanasta = false;

            } else if (gamepad2.dpad_right && arm_Medium) {
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

                lowBasketMuñeca = false;
                bajarBrazoCanasta = false;

                automatizadoMuñecaDown = false;

            }  else if (arm_Medium && (gamepad1.left_bumper || gamepad1.b)) {
                etesito.open();

                automatizadoMuñecaDown = false;
                bajarBrazoCanasta = false;

                lowBasketMuñeca = true;
                lowBasketMuñecaTimer.reset();

            } else if (arm_UpSpecimen && ((gamepad1.left_bumper || gamepad1.b))) {
                etesito.open();

                specimenWristDown = true;
                specimenRodeDown = true;
                specimenArmDown = true;
                specimenDownTimer.reset();

                automatizadoMuñecaDown = false;
                bajarBrazoCanasta = false;
                lowBasketMuñeca = false;

            } else if (arm_UpBasket && (gamepad1.left_bumper || gamepad1.b)) {
                etesito.open();

                automatizadoMuñecaDown = false;

                lowBasketMuñeca = false;

                bajarMuñecaCanasta = true;
                bajarBrazoCanasta = true;
                bajarBrazoCanastaTimer.reset();

            } else if (arm_Dowm && (gamepad1.right_bumper || gamepad1.a)){
                etesito.pick();

                automatizadoMuñecaDown = true;
                automatizadoMuñecaDownTimer.reset();

            } else if (gamepad1.right_bumper || gamepad1.a){
                etesito.pick();
                specimenWristDown = true;
                specimenRodeDown = true;
            }

            if (specimenWristDown && specimenDownTimer.seconds() > 0.3) {
                etesito.wrist_Medium();
                rodecontroller.targetPosition = 0;
                specimenWristDown = false;

            }

            if (specimenArmDown && specimenDownTimer.seconds() > 0.8){


                specimenArmDown = false;

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

            if (lowBasketMuñeca && lowBasketMuñecaTimer.seconds() >= 0.4) {

                etesito.wrist_up();
                lowBasketMuñeca = false;

            }

            if (lowBasketArm && lowBasketMuñecaTimer.seconds() >= 1) {
                rodecontroller.targetPosition = 0;
                lowBasketArm = true;
            }

        }

        armMax = 50;
        armMin = etesito.highArmpos - 50;

        armcontroller.targetPosition = Range.clip(armT, armMin, armMax);

        double deltaArmPos = etesito.armMotor.getCurrentPosition() - beforeArmPos;

        rodecontroller.targetPosition += (deltaArmPos / etesito.ratio);

        beforeArmPos = etesito.armMotor.getCurrentPosition();

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

        telemetry.addData("angulo", Math.toDegrees( botHeading));
    }

}

