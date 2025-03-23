package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Comands.Constants.climbCoefficients;
import static org.firstinspires.ftc.teamcode.Comands.Constants.climbingRodePos1;
import static org.firstinspires.ftc.teamcode.Comands.Constants.climbingRodePos2;
import static org.firstinspires.ftc.teamcode.Comands.Constants.down2RodePos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.downRodePos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.BasketArmpos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.highRodePos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.frontArmPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.lowBasketRodePos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.postSpecimenArmPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.ratioArm;
import static org.firstinspires.ftc.teamcode.Comands.Constants.specimenArmPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.specimenRodePos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Comands.Etesito;
import org.firstinspires.ftc.teamcode.Comands.PIDFController;

@TeleOp(name = "ETESITO")

public class ET extends OpMode {

    private final Etesito etesito = new Etesito();

    PIDFController climbControllerRight = new PIDFController(climbCoefficients);
    PIDFController climbControllerLeft = new PIDFController(climbCoefficients);

    double currentRodeTicks = 0;

    double rodeMin = Double.MIN_VALUE;
    double rodeMax = Double.MAX_VALUE;

    double armMin = Double.MIN_VALUE;
    double armMax = Double.MAX_VALUE;

    private int armPosition = 0;

    private double previousArmPosition;

    private double powerArm;

    private double rdTarget;

    private double armTarget;

    boolean escalando = false;

    private final ElapsedTime toggleWristTimer = new ElapsedTime();

    private final ElapsedTime timerRode = new ElapsedTime();

    private boolean toggleRodeDown = true;
    private final ElapsedTime toggleRodeDownTimer = new ElapsedTime();

    private final ElapsedTime manualWristTimer = new ElapsedTime();

    private final ElapsedTime contractArmDownTimer = new ElapsedTime();
    private boolean contractArmDown = false;

    private final ElapsedTime extendArmHighBasketTimer = new ElapsedTime();
    private boolean extendArmHighBasket = false;

    private final ElapsedTime lowerArmBasketTimer = new ElapsedTime();
    private boolean contractArmBasket = false;
    private boolean lowerWristBasket = false;

    private final ElapsedTime specimenUpArmTimer = new ElapsedTime();
    private boolean specimenUpArm = false;
    private boolean specimenUpWrist = false;

    private final ElapsedTime specimenDownTimer = new ElapsedTime();
    private boolean specimenOpenClaw = false;
    private boolean specimenArmSemiDown = false;
    private boolean specimenArmDown = false;
    private boolean specimenWristDown = false;
    private boolean specimenRodeDown = false;

    private final ElapsedTime colgandoAutomatizadoTimer = new ElapsedTime();
    private boolean colgandoAutomatizado = false;

    private final ElapsedTime timerColgar = new ElapsedTime();

    private final ElapsedTime togglePalitoTimer = new ElapsedTime();

    private final ElapsedTime waitServosTimer = new ElapsedTime();
    private boolean waitServos = false;

    boolean manualRight;
    boolean manualLeft;

    boolean voltageIndicatorBoolean;

    boolean lanzarBrazitos = false;

    @Override
    public void init() {
        etesito.init(hardwareMap, false, true);

        toggleRodeDown = true;

        lanzarBrazitos = false;

        rodeMax = 250;

        climbControllerRight.reset();
        climbControllerLeft.reset();

        /*telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );*/

        etesito.guardarBrazitosColgada();

        telemetry.addLine("Robot Initialized.");
        telemetry.update();

        rdTarget = downRodePos;
        armTarget = 0;

        etesito.rodeController.targetPosition = etesito.rodeMotor.getCurrentPosition();

    }

    int beforeArmPos = 0;

    int beforeRodePos = 0;

    @Override
    public void loop() {

        boolean colgando = gamepad1.left_trigger > 0.3;

        etesito.rodeController.targetPosition = Range.clip(etesito.rodeController.targetPosition, rodeMin, rodeMax);
        etesito.rodeMotor.setPower(etesito.rodeController.update(etesito.rodeMotor.getCurrentPosition()) * 0.09);

        if (colgando) {

            if (!lanzarBrazitos) {
                etesito.lanzarBrazitosColgada();
                etesito.servosUping();

            }

            lanzarBrazitos = true;

            etesito.dropSpecimen();

            escalando = gamepad1.right_trigger > 0.5;

            if (escalando) {
                etesito.setLight("blue");
                etesito.rodeController.targetPosition = climbingRodePos2;

            } else {
                etesito.setLight("green");
                etesito.rodeController.targetPosition = climbingRodePos1;

            }

            telemetry.addLine("colgando_activado");

            if (gamepad2.dpad_up) {
                colgandoAutomatizado = true;
                colgandoAutomatizadoTimer.reset();

                    climbControllerRight.targetPosition = 0;
                    climbControllerLeft.targetPosition = 0;

                voltageIndicatorBoolean = false;

            } else if (gamepad2.dpad_down) {

                etesito.servosDown();

                colgandoAutomatizado = false;

            }

            if (colgandoAutomatizado && colgandoAutomatizadoTimer.seconds() > 1.9) {

                if (escalando) {
                    etesito.servosClimbing();

                } else {
                    etesito.servosUping();

                }
                colgandoAutomatizado = false;
            }


            manualRight = Math.abs(gamepad2.right_stick_y) > 0.5;
            manualLeft = Math.abs(gamepad2.left_stick_y) > 0.5;

            if (manualRight) {
                etesito.mCR.setPower(Range.clip(gamepad2.right_stick_y, -0.98, 0.98));
                climbControllerRight.targetPosition = etesito.mCR.getCurrentPosition();
                telemetry.addLine("ManualRight");

            } else {
                etesito.mCR.setPower(climbControllerRight.update(etesito.mCR.getCurrentPosition()));

            }

            if (manualLeft) {
                etesito.mCL.setPower(Range.clip(gamepad2.left_stick_y, -0.98, 0.98));
                climbControllerLeft.targetPosition = etesito.mCL.getCurrentPosition();
                telemetry.addLine("ManualLeft");

            } else {
                etesito.mCL.setPower(climbControllerLeft.update(etesito.mCL.getCurrentPosition()));
            }

            if (manualRight && manualLeft) {
                voltageIndicatorBoolean = true;
                timerColgar.reset();

            } else {
                voltageIndicatorBoolean = false;

            }

            if ((Math.abs(etesito.mCL.getCurrent(CurrentUnit.AMPS)) > 4 && Math.abs(etesito.mCL.getCurrent(CurrentUnit.AMPS)) > 4 && timerColgar.seconds() > 0.2 && voltageIndicatorBoolean)) {
                etesito.servosDown();

                telemetry.addLine("motor_forzado");
            }

            if (gamepad2.y) {
                etesito.servosUping();

            } else if (gamepad2.b) {
                etesito.servosClimbing();

            } else if (gamepad2.a) {
                etesito.servosDown();

            }

            if (gamepad2.right_bumper){
                etesito.lanzarBrazitosColgada();

                waitServos = true;
                waitServosTimer.reset();


            }else if (gamepad2.left_bumper){
                etesito.guardarBrazitosColgada();

            }

            if (waitServos && waitServosTimer.seconds() > 0.2){
                etesito.servosUping();
                waitServos = false;

            }

            if (gamepad2.dpad_left) {
                etesito.rodeController.targetPosition = 0;

            } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
                if (escalando) etesito.rodeController.targetPosition = climbingRodePos2;
                else etesito.rodeController.targetPosition = climbingRodePos1;

            }

        }
        else {
            if (gamepad2.start){
                etesito.resetArmEncoder();
            }

            if (gamepad2.options){
                etesito.resetRodeEncoder();

            }

            boolean manualArm = Math.abs(gamepad2.left_stick_y) > 0.5;

            if (manualArm) {
                etesito.armMotor.setPower(-gamepad2.left_stick_y * 0.35);
                armTarget = (etesito.armMotor.getCurrentPosition());

            } else {
                etesito.armMotor.setPower(-etesito.armController.update(etesito.armMotor.getCurrentPosition()) * powerArm);

            }

            boolean manualRode = Math.abs(gamepad2.right_stick_y) > 0.3;

            if (manualRode) {
                etesito.rodeMotor.setPower(gamepad2.right_stick_y * 0.5);
                etesito.rodeController.targetPosition = (etesito.rodeMotor.getCurrentPosition());

            } else {
                etesito.rodeMotor.setPower(etesito.rodeController.update(etesito.rodeMotor.getCurrentPosition()) * 0.09);

            }
            telemetry.addLine("colgando_desactivado");

            escalando = false;

            etesito.setLight("orange");

            if (gamepad2.left_bumper || gamepad2.right_bumper || gamepad1.left_bumper || gamepad1.right_bumper) {
                etesito.servos_test();
                etesito.guardarBrazitosColgada();
            }

            boolean extended = !(etesito.rodeMotor.getCurrentPosition() >= -400);

            if (gamepad2.dpad_down) {
                extendArmHighBasket = false;

                toggleRodeDown = true;

                if (armPosition == 3){
                    etesito.rodeController.targetPosition = 0;

                }else {
                    etesito.rodeController.targetPosition = 0;

                }

            } else if (gamepad2.dpad_up) {

                if (armPosition == 2) {
                    specimenArmSemiDown = true;
                    specimenOpenClaw = true;
                    specimenWristDown = true;
                    specimenRodeDown = true;
                    specimenArmDown = true;

                    specimenDownTimer.reset();
                } else if (armPosition == 0) {
                    if (toggleRodeDownTimer.seconds() > 0.2){
                        if (toggleRodeDown){
                            toggleRodeDown = false;

                        } else {
                            toggleRodeDown = true;

                        }
                        toggleRodeDownTimer.reset();

                    }

                }
                etesito.rodeController.targetPosition = rdTarget;
            }

            if (gamepad2.y && !extended) {
                powerArm = 0.4;
                armTarget = BasketArmpos;
                etesito.wristUp();

                armPosition = 3;

                extendArmHighBasket = true;
                extendArmHighBasketTimer.reset();

                toggleRodeDown = true;

                telemetry.addLine("Arm_up");

            } else if (gamepad2.x && !extended) {
                powerArm = 0.4;
                armTarget = BasketArmpos;
                etesito.wristUp();

                armPosition = 2;

                extendArmHighBasket = false;

                toggleRodeDown = true;

                telemetry.addLine("ArmSemiDown");

            } else if (gamepad2.b && !extended) {
                powerArm = 0.4;
                armTarget = frontArmPos;
                etesito.wristMedium();

                armPosition = 1;

                extendArmHighBasket = false;

                toggleRodeDown = true;

                telemetry.addLine("Arm_mediumLow");

            } else if (gamepad2.a && !extended) {

                if (previousArmPosition == 2) {
                    etesito.wristDown();

                } else {
                    etesito.wristContract();

                }
                powerArm = 0.1;
                armTarget = 0;

                armPosition = 0;

                extendArmHighBasket = false;

                toggleRodeDown = true;

                telemetry.addLine("Arm_down");

            }

            switch (armPosition) {
                case 3:
                    rdTarget = highRodePos;
                    rodeMin = highRodePos - 800;
                    break;

                case 2:
                    rdTarget = specimenRodePos - 100;
                    rodeMin = specimenRodePos - 900;
                    break;

                case 1:
                    rdTarget = lowBasketRodePos;
                    rodeMin = lowBasketRodePos - 200;
                    break;

                case 0:

                        if (toggleRodeDown) {
                            rdTarget = down2RodePos;

                        } else {
                            rdTarget = downRodePos;
                        }

                    if (etesito.wristIsMedium) {
                        rodeMin = downRodePos - 150;

                    } else {
                        rodeMin = downRodePos - 350;

                    }
                    break;
            }

            if (extendArmHighBasket && extendArmHighBasketTimer.seconds() >= 0.4) {
                etesito.rodeController.targetPosition = rdTarget;

                extendArmHighBasket = false;
            }

            if (gamepad2.right_trigger > 0.5 && manualWristTimer.seconds() > 0.2) {
                etesito.wristManualUp();
                manualWristTimer.reset();

            } else if (gamepad2.left_trigger > 0.5 && manualWristTimer.seconds() > 0.2) {
                etesito.wristManualDown();
                manualWristTimer.reset();

            } else {
                etesito.wristManualMantener();

            }

            if (gamepad2.dpad_left) {
                contractArmBasket = false;

                switch (armPosition) {
                    case 3:
                        etesito.wristUp();
                        break;
                    case 2:
                        etesito.wristSpecimen();
                        specimenWristDown = false;
                        specimenArmSemiDown = false;
                        specimenOpenClaw = false;
                        specimenRodeDown = false;
                        specimenArmDown = false;
                        break;
                    case 1:
                        etesito.wristMedium();
                        break;
                    case 0:
                        etesito.wristContract();

                        contractArmDown = true;
                        contractArmDownTimer.reset();

                        break;

                }

            } else if (gamepad2.dpad_right) {
                contractArmDown = false;

                switch (armPosition) {
                    case 3:
                        etesito.wristDown();

                        contractArmBasket = true;
                        lowerArmBasketTimer.reset();
                        break;

                    case 2:
                        etesito.wristSpecimen();
                        break;

                    case 1:
                        etesito.wristDown();
                        break;

                    case 0:

                        if (etesito.toggleWrist && toggleWristTimer.seconds() > 0.2) {
                            etesito.wristDown();
                            etesito.toggleWrist = false;
                            toggleWristTimer.reset();
                        }else if (!etesito.toggleWrist && toggleWristTimer.seconds() > 0.2) {
                            etesito.wristContract();
                            etesito.toggleWrist = true;
                            toggleWristTimer.reset();
                        }

                        break;

                }

            }

            if (gamepad1.dpad_left && togglePalitoTimer.seconds() > 0.2){
                etesito.esconderPalito();
                togglePalitoTimer.reset();
            }else if (gamepad1.dpad_right && togglePalitoTimer.seconds() > 0.2){
                etesito.lanzarSample();
                togglePalitoTimer.reset();
            }

            if (gamepad1.left_bumper || gamepad1.b) {
                etesito.dropSample();

                contractArmDown = false;

                if (armPosition == 1) {
                    lowerWristBasket = true;
                    contractArmBasket = true;
                    lowerArmBasketTimer.reset();

                } else if (armPosition == 3) {
                    lowerWristBasket = true;
                    contractArmBasket = true;
                    lowerArmBasketTimer.reset();
                }

            } else if (gamepad1.right_bumper || gamepad1.a) {
                etesito.pickSample();

                lowerWristBasket = false;
                contractArmBasket = false;

            } else {
                etesito.pickSampleSlow();

            }


            if (gamepad2.left_bumper) {
                etesito.dropSpecimen();

                specimenUpArm = false;
                specimenUpWrist = false;

            } else if (gamepad2.right_bumper && !extended) {
                etesito.pickSpecimen();

                specimenArmSemiDown = false;
                specimenOpenClaw = false;
                specimenWristDown = false;
                specimenRodeDown = false;
                specimenArmDown = false;

                if (armPosition == 0) {
                    specimenUpArm = true;
                    specimenUpWrist = true;
                    specimenUpArmTimer.reset();
                }
            }

            if (specimenUpArm && specimenUpArmTimer.seconds() > 0.3) {
                armTarget = specimenArmPos;
                powerArm = 0.4;
                armPosition = 2;

                toggleRodeDown = true;

                specimenUpArm = false;

            }

            if (specimenUpWrist && specimenUpArmTimer.seconds() > 0.6) {
                etesito.wristSpecimen();
                specimenUpWrist = false;

            }

            if (specimenArmSemiDown && specimenDownTimer.seconds() > 0.2) {
                armTarget = postSpecimenArmPos;
                powerArm = 0.4;

                toggleRodeDown = true;

                specimenArmSemiDown = false;
            }

            if (specimenOpenClaw && specimenDownTimer.seconds() > 0.4) {
                etesito.dropSpecimen();
                specimenOpenClaw = false;
            }

            if (specimenRodeDown && specimenDownTimer.seconds() > 0.6) {

                etesito.rodeController.targetPosition = 0;
                specimenRodeDown = false;
            }

            if (specimenWristDown && specimenDownTimer.seconds() > 0.65) {
                etesito.wristPickSpecimen();
                specimenWristDown = false;
            }

            if (specimenArmDown && specimenDownTimer.seconds() > 0.9) {
                powerArm = 0.1;
                armTarget = 0;

                armPosition = 0;
                specimenArmDown = false;
            }

            if (lowerWristBasket && lowerArmBasketTimer.seconds() > 0.3) {
                etesito.wristContract();
                lowerWristBasket = false;
            }

            if (contractArmBasket && lowerArmBasketTimer.seconds() > 0.6) {
                etesito.rodeController.targetPosition = 0;
                contractArmBasket = false;
            }

            if (contractArmDown && contractArmDownTimer.seconds() > 0.3) {
                etesito.rodeController.targetPosition = 0;
                contractArmDown = false;
            }

        }

        armMax = 20000;
        armMin = BasketArmpos - 350;

        etesito.armController.targetPosition = Range.clip(armTarget, armMin, armMax);

        previousArmPosition = armPosition;

        double deltaArmPos = etesito.armMotor.getCurrentPosition() - beforeArmPos;

        etesito.rodeController.targetPosition += (deltaArmPos / ratioArm);
        beforeArmPos = etesito.armMotor.getCurrentPosition();

        telemetry.addData("rodeTarget", etesito.rodeController.targetPosition);
        telemetry.addData("rodePos", etesito.rodeMotor.getCurrentPosition());
        telemetry.addData("armPos", etesito.armMotor.getCurrentPosition());
        telemetry.addData("rode Power", etesito.rodeMotor.getPower());

        double potenciaChassis;

        boolean slowMode = gamepad1.right_trigger > 0.5;

        if (slowMode) potenciaChassis = 0.4;
        else potenciaChassis = 1;

        double botHeading = etesito.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (gamepad1.dpad_up) {
            etesito.imu.resetYaw();
        }

            etesito.setDrivePower(
                    etesito.chassisPower(botHeading, potenciaChassis, gamepad1)[0],
                    etesito.chassisPower(botHeading, potenciaChassis, gamepad1)[1],
                    etesito.chassisPower(botHeading, potenciaChassis, gamepad1)[2],
                    etesito.chassisPower(botHeading, potenciaChassis, gamepad1)[3]
            );

    }


}

