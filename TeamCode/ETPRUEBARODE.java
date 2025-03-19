package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Comands.Etesito;
import org.firstinspires.ftc.teamcode.Comands.PIDFController;

@TeleOp(name = "ETESITO_rodetest")

public class ET extends OpMode {

    private final Etesito etesito = new Etesito();

    PIDFController armcontroller = new PIDFController(Etesito.armCoefficients);
    PIDFController rodecontroller = new PIDFController(Etesito.rodeCoefficients);
    PIDFController climbControllerRight = new PIDFController(Etesito.climbCoefficients);
    PIDFController climbControllerLeft = new PIDFController(Etesito.climbCoefficients);

    double rodeMin = Double.MIN_VALUE;
    double rodeMax = Double.MAX_VALUE;

    double armMin = Double.MIN_VALUE;
    double armMax = Double.MAX_VALUE;

    private int armPosition = 0;

    private double previousArmPosition;

    private double powerArm;

    private double powerRode;

    private double rdTarget;

    private double armTarget;

    boolean escalando = false;

    private final ElapsedTime manualWristTimer = new ElapsedTime();

    private final ElapsedTime extendArmHighBasketTimer = new ElapsedTime();
    private boolean extendArmHighBasket = false;

    private final ElapsedTime contractArmDownTimer = new ElapsedTime();
    private boolean contractArmDown = false;

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

    private final ElapsedTime lowBasketWristTimer = new ElapsedTime();
    private boolean lowBasketWrist = false;
    private boolean lowBasketArm = false;

    private final ElapsedTime colgandoAutomatizadoTimer = new ElapsedTime();
    private boolean colgandoAutomatizado = false;

    private final ElapsedTime timerColgar = new ElapsedTime();

    private final ElapsedTime togglePalito = new ElapsedTime();

    boolean manualRight;
    boolean manualLeft;

    boolean voltageIndicatorBoolean;

    @Override
    public void init() {
        etesito.init(hardwareMap, false);

        rodeMax = 250;

        etesito.imu.resetYaw();

        armcontroller.reset();
        rodecontroller.reset();
        climbControllerRight.reset();
        climbControllerLeft.reset();

        /*telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );*/

        telemetry.addLine("Robot Initialized.");
        telemetry.update();

        rdTarget = Etesito.downRodePos;
        armTarget = 0;

        rodecontroller.targetPosition = etesito.rodeMotor.getCurrentPosition();

    }

    int beforeArmPos = 0;

    @Override
    public void loop() {

        boolean colgando = gamepad1.left_trigger > 0.3;

        rodecontroller.targetPosition = Range.clip(rodecontroller.targetPosition, rodeMin, rodeMax);
        etesito.rodeMotor.setPower(rodecontroller.update(etesito.rodeMotor.getCurrentPosition()) * 0.6);

        if (colgando) {

            etesito.dropSpecimen();

            escalando = gamepad1.right_trigger > 0.5;

            if (escalando) {
                etesito.setLight("blue");
                rodecontroller.targetPosition = Etesito.climbingRodePos2;

            } else {
                etesito.setLight("green");
                rodecontroller.targetPosition = Etesito.climbingRodePos1;

            }

            telemetry.addLine("colgando_activado");

            if (gamepad2.right_stick_button) {
                telemetry.addLine("Right_Reset");

            } else if (gamepad2.left_stick_button) {
                telemetry.addLine("Left_Reset");

            }

            if (gamepad2.dpad_up) {
                colgandoAutomatizado = true;
                colgandoAutomatizadoTimer.reset();

                if (escalando) {
                    climbControllerRight.targetPosition = -4500;
                    climbControllerLeft.targetPosition = -4500;

                } else {
                    climbControllerRight.targetPosition = -3900;
                    climbControllerLeft.targetPosition = -3900;

                }

                voltageIndicatorBoolean = false;

            } else if (gamepad2.dpad_down) {

                etesito.servos_down();

                colgandoAutomatizado = false;

            }

            if (colgandoAutomatizado && colgandoAutomatizadoTimer.seconds() > 1.9) {

                if (escalando) {
                    etesito.servos_Climbing();

                } else {
                    etesito.servos_Uping();

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
                etesito.servos_down();

                telemetry.addLine("motor_forzado");
            }

            if (gamepad2.y) {
                etesito.servos_Uping();

            } else if (gamepad2.b) {
                etesito.servos_Climbing();

            } else if (gamepad2.a) {
                etesito.servos_down();

            }

            if (gamepad1.dpad_left || gamepad2.dpad_left) {
                rodecontroller.targetPosition = 0;

            } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
                if (escalando) rodecontroller.targetPosition = Etesito.climbingRodePos2;
                else rodecontroller.targetPosition = Etesito.climbingRodePos1;

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
                etesito.armMotor.setPower(-armcontroller.update(etesito.armMotor.getCurrentPosition()) * powerArm);

            }

            telemetry.addLine("colgando_desactivado");

            escalando = false;

            etesito.setLight("orange");

            if (gamepad2.left_bumper || gamepad2.right_bumper || gamepad1.left_bumper || gamepad1.right_bumper) {
                etesito.servos_Uping();
            }

            rodecontroller.targetPosition += (gamepad2.right_stick_y * 28);

            boolean extended = !(etesito.rodeMotor.getCurrentPosition() >= -200);

            if (gamepad2.right_stick_x > 0.9) {
                etesito.resetRodeEncoder();
                rodecontroller.reset();

            } else if (gamepad2.left_stick_x > 0.9) {
                etesito.resetArmEncoder();
                armcontroller.reset();
            }

            if (gamepad2.dpad_down) {
                rodecontroller.targetPosition = 0;
                extendArmHighBasket = false;

            } else if (gamepad2.dpad_up) {
                rodecontroller.targetPosition = rdTarget;

                if (armPosition == 2) {
                    specimenArmSemiDown = true;
                    specimenOpenClaw = true;
                    specimenWristDown = true;
                    specimenRodeDown = true;
                    specimenArmDown = true;

                    specimenDownTimer.reset();
                }
            }

            if (gamepad2.y && !extended) {
                powerArm = 0.4;
                armTarget = Etesito.highBasketArmpos;
                etesito.wrist_up();

                armPosition = 3;

                extendArmHighBasket = true;
                extendArmHighBasketTimer.reset();

                telemetry.addLine("Arm_up");

            } else if (gamepad2.x && !extended) {
                powerArm = 0.4;
                armTarget = Etesito.specimenArmPos;
                armPosition = 2;

                extendArmHighBasket = false;

                specimenUpWrist = true;
                specimenUpArmTimer.reset();

                telemetry.addLine("ArmSpecimen");

            } else if (gamepad2.b && !extended) {
                powerArm = 0.4;
                armTarget = Etesito.lowBasketArmpos;
                etesito.wrist_Medium();

                armPosition = 1;

                extendArmHighBasket = false;

                telemetry.addLine("Arm_mediumLow");

            } else if (gamepad2.a && !extended) {

                if (previousArmPosition == 2) {
                    etesito.wristPickSpecimen();

                } else {
                    etesito.wrist_Contract();

                }
                powerArm = 0.1;
                armTarget = 0;

                armPosition = 0;

                extendArmHighBasket = false;

                telemetry.addLine("Arm_down");

            }

            switch (armPosition) {
                case 3:
                    rdTarget = Etesito.highRodePos;
                    rodeMin = Etesito.highRodePos - 800;
                    break;

                case 2:
                    rdTarget = Etesito.specimenRodePos - 100;
                    rodeMin = Etesito.specimenRodePos - 900;
                    break;

                case 1:
                    rdTarget = Etesito.lowBasketRodePos;
                    rodeMin = Etesito.lowBasketRodePos - 200;
                    break;

                case 0:
                    rdTarget = Etesito.downRodePos;

                    if (etesito.wristIsMedium) {
                        rodeMin = Etesito.downRodePos - 150;

                    } else {
                        rodeMin = Etesito.downRodePos - 450;

                    }
                    break;
            }

            if (extendArmHighBasket && extendArmHighBasketTimer.seconds() >= 0.6) {
                rodecontroller.targetPosition = rdTarget;
                extendArmHighBasket = false;
            }

            if (gamepad2.right_trigger > 0.5 && manualWristTimer.seconds() > 0.2) {
                etesito.wrist_ManualUp();
                manualWristTimer.reset();

            } else if (gamepad2.left_trigger > 0.5 && manualWristTimer.seconds() > 0.2) {
                etesito.wrist_ManualDown();
                manualWristTimer.reset();

            } else {
                etesito.wrist_ManualMantener();

            }

            if (gamepad2.dpad_left) {
                contractArmBasket = false;

                switch (armPosition) {
                    case 3:
                        etesito.wrist_up();
                        break;
                    case 2:
                        etesito.wrist_Specimen();
                        specimenWristDown = false;
                        specimenArmSemiDown = false;
                        specimenOpenClaw = false;
                        specimenRodeDown = false;
                        specimenArmDown = false;
                        break;
                    case 1:
                        etesito.wrist_Medium();
                        break;
                    case 0:
                        etesito.wrist_Contract();

                        contractArmDown = true;
                        contractArmDownTimer.reset();

                        break;

                }

            } else if (gamepad2.dpad_right) {
                contractArmDown = false;

                switch (armPosition) {
                    case 3:
                        etesito.wrist_down();

                        contractArmBasket = true;
                        lowerArmBasketTimer.reset();
                        break;

                    case 2:
                        etesito.wrist_Specimen();
                        break;

                    case 1:
                        etesito.wrist_down();
                        break;

                    case 0:
                        etesito.wrist_down();

                        break;

                }

            }

            if (gamepad2.dpad_left && togglePalito.seconds() > 0.2){
                etesito.esconderPalito();
                togglePalito.reset();
            }else if (gamepad2.dpad_right && togglePalito.seconds() > 0.2){
                etesito.lanzarSample();
                togglePalito.reset();
            }

            if (gamepad1.left_bumper || gamepad1.b) {
                etesito.dropSample();

                contractArmDown = false;

                if (armPosition == 1) {
                    lowBasketWrist = true;
                    lowBasketArm = true;
                    lowBasketWristTimer.reset();

                } else if (armPosition == 3) {
                    lowerWristBasket = true;
                    contractArmBasket = true;
                    lowerArmBasketTimer.reset();
                }

            } else if (gamepad1.right_bumper || gamepad1.a) {
                etesito.pickSample();

                lowBasketWrist = false;
                lowerWristBasket = false;
                contractArmBasket = false;

                lowBasketArm = false;


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
                armTarget = Etesito.specimenArmPos;
                powerArm = 0.4;
                armPosition = 2;
                specimenUpArm = false;

            }

            if (specimenUpWrist && specimenUpArmTimer.seconds() > 0.6) {
                etesito.wrist_Specimen();
                specimenUpWrist = false;

            }

            if (specimenArmSemiDown && specimenDownTimer.seconds() > 0.2) {
                armTarget = Etesito.postSpecimenArmPos;
                powerArm = 0.4;
                specimenArmSemiDown = false;
            }

            if (specimenOpenClaw && specimenDownTimer.seconds() > 0.4) {
                etesito.dropSpecimen();
                specimenOpenClaw = false;
            }

            if (specimenRodeDown && specimenDownTimer.seconds() > 0.6) {
                rodecontroller.targetPosition = 0;
                specimenRodeDown = false;
            }

            if (specimenWristDown && specimenDownTimer.seconds() > 0.65) {
                etesito.wristPickSpecimen();
                specimenWristDown = false;
            }

            if (specimenArmDown && specimenDownTimer.seconds() > 0.9) {
                armTarget = 0;
                powerArm = 0.1;

                armPosition = 0;
                specimenArmDown = false;
            }

            if (lowerWristBasket && lowerArmBasketTimer.seconds() > 0.3) {
                etesito.wrist_Contract();
                lowerWristBasket = false;
            }

            if (contractArmBasket && lowerArmBasketTimer.seconds() > 0.6) {
                rodecontroller.targetPosition = 0;
                contractArmBasket = false;
            }

            if (contractArmDown && contractArmDownTimer.seconds() > 0.3) {
                rodecontroller.targetPosition = 0;
                contractArmDown = false;
            }

            if (lowBasketWrist && lowBasketWristTimer.seconds() >= 0.3) {
                etesito.wrist_up();
                lowBasketWrist = false;
            }

            if (lowBasketArm && lowBasketWristTimer.seconds() >= 0.5) {
                rodecontroller.targetPosition = 0;
                lowBasketArm = true;
            }
        }

        armMax = 20000;
        armMin = Etesito.highBasketArmpos - 350;

        armcontroller.targetPosition = Range.clip(armTarget, armMin, armMax);

        previousArmPosition = armPosition;

        double deltaArmPos = etesito.armMotor.getCurrentPosition() - beforeArmPos;

        rodecontroller.targetPosition += (deltaArmPos / etesito.ratioArm);
        beforeArmPos = etesito.armMotor.getCurrentPosition();

        telemetry.addData("rodePos", etesito.rodeMotor.getCurrentPosition());
        telemetry.addData("armPos", etesito.armMotor.getCurrentPosition());
        telemetry.addData("rode Power", etesito.rodeMotor.getPower());


        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double potenciaChassis;

        boolean slowMode = gamepad1.right_trigger > 0.5;

        if (slowMode) potenciaChassis = 0.4;
        else potenciaChassis = 1;

        double botHeading = etesito.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (gamepad1.dpad_up) {
            etesito.imu.resetYaw();
        }

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        double frontLeftPower = ((rotY + rotX + rx) / denominator) * potenciaChassis;
        double backLeftPower = ((rotY - rotX + rx) / denominator) * potenciaChassis;
        double frontRightPower = ((rotY - rotX - rx) / denominator) * potenciaChassis;
        double backRightPower = ((rotY + rotX - rx) / denominator) * potenciaChassis;

            etesito.setDrivePower(
                    frontLeftPower,
                    backLeftPower,
                    frontRightPower,
                    backRightPower
            );

    }

}

