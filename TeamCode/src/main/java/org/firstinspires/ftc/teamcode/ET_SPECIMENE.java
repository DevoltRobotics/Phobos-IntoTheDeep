package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Comands.Constants.basketArmPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.climbCoefficients;
import static org.firstinspires.ftc.teamcode.Comands.Constants.climbingRodePos1;
import static org.firstinspires.ftc.teamcode.Comands.Constants.downRodePos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.extensionLimit;
import static org.firstinspires.ftc.teamcode.Comands.Constants.highRodePos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.postSpecimenArmPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.preSpecimenRodePos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.ratioArm;
import static org.firstinspires.ftc.teamcode.Comands.Constants.servosClimbingPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.servosHangingPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.specimenArmPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.specimenRodePos;

import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Comands.Etesito;
import org.firstinspires.ftc.teamcode.Comands.PIDFController;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "ETESITO_SPECIMENE")

public class ET_SPECIMENE extends OpMode {

    private final Etesito etesito = new Etesito();

    private final Pose startPose = new Pose(0, 0, 0);

    PIDFController climbControllerRight = new PIDFController(climbCoefficients);
    PIDFController climbControllerLeft = new PIDFController(climbCoefficients);

    double rodeMin = Double.MIN_VALUE;
    double rodeMax = Double.MAX_VALUE;

    double armMin = Double.MIN_VALUE;
    double armMax = Double.MAX_VALUE;

    private int armPosition = 0;

    private double previousArmPosition;

    private double powerArm;

    private double rdTarget;

    private double armTarget;

    private final ElapsedTime toggleWristTimer = new ElapsedTime();

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

    private final ElapsedTime automaticHangingTimer = new ElapsedTime();
    private boolean automaticHang = false;

    private final ElapsedTime hangTimer = new ElapsedTime();

    private final ElapsedTime togglePalitoTimer = new ElapsedTime();

    boolean voltageIndicatorBoolean;

    boolean launchArmsAutomatic = false;
    boolean upServos = false;
    private final ElapsedTime launchArmsAutomaticTimer = new ElapsedTime();

    boolean launchArms = false;

    private boolean toggleHang;
    private final ElapsedTime toggleHangTimer = new ElapsedTime();

    private boolean downArmClimb;
    private final ElapsedTime downArmClimbTimer = new ElapsedTime();

    @Override
    public void init() {
        etesito.init(hardwareMap, false, true);

        launchArms = false;

        rodeMax = 250;

        climbControllerRight.reset();
        climbControllerLeft.reset();

        /*telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );*/

        telemetry.addLine("Robot Initialized.");
        telemetry.update();

        rdTarget = downRodePos;
        armTarget = 0;

        etesito.rodeController.targetPosition = etesito.rodeMotor.getCurrentPosition();
        etesito.armController.targetPosition = etesito.armMotor.getCurrentPosition();

        etesito.rodeController.targetPosition = etesito.rodeMotor.getCurrentPosition();

    }

    int beforeArmPos = 0;

    @Override
    public void loop() {

        etesito.rodeController.targetPosition = Range.clip(etesito.rodeController.targetPosition, rodeMin, rodeMax);
        etesito.rodeMotor.setPower(etesito.rodeController.update(etesito.rodeMotor.getCurrentPosition()) * 0.09);

        if (gamepad1.left_trigger > 0.3) {
            toggleHang = true;
        }

        if (toggleHang) {
            etesito.wristDown();

            telemetry.addLine("colgando_activado");

            if (!launchArms) {
                etesito.servosDownNomral();
                launchArmsAutomatic = true;
                upServos = true;
                launchArmsAutomaticTimer.reset();
            }

            if (launchArmsAutomatic && launchArmsAutomaticTimer.seconds() > 0.15) {
                etesito.launchHangArms();
                launchArmsAutomatic = false;

            }

            if (upServos && launchArmsAutomaticTimer.seconds() > 0.3) {
                etesito.servosHanging();
                upServos = false;

            }

            if (gamepad1.dpad_right) {
                etesito.armMotor.setPower(1);

                downArmClimb = true;
                downArmClimbTimer.reset();

            } else if (gamepad1.dpad_left) {
                etesito.armMotor.setPower(-1);
                etesito.armController.targetPosition = 0;

                downArmClimb = true;
                downArmClimbTimer.reset();

            } else {
                etesito.armMotor.setPower(0);
            }


            launchArms = true;

            etesito.dropSpecimen();

            //etesito.setLight("green");

            if (gamepad2.dpad_up) {
                automaticHang = true;
                automaticHangingTimer.reset();

                climbControllerRight.targetPosition = -537;
                climbControllerLeft.targetPosition = -537;

                voltageIndicatorBoolean = false;
            }

            if (automaticHang && automaticHangingTimer.seconds() > 1.9) {
                etesito.servosClimbing();
                automaticHang = false;
            }

            boolean manualRightHang = Math.abs(gamepad2.right_stick_y) > 0.3;
            boolean manualLeftHang = Math.abs(gamepad2.left_stick_y) > 0.3;

            if (manualRightHang) {
                etesito.mCR.setPower(gamepad2.right_stick_y);
                climbControllerRight.targetPosition = etesito.mCR.getCurrentPosition();
                telemetry.addLine("ManualRight");

            } else {
                etesito.mCR.setPower(climbControllerRight.update(etesito.mCR.getCurrentPosition()));

            }

            if (manualLeftHang) {
                etesito.mCL.setPower(gamepad2.left_stick_y);
                climbControllerLeft.targetPosition = etesito.mCL.getCurrentPosition();
                telemetry.addLine("ManualLeft");

            } else {
                etesito.mCL.setPower(climbControllerLeft.update(etesito.mCL.getCurrentPosition()));
            }

            if (manualRightHang || manualLeftHang) {
                voltageIndicatorBoolean = true;


            } else {
                hangTimer.reset();
                voltageIndicatorBoolean = false;

            }

            if ((Math.abs(etesito.mCL.getCurrent(CurrentUnit.AMPS)) > 4 || Math.abs(etesito.mCL.getCurrent(CurrentUnit.AMPS)) > 4)
                    && hangTimer.seconds() > 0.2 && voltageIndicatorBoolean) {
                etesito.servosDown();

                telemetry.addLine("motor_forzado");
            }

            if (gamepad2.y) {
                etesito.servosSb.mirrorServoPosCMD(servosHangingPos);

            } else if (gamepad2.b) {
                etesito.servosSb.mirrorServoPosCMD(servosClimbingPos);

            } else if (gamepad2.a) {
                etesito.servosDown();

            }

            if (gamepad2.right_bumper) {
                etesito.launchHangArms();

            } else if (gamepad2.left_bumper) {
                etesito.supportHangArms();

            }

            if (gamepad2.dpad_left) {
                etesito.rodeController.targetPosition = 0;

            } else if (gamepad2.dpad_right) {
                etesito.rodeController.targetPosition = climbingRodePos1;

            }

            telemetry.addData("HangTimer", hangTimer.seconds());

        } else {
            if (gamepad2.start) {
                etesito.resetArmEncoder();
            }

            if (gamepad2.options) {
                etesito.resetRodeEncoder();

            }

            boolean manualArm = Math.abs(gamepad2.left_stick_y) > 0.5;

            if (manualArm) {
                etesito.armMotor.setPower(-gamepad2.left_stick_y * 0.35);
                armTarget = (etesito.armMotor.getCurrentPosition());

            } else {
                etesito.armMotor.setPower(-etesito.armController.update(etesito.armMotor.getCurrentPosition()) * powerArm);
            }

            etesito.rodeController.targetPosition += gamepad2.right_stick_y * 40;

            telemetry.addLine("colgando_desactivado");

            //etesito.setLight("orange");

            if (gamepad2.left_bumper || gamepad2.right_bumper || gamepad1.left_bumper || gamepad1.right_bumper) {
                etesito.servosHanging();
                etesito.supportHangArms();
            }

            boolean extended = !(etesito.rodeMotor.getCurrentPosition() >= -400);

            if (gamepad2.dpad_down) {
                extendArmHighBasket = false;

                etesito.rodeController.targetPosition = 0;

            } else if (gamepad2.dpad_up) {

                if (armPosition == 2) {
                    specimenArmSemiDown = true;
                    specimenOpenClaw = true;
                    specimenWristDown = true;
                    specimenRodeDown = true;
                    specimenArmDown = true;

                    specimenDownTimer.reset();
                }

                etesito.rodeController.targetPosition = rdTarget;
            }

            if (gamepad2.y && !extended) {
                powerArm = 0.4;
                armTarget = basketArmPos;
                etesito.wristUp();

                armPosition = 1;

                extendArmHighBasket = true;
                extendArmHighBasketTimer.reset();

            } else if (gamepad2.b && !extended) {
                powerArm = 0.4;
                armTarget = basketArmPos;
                etesito.wristUp();

                armPosition = 1;

            } else if (gamepad2.x && !extended) {
                powerArm = 0.4;
                armTarget = specimenArmPos;
                etesito.wristSpecimen();

                armPosition = 2;

                extendArmHighBasket = false;

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

            }

            switch (armPosition) {
                case 2:
                    rdTarget = specimenRodePos;
                    rodeMin = specimenRodePos - 900;
                    break;

                case 1:
                    rdTarget = highRodePos;
                    rodeMin = highRodePos - 800;
                    break;

                case 0:
                    rdTarget = downRodePos;

                    if (etesito.wristIsMedium) {
                        rodeMin = extensionLimit;

                    } else {
                        rodeMin = extensionLimit - 100;

                    }
                    break;
            }

            if (extendArmHighBasket && extendArmHighBasketTimer.seconds() >= 0.3) {
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
                    case 2:
                        etesito.wristSpecimen();
                        specimenWristDown = false;
                        specimenArmSemiDown = false;
                        specimenOpenClaw = false;
                        specimenRodeDown = false;
                        specimenArmDown = false;
                        break;
                    case 1:
                        etesito.wristUp();
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
                        if (toggleWristTimer.seconds() > 0.35) {
                            if (etesito.wristIsMedium) {
                                etesito.wristDown();
                            } else {
                                etesito.wristContract();
                            }
                            toggleWristTimer.reset();
                        }

                        break;
                }
            }

            if (gamepad1.dpad_left && togglePalitoTimer.seconds() > 0.2) {
                etesito.esconderPalito();
                togglePalitoTimer.reset();
            } else if (gamepad1.dpad_right && togglePalitoTimer.seconds() > 0.2) {
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

                }

            } else if (gamepad1.right_bumper || gamepad1.a) {
                etesito.pickSample();

                lowerWristBasket = false;
                contractArmBasket = false;

            } else {
                etesito.intake0();

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

                specimenUpArm = false;
            }

            if (specimenUpWrist && specimenUpArmTimer.seconds() > 0.6) {
                etesito.wristSpecimen();
                etesito.rodeController.targetPosition = preSpecimenRodePos;
                specimenUpWrist = false;
            }

            if (specimenArmSemiDown && specimenDownTimer.seconds() > 0.15) {
                armTarget = postSpecimenArmPos;
                etesito.wristDown();
                powerArm = 0.4;

                specimenArmSemiDown = false;
            }

            if (specimenOpenClaw && specimenDownTimer.seconds() > 0.3) {
                etesito.dropSpecimen();
                specimenOpenClaw = false;
            }

            if (specimenRodeDown && specimenDownTimer.seconds() > 0.6) {
                etesito.rodeController.targetPosition = 0;
                specimenRodeDown = false;
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

            if (contractArmBasket && lowerArmBasketTimer.seconds() > 0.5) {
                etesito.rodeController.targetPosition = 0;
                contractArmBasket = false;
            }

            if (contractArmDown && contractArmDownTimer.seconds() > 0.3) {
                etesito.rodeController.targetPosition = 0;
                contractArmDown = false;
            }

        }

        armMax = 20000;
        armMin = basketArmPos - 350;

        etesito.armController.targetPosition = Range.clip(armTarget, armMin, armMax);

        previousArmPosition = armPosition;

        double deltaArmPos = etesito.armMotor.getCurrentPosition() - beforeArmPos;

        etesito.rodeController.targetPosition += (deltaArmPos / ratioArm);
        beforeArmPos = etesito.armMotor.getCurrentPosition();

        telemetry.addData("rodeTarget", etesito.rodeController.targetPosition);
        telemetry.addData("rodePos", etesito.rodeMotor.getCurrentPosition());
        telemetry.addData("armPos", etesito.armMotor.getCurrentPosition());
        telemetry.addData("rode Power", etesito.rodeMotor.getPower());
        telemetry.addData("arm Power", etesito.armMotor.getPower());

        double turbo = 1 - (gamepad1.right_trigger * 0.65);

        /* Telemetry Outputs of our Follower */

        double chassisPower = 1 - (gamepad1.right_trigger * 0.65);

        if (gamepad1.dpad_up) {
            etesito.imu.resetYaw();
        }

        double botHeading = etesito.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        etesito.setDrivePower(
                etesito.chassisPower(botHeading, chassisPower, gamepad1)[0],
                etesito.chassisPower(botHeading, chassisPower, gamepad1)[1],
                etesito.chassisPower(botHeading, chassisPower, gamepad1)[2],
                etesito.chassisPower(botHeading, chassisPower, gamepad1)[3]
        );

    }


}

