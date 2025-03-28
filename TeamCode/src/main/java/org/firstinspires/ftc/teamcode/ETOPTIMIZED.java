package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Comands.Constants.BasketArmpos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.basketWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.climbCoefficients;
import static org.firstinspires.ftc.teamcode.Comands.Constants.climbingRodePos1;
import static org.firstinspires.ftc.teamcode.Comands.Constants.climbingRodePos2;
import static org.firstinspires.ftc.teamcode.Comands.Constants.closeClawPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.contractWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.downRodePos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.downWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.highRodePos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.launchArmsPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.ratioArm;
import static org.firstinspires.ftc.teamcode.Comands.Constants.ratioRodeSmooth;
import static org.firstinspires.ftc.teamcode.Comands.Constants.servosClimbingPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.servosHangingPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.specimenArmPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.specimenRodePos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.specimenWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.supportArmsPos;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Comands.Etesito;
import org.firstinspires.ftc.teamcode.Comands.PIDFController;

@TeleOp(name = "ETESITO_OPTIMIZED")

public class ETOPTIMIZED extends OpMode {

    private final Etesito etesito = new Etesito();

    PIDFController climbControllerRight = new PIDFController(climbCoefficients);
    PIDFController climbControllerLeft = new PIDFController(climbCoefficients);

    double rodeMin = Double.MIN_VALUE;
    double rodeMax = Double.MAX_VALUE;

    private double previousArmPosition;

    private int rdTarget;

    boolean climb = false;

    private final ElapsedTime toggleWristTimer = new ElapsedTime();

    private final ElapsedTime manualWristTimer = new ElapsedTime();

    private final ElapsedTime automaticHangingTimer = new ElapsedTime();
    private boolean automaticHang = false;

    private final ElapsedTime hangTimer = new ElapsedTime();

    private boolean toggleHang = false;
    private final ElapsedTime toggleHangTimer = new ElapsedTime();

    private boolean voltageIndicatorBoolean;

    boolean lanzarBrazitos = false;
    @Override
    public void init() {
        etesito.init(hardwareMap, true, true);

        etesito.rodeSb.setDefaultCommand(etesito.rodeSb.rodeUpdate());
        etesito.armSb.setDefaultCommand(etesito.armSb.armUpdate());

        rodeMax = 250;

        climbControllerRight.reset();
        climbControllerLeft.reset();

        /*telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );*/


        telemetry.addLine("Robot Initialized.");
        telemetry.update();
    }

    int beforeArmPos = 0;

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();

        if (gamepad1.left_trigger > 0.3 && toggleHangTimer.seconds() > 0.5){
            toggleHang = !toggleHang;
            toggleHangTimer.reset();
        }

        if (toggleHang) {

            telemetry.addLine("colgando_activado");

            if (!lanzarBrazitos) {
                etesito.launchArms().schedule();
            }

            lanzarBrazitos = true;

            etesito.dropSpecimen();

            etesito.lightSb.colorCmd("green");

            if (gamepad2.dpad_up) {
                automaticHang = true;
                automaticHangingTimer.reset();

                climbControllerRight.targetPosition = 0;
                climbControllerLeft.targetPosition = 0;

                voltageIndicatorBoolean = false;
            }

            if (automaticHang && automaticHangingTimer.seconds() > 1.9) {
                etesito.servosSb.mirrorServoPosCMD(servosClimbingPos);
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
                hangTimer.reset();

            } else {
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
                etesito.launchersSb.mirrorServoPosCMD(launchArmsPos);

            } else if (gamepad2.left_bumper) {
                etesito.launchersSb.mirrorServoPosCMD(supportArmsPos);

            }

            if (gamepad2.dpad_left) {
                etesito.rodeController.targetPosition = 0;

            } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
                if (climb) etesito.rodeController.targetPosition = climbingRodePos2;
                else etesito.rodeController.targetPosition = climbingRodePos1;

            }

        } else {
            telemetry.addLine("colgando_desactivado");
            climb = false;

            etesito.lightSb.colorCmd("orange");

            etesito.mCR.setPower(climbControllerRight.update(etesito.mCR.getCurrentPosition()));
            etesito.mCL.setPower(climbControllerLeft.update(etesito.mCL.getCurrentPosition()));

            climbControllerRight.targetPosition = etesito.mCR.getCurrentPosition();
            climbControllerLeft.targetPosition = etesito.mCL.getCurrentPosition();

            etesito.rodeController.targetPosition += gamepad2.right_stick_y * 40;
            etesito.armController.targetPosition += gamepad2.left_stick_y * 40;

            if (gamepad2.start) {
                etesito.resetArmEncoder();
            }
            if (gamepad2.options) {
                etesito.resetRodeEncoder();
            }

            if (gamepad2.left_bumper || gamepad2.right_bumper || gamepad1.left_bumper || gamepad1.right_bumper) {
                etesito.servosHanging();
                etesito.launchersSb.mirrorServoPosCMD(supportArmsPos);
                etesito.esconderPalito();
            }

            if (gamepad2.dpad_down) {
                if (etesito.armPosition == 1 || etesito.armPosition == 2) {
                    etesito.rodeSb.rodeToPosSmooth(0, etesito.rodeMotor.getCurrentPosition() * (ratioRodeSmooth)).schedule();

                } else {
                    etesito.rodeSb.rodeToPos(0).schedule();
                }

            } else if (gamepad2.dpad_up) {
                if (etesito.armPosition == 2) {
                    etesito.putSpecimenCmd().schedule();

                }
                etesito.rodeSb.rodeToPos(rdTarget).schedule();

            }

            boolean extended = !(etesito.rodeMotor.getCurrentPosition() >= -400);

            if (gamepad2.y && !extended) {
                etesito.extendArmHighBasketCmd().schedule();
                etesito.armPosition = 1;

            } else if (gamepad2.x && !extended) {
                etesito.armSb.armToPos(BasketArmpos).schedule();
                etesito.wristSb.servoPosCMD(basketWristPos).schedule();
                etesito.armPosition = 1;

            } else if (gamepad2.b && !extended) {
                etesito.armSb.armToPos(specimenArmPos).schedule();
                etesito.armPosition = 2;

            } else if (gamepad2.a && !extended) {
                if (previousArmPosition == 2) {
                    etesito.wristSb.servoPosCMD(downWristPos).schedule();

                } else {
                    etesito.wristSb.servoPosCMD(contractWristPos).schedule();

                }
                etesito.armSb.armToPosSmooth(0, 1).schedule();
                etesito.armPosition = 0;

            }

            switch (etesito.armPosition) {
                case 2:
                    rdTarget = specimenRodePos - 100;
                    rodeMin = specimenRodePos - 900;
                    break;

                case 1:
                    rdTarget = highRodePos;
                    rodeMin = highRodePos - 800;
                    break;

                case 0:
                    rdTarget = downRodePos;

                    if (etesito.wristIsMedium) {
                        rodeMin = downRodePos - 150;

                    } else {
                        rodeMin = downRodePos - 350;
                    }
                    break;
            }

            if (manualWristTimer.seconds() > 0.2) {

                if (gamepad2.right_trigger > 0.5) {
                    etesito.wristSb.servoPosCMD(etesito.wrist.getPosition() + 0.03).schedule();
                    manualWristTimer.reset();

                } else if (gamepad2.left_trigger > 0.5) {
                    etesito.wristSb.servoPosCMD(etesito.wrist.getPosition() - 0.03).schedule();
                    manualWristTimer.reset();

                }
            }else {
                etesito.wristSb.servoPosCMD(etesito.wrist.getPosition());
            }

            if (gamepad2.dpad_left) {
                switch (etesito.armPosition) {
                    case 2:
                        etesito.wristSb.servoPosCMD(specimenWristPos).schedule();
                        break;
                    case 1:
                        etesito.wristSb.servoPosCMD(basketWristPos).schedule();
                        break;
                    case 0:
                        etesito.contractArmDownCmd().schedule();
                        break;
                }

            } else if (gamepad2.dpad_right) {
                switch (etesito.armPosition) {
                    case 2:
                        etesito.wristSb.servoPosCMD(specimenWristPos).schedule();
                        break;
                    case 1:
                        etesito.wristSb.servoPosCMD(contractWristPos).schedule();
                        break;
                    case 0:
                        if (toggleWristTimer.seconds() > 0.15) {
                            if (etesito.wristIsMedium){
                                etesito.wristSb.servoPosCMD(downWristPos).schedule();
                            }else {
                                etesito.wristSb.servoPosCMD(contractWristPos).schedule();
                            }
                            toggleWristTimer.reset();
                        }
                        break;
                }
            }

            if (gamepad1.left_bumper || gamepad1.b) {
                etesito.intakeSb.crservoCMD(-1).schedule();

                if (etesito.armPosition == 1) {
                    etesito.contractArmBasketCmd().schedule();
                }

            } else if (gamepad1.right_bumper || gamepad1.a) {
                etesito.intakeSb.crservoCMD(1).schedule();

            } else {
                etesito.intakeSb.crservoCMD(0).schedule();

            }

            if (gamepad2.left_bumper) {
                etesito.dropSpecimen();

            }
            else if (gamepad2.right_bumper) {

                if (etesito.armPosition == 0) {
                    etesito.pickSpecimenCmd().schedule();
                    etesito.armPosition = 2;

                } else {
                    etesito.clawSb.servoPosCMD(closeClawPos).schedule();

                }
            }
        }

        double deltaArmPos = etesito.armMotor.getCurrentPosition() - beforeArmPos;

//      etesito.rodeController.targetPosition += (deltaArmPos / ratioArm);
        beforeArmPos = etesito.armMotor.getCurrentPosition();

        previousArmPosition = etesito.armPosition;

        telemetry.addData("armPositionNumber", etesito.armPosition);
        telemetry.addData("rodeTarget", etesito.rodeController.targetPosition);
        telemetry.addData("rodePos", etesito.rodeMotor.getCurrentPosition());
        telemetry.addData("armPos", etesito.armMotor.getCurrentPosition());
        telemetry.addData("armTarget", etesito.armController.targetPosition);

        telemetry.addData("servitosPos", etesito.launcherLeft.getPosition());

        if (gamepad1.dpad_up) {
                etesito.imu.resetYaw();
            }

        double chassisPower = 1 - (gamepad1.right_trigger * 0.65);

        double botHeading = etesito.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        /* etesito.setDrivePower(
                etesito.chassisPower(botHeading, chassisPower, gamepad1)[0],
                etesito.chassisPower(botHeading, chassisPower, gamepad1)[1],
                etesito.chassisPower(botHeading, chassisPower, gamepad1)[2],
                etesito.chassisPower(botHeading, chassisPower, gamepad1)[3]
            );

            */

        }
}

