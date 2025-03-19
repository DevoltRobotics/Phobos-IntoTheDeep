package org.firstinspires.ftc.teamcode.Comands;

import static org.firstinspires.ftc.teamcode.Comands.Constants.BasketArmpos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.basketWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.climbingRodePos1;
import static org.firstinspires.ftc.teamcode.Comands.Constants.climbingRodePos2;
import static org.firstinspires.ftc.teamcode.Comands.Constants.contractWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.down2RodePos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.downRodePos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.downWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.highRodePos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.postSpecimenArmPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.ratioArm;
import static org.firstinspires.ftc.teamcode.Comands.Constants.specimenRodePos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "ETESITO_OPTIMIZED")

public class ETOPTIMIZED extends OpMode {

    private final Etesito etesito = new Etesito();

    PIDFController climbControllerRight = new PIDFController(Etesito.climbCoefficients);
    PIDFController climbControllerLeft = new PIDFController(Etesito.climbCoefficients);

    double currentRodeTicks = 0;

    double rodeMin = Double.MIN_VALUE;
    double rodeMax = Double.MAX_VALUE;

    double armMin = Double.MIN_VALUE;
    double armMax = Double.MAX_VALUE;

    private int armPosition = 0;

    private double previousArmPosition;

    private int rdTarget;

    boolean escalando = false;

    private final ElapsedTime toggleWristTimer = new ElapsedTime();

    private final ElapsedTime timerRode = new ElapsedTime();

    private boolean toggleRodeDown = true;
    private final ElapsedTime toggleRodeDownTimer = new ElapsedTime();

    private final ElapsedTime manualWristTimer = new ElapsedTime();

    private final ElapsedTime colgandoAutomatizadoTimer = new ElapsedTime();
    private boolean colgandoAutomatizado = false;

    private final ElapsedTime timerColgar = new ElapsedTime();

    boolean manualRight;
    boolean manualLeft;

    boolean voltageIndicatorBoolean;

    @Override
    public void init() {
        etesito.init(hardwareMap, false);

        toggleRodeDown = true;

        rodeMax = 250;

        etesito.imu.resetYaw();

        climbControllerRight.reset();
        climbControllerLeft.reset();

        /*telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );*/


        telemetry.addLine("Robot Initialized.");
        telemetry.update();

        rdTarget = downRodePos;

    }

    int beforeArmPos = 0;

    int beforeRodePos = 0;

    @Override
    public void loop() {

        etesito.armSb.armUpdate().schedule();

        boolean colgando = gamepad1.left_trigger > 0.3;

        if (colgando) {

            etesito.lanzarBrazitosColgada();

            etesito.servosUping();
            escalando = gamepad1.right_trigger > 0.5;

            etesito.dropSpecimen();

            if (escalando) {
                etesito.setLight("blue");
                etesito.armSb.rodeToPos(climbingRodePos2);

            } else {
                etesito.setLight("green");
                etesito.armSb.rodeToPos(climbingRodePos1);

            }

            telemetry.addLine("colgando_activado");

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
                etesito.mCR.setPower(gamepad2.right_stick_y);
                climbControllerRight.targetPosition = etesito.mCR.getCurrentPosition();
                telemetry.addLine("ManualRight");

            } else {
                etesito.mCR.setPower(climbControllerRight.update(etesito.mCR.getCurrentPosition()));

            }

            if (manualLeft) {
                etesito.mCL.setPower(gamepad2.left_stick_y);
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

            }else if (gamepad2.left_bumper){
                etesito.guardarBrazitosColgada();

            }

            if (gamepad2.dpad_left) {
                etesito.rodeController.targetPosition = 0;

            } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
                if (escalando) etesito.rodeController.targetPosition = climbingRodePos2;
                else etesito.rodeController.targetPosition = climbingRodePos1;

            }

        }
        else {
            etesito.mCR.setPower(climbControllerRight.update(etesito.mCR.getCurrentPosition()));
            etesito.mCL.setPower(climbControllerLeft.update(etesito.mCL.getCurrentPosition()));

            climbControllerRight.targetPosition = etesito.mCR.getCurrentPosition();
            climbControllerLeft.targetPosition = etesito.mCL.getCurrentPosition();

            if (gamepad2.start) {
                etesito.resetArmEncoder();
            }
            if (gamepad2.options) {
                etesito.resetRodeEncoder();

            }

            boolean manualArm = Math.abs(gamepad2.left_stick_y) > 0.3;

            if (manualArm) {
                etesito.armMotor.setPower(-gamepad2.left_stick_y * 0.35);
                etesito.armSb.rodeToPos(etesito.armMotor.getCurrentPosition());

            } else {
                etesito.armMotor.setPower(-etesito.armController.update(etesito.armMotor.getCurrentPosition()) * 0.4);

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
                toggleRodeDown = true;

                if (armPosition == 1 || armPosition == 2) {
                    etesito.armSb.rodeToPosSmooth(0, 0.8);

                } else {
                    etesito.rodeController.targetPosition = 0;

                }

            } else if (gamepad2.dpad_up) {

                if (armPosition == 2) {
                    etesito.putSpecimenCmd().schedule();

                } else if (armPosition == 0) {
                    if (toggleRodeDownTimer.seconds() > 0.2) {
                        if (toggleRodeDown) {
                            toggleRodeDown = false;

                        } else {
                            toggleRodeDown = true;

                        }
                        toggleRodeDownTimer.reset();

                    }

                }
                etesito.armSb.rodeToPos(rdTarget);
            }

            if (gamepad2.y && !extended) {
                etesito.extendArmHighBasketCmd().schedule();
                toggleRodeDown = true;

                armPosition = 1;

                telemetry.addLine("Arm_up");

            } else if (gamepad2.x && !extended) {
                etesito.armSb.armToPos(BasketArmpos).schedule();
                etesito.wristAction.servoPosCMD(basketWristPos);

                armPosition = 1;

                toggleRodeDown = true;

                telemetry.addLine("Arm_mediumLow");

            } else if (gamepad2.b && !extended) {
                etesito.armSb.armToPos(postSpecimenArmPos).schedule();
                armPosition = 2;

                toggleRodeDown = true;

                telemetry.addLine("ArmSemiDown");

            } else if (gamepad2.a && !extended) {
                if (previousArmPosition == 2) {
                    etesito.wristAction.servoPosCMD(downWristPos);

                } else {
                    etesito.wristAction.servoPosCMD(contractWristPos);

                }
                etesito.armSb.armToPos(0).schedule();

                armPosition = 0;

                toggleRodeDown = true;

                telemetry.addLine("Arm_down");

            }

            switch (armPosition) {
                case 2:
                    rdTarget = specimenRodePos - 100;
                    rodeMin = specimenRodePos - 900;
                    break;

                case 1:
                    rdTarget = highRodePos;
                    rodeMin = highRodePos - 800;
                    break;

                case 0:
                    if (toggleRodeDown) {
                        rdTarget = downRodePos;

                    } else {
                        rdTarget = down2RodePos;
                    }

                    if (etesito.wristIsMedium) {
                        rodeMin = downRodePos - 150;

                    } else {
                        rodeMin = downRodePos - 350;
                    }
                    break;
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
                switch (armPosition) {
                    case 2:
                        etesito.wristSpecimen();
                        break;
                    case 1:
                        etesito.wristUp();
                        break;
                    case 0:
                        etesito.contractArmDownCmd().schedule();
                        break;
                }

            } else if (gamepad2.dpad_right) {
                switch (armPosition) {
                    case 2:
                        etesito.wristSpecimen();
                        break;
                    case 1:
                        etesito.wristContract();
                        break;
                    case 0:
                        if (toggleWristTimer.seconds() > 0.15) {
                            etesito.wristToggle();
                            toggleWristTimer.reset();
                        }
                        break;
                }
            }

            if (gamepad1.dpad_left) {
                etesito.esconderPalito();
            } else if (gamepad1.dpad_right) {
                etesito.lanzarSample();
            }

            if (gamepad1.left_bumper || gamepad1.b) {
                etesito.dropSample();

                if (armPosition == 1) {
                    etesito.contractArmBasketCmd().schedule();
                }

            } else if (gamepad1.right_bumper || gamepad1.a) {
                etesito.pickSample();

            } else {
                etesito.pickSampleSlow();

            }

            if (gamepad2.left_bumper) {
                etesito.dropSpecimen();

            } else if (gamepad2.right_bumper && !extended) {
                etesito.pickSpecimen();

                if (armPosition == 0) {
                    etesito.pickSpecimenCmd();
                }
            }

        }

        previousArmPosition = armPosition;

        telemetry.addData("rodeTarget", etesito.rodeController.targetPosition);
        telemetry.addData("rodePos", etesito.rodeMotor.getCurrentPosition());
        telemetry.addData("armPos", etesito.armMotor.getCurrentPosition());
        telemetry.addData("rode Power", etesito.rodeMotor.getPower());

        double chassisPower = 1 - (gamepad1.right_trigger * 0.65);;

        double botHeading = etesito.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (gamepad1.dpad_up) {
            etesito.imu.resetYaw();
        }

            etesito.setDrivePower(
                    etesito.chassisPower(botHeading, chassisPower, gamepad1)[0],
                    etesito.chassisPower(botHeading, chassisPower, gamepad1)[1],
                    etesito.chassisPower(botHeading, chassisPower, gamepad1)[2],
                    etesito.chassisPower(botHeading, chassisPower, gamepad1)[3]
            );

    }


}

