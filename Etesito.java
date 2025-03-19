package org.firstinspires.ftc.teamcode.Comands;

import com.acmerobotics.dashboard.config.Config;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Comands.Constants.BasketArmpos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.basketWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.closeClawPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.contractWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.downWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.esconderPalitoPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.highRodePos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.initWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.lanzarBrazitosPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.lanzarSamplePos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.mediumWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.openClawPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.pickSpecimenWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.postSpecimenArmPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.servosClimbingPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.servosTEstPost;
import static org.firstinspires.ftc.teamcode.Comands.Constants.servosUpingPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.sostenerBrazitosPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.specimenArmPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.specimenRodePos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.specimenWristPos;

@Config
public class Etesito {

    public static PIDFController.PIDCoefficients armCoefficients = new PIDFController.PIDCoefficients(0.0015, 0, 0.0017);
    public static PIDFController.PIDCoefficients rodeCoefficients = new PIDFController.PIDCoefficients(0.015, 0, 0.007);

    public static PIDFController.PIDCoefficients climbCoefficients = new PIDFController.PIDCoefficients(0.015, 0, 0.0017);

    public static PIDFController.PIDCoefficients chassisCoefficients = new PIDFController.PIDCoefficients(0.06, 0, 0.035);

    public PIDFController armController = new PIDFController(armCoefficients);
    public PIDFController rodeController = new PIDFController(rodeCoefficients);
    public PIDFController chassiscontroller = new PIDFController(chassisCoefficients);
    PIDFController climbControllerRight = new PIDFController(climbCoefficients);
    PIDFController climbControllerLeft = new PIDFController(climbCoefficients);

    public DcMotorEx FL, BL, BR, FR, armMotor, rodeMotor, mCR, mCL;

    public Servo sC1, sC2, claw, wrist, light, palito, palitoLeft, palitoRight;

    public CRServo intake;

    public boolean wristIsMedium;

    public IMU imu;

    public ServoAction clawAction;
    public ServoAction wristAction;
    public ServoAction servosAction;
    public CRservoAction intakeAction;
    public ArmSb armSb;


    public boolean toggleWrist = true;

    public void init(HardwareMap hardwareMap, boolean resetRode) {


        rodeController.reset();
        armController.reset();
        chassiscontroller.reset();

        armSb.setDefaultCommand(armSb.armUpdate());

        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rodeMotor = hardwareMap.get(DcMotorEx.class, "rd");
        rodeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        if (resetRode) {
            rodeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rodeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        palitoLeft = hardwareMap.get(Servo.class, "plL");
        palitoRight = hardwareMap.get(Servo.class, "plR");
        claw = hardwareMap.servo.get("cw");
        intake = hardwareMap.get(CRServo.class, "in");
        wrist = hardwareMap.servo.get("wr");
        palito = hardwareMap.servo.get("sr");

        light = hardwareMap.get(Servo.class, "rgb");

        mCR = hardwareMap.get(DcMotorEx.class, "mc1");
        mCL = hardwareMap.get(DcMotorEx.class, "mc2");
        mCR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mCL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mCR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mCR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mCL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mCL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mCL.setDirection(DcMotorSimple.Direction.REVERSE);

        sC1 = hardwareMap.get(Servo.class, "sc1");
        sC2 = hardwareMap.get(Servo.class, "sc2");

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);


        FL = hardwareMap.get(DcMotorEx.class, "fl"); //ex1
        BL = hardwareMap.get(DcMotorEx.class, "bl"); //ex2
        BR = hardwareMap.get(DcMotorEx.class, "br"); //ex3
        FR = hardwareMap.get(DcMotorEx.class, "fr"); //ex4

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);

        wristIsMedium = false;

        clawAction = new ServoAction(claw);
        wristAction = new ServoAction(wrist);
        servosAction = new ServoAction(sC1, sC2);
        intakeAction = new CRservoAction(intake);

        armSb = new ArmSb(rodeMotor, rodeController, armMotor, armController);

        CommandScheduler.getInstance().registerSubsystem(clawAction);
        CommandScheduler.getInstance().registerSubsystem(wristAction);
        CommandScheduler.getInstance().registerSubsystem(servosAction);
        CommandScheduler.getInstance().registerSubsystem(intakeAction);

        CommandScheduler.getInstance().registerSubsystem(armSb);
    }

    public void lanzarBrazitosColgada() {
        palitoRight.setPosition(0.5 - sostenerBrazitosPos);
        palitoLeft.setPosition(0.5 + sostenerBrazitosPos);
    }

    public void guardarBrazitosColgada() {
        palitoRight.setPosition(0.5 - lanzarBrazitosPos);
        palitoLeft.setPosition(0.5  + lanzarBrazitosPos);
    }

    public void pickSpecimen() {
        claw.setPosition(closeClawPos);
    }

    public void dropSpecimen() {
        claw.setPosition(openClawPos);
    }

    public void pickSample() {
        intake.setPower(1);
    }

    public void pickSampleSlow() {
        intake.setPower(0.1);
    }

    public void dropSample() {
        intake.setPower(-1);
    }

    public void intake0() {
        intake.setPower(0);
    }

    public void wristDown() {
        wrist.setPosition(downWristPos);
        wristIsMedium = false;
    }

    public void wristContract() {
        wrist.setPosition(contractWristPos);
        wristIsMedium = true;
    }

    public void wristToggle() {
        if (wristIsMedium){
            wristDown();

        }else {
            wristContract();
        }
    }

    public void wristMedium() {
        wrist.setPosition(mediumWristPos);
        wristIsMedium = true;
    }

    public void wristUp() {
        wrist.setPosition(basketWristPos);
        wristIsMedium = true;
    }

    public void wristPickSpecimen() {
        wrist.setPosition(pickSpecimenWristPos);
        wristIsMedium = false;
    }

    public void wristSpecimen() {
        wrist.setPosition(specimenWristPos);
        wristIsMedium = true;
    }

    public void wristInit() {
        wrist.setPosition(initWristPos);
        wristIsMedium = false;
        toggleWrist = true;
    }

    public void wristManualUp() {
        wrist.setPosition(wrist.getPosition() + 0.03);
    }

    public void wristManualDown() {
        wrist.setPosition(wrist.getPosition() - 0.03);
    }

    public void wristManualMantener() {
        wrist.setPosition(wrist.getPosition());
    }

    public void lanzarSample() {
        palito.setPosition(lanzarSamplePos);
    }

    public void esconderPalito() {
        palito.setPosition(esconderPalitoPos);
    }

    public void servosOff() {
        sC1.getController().pwmDisable();
        sC2.getController().pwmDisable();
    }

    public void servosUping() {

        sC1.setPosition(0.5 + servosUpingPos);
        sC2.setPosition(0.5 - servosUpingPos);
    }

    public void servosClimbing() {

        sC1.setPosition(0.5 + servosClimbingPos);
        sC2.setPosition(0.5 - servosClimbingPos);
    }

    public void servos_test() {

        sC1.setPosition(0.5 + servosTEstPost);
        sC2.setPosition(0.5 - servosTEstPost);

    }

    public void servosDown() {
        ElapsedTime timer = new ElapsedTime();

        sC1.setPosition(0);
        sC2.setPosition(1);

        if (timer.seconds() > 0.2) {
            servosOff();

        }
    }

    public void setLight(String color) {
        switch (color) {
            case "purple":
                light.setPosition(0.72);
                break;
            case "blue":
                light.setPosition(0.62);
                break;
            case "green":
                light.setPosition(0.5);
                break;
            case "yellow":
                light.setPosition(0.388);
                break;
            case "orange":
                light.setPosition(0.33);
                break;
            case "red":
                light.setPosition(0.28);
                break;
        }
    }

    public void resetArmEncoder() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetRodeEncoder() {
        rodeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rodeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setDrivePower(double fL, double bL, double fR, double bR) {
        FL.setPower(fL);
        BL.setPower(bL);
        FR.setPower(fR);
        BR.setPower(bR);
    }

    public static double lerp(double start, double end, double t) {
        return start * (1 - t) + end * t;
    }

    public double[] chassisPower (double botHeading, double potenciaChassis, Gamepad gamepad1){
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        return new double[]{
                ((rotY + rotX + rx) / denominator) * potenciaChassis, //fl
                ((rotY - rotX + rx) / denominator) * potenciaChassis, // bl
                ((rotY - rotX - rx) / denominator) * potenciaChassis, //fr
                ((rotY + rotX - rx) / denominator) * potenciaChassis //br
        };

    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public Command contractArmDownCmd(){
        return new SequentialCommandGroup(
            wristAction.servoPosCMD(contractWristPos),
            new WaitCommand(300),
            armSb.rodeToPos(0)

        );
    }

    public Command extendArmHighBasketCmd(){
        return new SequentialCommandGroup(
                wristAction.servoPosCMD(basketWristPos),
                armSb.armToPos(BasketArmpos),
                new WaitCommand(300),
                armSb.rodeToPos(highRodePos)

        );
    }

    public Command contractArmBasketCmd(){
        return new SequentialCommandGroup(
                armSb.armToPos(BasketArmpos),
                new WaitCommand(400),

                armSb.rodeToPosSmooth(0, 0.5)
        );
    }

    public Command putSpecimenCmd(){
        return new SequentialCommandGroup(
                armSb.rodeToPos(specimenRodePos),
                new WaitCommand(200),

                armSb.armToPos(postSpecimenArmPos),
                new WaitCommand(200),

                clawAction.servoPosCMD(openClawPos),
                new WaitCommand(200),

                armSb.rodeToPosSmooth(0, 3),
                new WaitCommand(50),

                wristAction.servoPosCMD(downWristPos),
                new WaitCommand(300),

                armSb.armToPosSmooth(0, 0.5)
        );
    }

    public Command pickSpecimenCmd(){
        return new SequentialCommandGroup(
                clawAction.servoPosCMD(closeClawPos),
                new WaitCommand(300),

                armSb.armToPos(specimenArmPos),
                new WaitCommand(200),

                wristAction.servoPosCMD(specimenWristPos)
        );


    }


}