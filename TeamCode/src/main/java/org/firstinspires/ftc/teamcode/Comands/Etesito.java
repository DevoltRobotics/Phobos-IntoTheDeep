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

import static org.firstinspires.ftc.teamcode.Comands.Constants.basketArmPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.armCoefficients;
import static org.firstinspires.ftc.teamcode.Comands.Constants.basketWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.chassisCoefficients;
import static org.firstinspires.ftc.teamcode.Comands.Constants.climbCoefficients;
import static org.firstinspires.ftc.teamcode.Comands.Constants.closeClawPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.contractWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.downWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.contractAbramPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.highRodePos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.initArmPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.initWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.launchArmsPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.midOpenAbramPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.mediumWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.openClawPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.pickSpecimenWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.postSpecimenArmPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.preSubWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.rodeCoefficients;
import static org.firstinspires.ftc.teamcode.Comands.Constants.servosClimbingPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.servosInitPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.servosTestPost;
import static org.firstinspires.ftc.teamcode.Comands.Constants.servosHangingPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.preSpecimenRodePos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.supportArmsPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.specimenArmPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.specimenRodePos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.specimenWristPos;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.ArmSb;
import org.firstinspires.ftc.teamcode.subsystems.BooleanSb;
import org.firstinspires.ftc.teamcode.subsystems.CRservoSb;
import org.firstinspires.ftc.teamcode.subsystems.RodeSb;
import org.firstinspires.ftc.teamcode.subsystems.ServoSb;

@Config
public class Etesito {

    public PIDFController armController = new PIDFController(armCoefficients);
    public PIDFController rodeController = new PIDFController(rodeCoefficients);
    public PIDFController chassiscontroller = new PIDFController(chassisCoefficients);
    PIDFController climbControllerRight = new PIDFController(climbCoefficients);
    PIDFController climbControllerLeft = new PIDFController(climbCoefficients);

    public DcMotorEx fl, bl, br, fr, armMotor, rodeMotor, mCR, mCL;

    public WebcamName webcam;

    public Servo sC1, sC2, claw, wrist, light, abraham, launcherLeft, launcherRight;

    public CRServo intake;

    public boolean wristIsMedium;

    public IMU imu;

    public CRservoSb intakeSb;

    public ServoSb clawSb;
    public ServoSb wristSb;
    public ServoSb abrahamSb;
    public ServoSb servosSb;
    public ServoSb launchersSb;
    public ServoSb lightSb;

    public ChassisSubsystem chassisSb;

    public ArmSb armSb;
    public RodeSb rodeSb;

    public BooleanSb booleanAction;

    public int armPosition = 0;

    public double headingAuto = 0;
    
    public void init(HardwareMap hardwareMap, boolean resetRode, boolean resetImu) {
        rodeController.reset();
        armController.reset();
        climbControllerRight.reset();
        climbControllerLeft.reset();
        chassiscontroller.reset();

        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

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

        launcherLeft = hardwareMap.get(Servo.class, "plL");
        launcherRight = hardwareMap.get(Servo.class, "plR");
        claw = hardwareMap.servo.get("cw");
        intake = hardwareMap.get(CRServo.class, "in");
        wrist = hardwareMap.servo.get("wr");
        abraham = hardwareMap.servo.get("sr");

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

        if (resetImu){
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
            imu.initialize(parameters);
            imu.resetYaw();
        }

        fl = hardwareMap.get(DcMotorEx.class, "fl"); //ex1
        bl = hardwareMap.get(DcMotorEx.class, "bl"); //ex2
        br = hardwareMap.get(DcMotorEx.class, "br"); //ex3
        fr = hardwareMap.get(DcMotorEx.class, "fr"); //ex4

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);

        wristIsMedium = false;

        intakeSb = new CRservoSb(intake);

        clawSb = new ServoSb(claw);
        wristSb = new ServoSb(wrist);
        abrahamSb = new ServoSb(abraham);
        servosSb = new ServoSb(sC1, sC2);
        launchersSb = new ServoSb(launcherRight, launcherLeft);

        lightSb = new ServoSb(light);

        chassisSb = new ChassisSubsystem(chassiscontroller, fl, bl, br, fr);

        armSb = new ArmSb(armMotor, armController);
        rodeSb = new RodeSb(rodeMotor, rodeController);

        booleanAction = new BooleanSb();

        CommandScheduler.getInstance().registerSubsystem(clawSb, wristSb, chassisSb, servosSb, launchersSb, intakeSb,
                                                         armSb, rodeSb, booleanAction);
    }

    public void launchHangArms() {
        launcherRight.setPosition(0.5 + launchArmsPos);
        launcherLeft.setPosition(0.5 - launchArmsPos);
    }

    public void supportHangArms() {
        launcherRight.setPosition(0.5 + supportArmsPos);
        launcherLeft.setPosition(0.5  - supportArmsPos);
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

    public void wristSpecimen() {
        wrist.setPosition(specimenWristPos);
        wristIsMedium = true;
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
        abraham.setPosition(midOpenAbramPos);
    }

    public void esconderPalito() {
        abraham.setPosition(contractAbramPos);
    }

    public void servosOff() {
        sC1.getController().pwmDisable();
        sC2.getController().pwmDisable();
    }

    public void servosHanging() {

        sC1.setPosition(0.5 + servosHangingPos);
        sC2.setPosition(0.5 - servosHangingPos);
    }

    public void servosClimbing() {

        sC1.setPosition(0.5 + servosClimbingPos);
        sC2.setPosition(0.5 - servosClimbingPos);
    }

    public void servos_test() {

        sC1.setPosition(0.5 + servosTestPost);
        sC2.setPosition(0.5 - servosTestPost);

    }

    public void servosDownNomral() {
        sC1.setPosition(0);
        sC2.setPosition(1);


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
        fl.setPower(fL);
        bl.setPower(bL);
        fr.setPower(fR);
        br.setPower(bR);
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
            wristSb.servoPosCMD(contractWristPos),
            booleanAction.booleanCmd(wristIsMedium, true),
            new WaitCommand(300),
                rodeSb.rodeToPos(0)

        );
    }

    public Command extendArmHighBasketCmd(){
        return new SequentialCommandGroup(
                wristSb.servoPosCMD(basketWristPos),
                armSb.armToPos(basketArmPos),
                new WaitCommand(350),
                rodeSb.rodeToPos(highRodePos),
                new WaitCommand(200)

        );
    }

    public Command contractArmBasketCmd(){
        return new SequentialCommandGroup(
                wristSb.servoPosCMD(downWristPos),
                new WaitCommand(200),

                intakeSb.crservoCMD(0),
                rodeSb.rodeToPosSmooth(0, 0.5)

        );
    }

    public Command subContractArmBasketCmd(){
        return new SequentialCommandGroup(
                wristSb.servoPosCMD(preSubWristPos),
                new WaitCommand(200),

                intakeSb.crservoCMD(0),
                rodeSb.rodeToPos(-500),

                new WaitCommand(200)

        );
    }


    /*public Command putSpecimenSeqCmd(){
        return new SequentialCommandGroup(
                rodeSb.rodeToPos(specimenRodePos),
                new WaitCommand(200),

                armSb.armToPos(postSpecimenArmPos),
                new WaitCommand(200),

                clawSb.servoPosCMD(openClawPos),
                new WaitCommand(200),

                rodeSb.rodeToPosSmooth(0, 3),
                new WaitCommand(50),

                wristSb.servoPosCMD(downWristPos),
                booleanAction.booleanCmd(wristIsMedium, false),
                new WaitCommand(300),

                armSb.armToPosSmooth(0, 0.5),
                booleanAction.numberCmd(armPosition, 0)
                );
    }*/

    public Command putSpecimenOneSeqCmd(){
        return new SequentialCommandGroup(
                rodeSb.rodeToPos(specimenRodePos),
                new WaitCommand(150),

                armSb.armToPos(postSpecimenArmPos),
                wristSb.servoPosCMD(downWristPos),
                new WaitCommand(150),

                clawSb.servoPosCMD(openClawPos),
                new WaitCommand(300),

                rodeSb.rodeToPos(0),
                new WaitCommand(100)
        );
    }

    public Command pickSpecimenOneSeqCmd(){
        return new SequentialCommandGroup(
                wristSb.servoPosCMD(pickSpecimenWristPos),

                rodeSb.rodeToPos(-350),
                new WaitCommand(300),

                clawSb.servoPosCMD(closeClawPos),
                new WaitCommand(400),

                armSb.armToPos(specimenArmPos),
                new WaitCommand(300)

                );
    }

    public Command pickSpecimenOne2SeqCmd(){
        return new SequentialCommandGroup(
                rodeSb.rodeToPos(-150),
                new WaitCommand(300),

                clawSb.servoPosCMD(closeClawPos),
                new WaitCommand(300),

                armSb.armToPos(specimenArmPos),
                new WaitCommand(300)

        );
    }

    public Command pickSpecimenTwoSeqCmd(){
        return new SequentialCommandGroup(
                wristSb.servoPosCMD(specimenWristPos),
                rodeSb.rodeToPos(preSpecimenRodePos)

        );
    }

    public Command launchArms(){
        return new SequentialCommandGroup(
                launchersSb.mirrorServoPosCMD(launchArmsPos),
                new WaitCommand(500),
                servosSb.mirrorServoPosCMD(servosHangingPos)
        );
    }

    public Command initCmd(){
        return new SequentialCommandGroup(
                clawSb.servoPosCMD(closeClawPos),
                wristSb.servoPosCMD(basketWristPos),
                servosSb.mirrorServoPosCMD(servosInitPos),
                launchersSb.mirrorServoPosCMD(supportArmsPos),

                new WaitCommand(300),

                armSb.armToPos(initArmPos),

                new WaitCommand(800),

                wristSb.servoPosCMD(initWristPos)
                );
    }

    public Command downArm3rdSample(){
        return new SequentialCommandGroup(
                abrahamSb.servoPosCMD(contractAbramPos),
                wristSb.servoPosCMD(downWristPos),
                new WaitCommand(200),

                armSb.armToPosSmooth(0, 0.4),

                intakeSb.crservoCMD(1),
                rodeSb.rodeToPos(-400)
        );
    }

}