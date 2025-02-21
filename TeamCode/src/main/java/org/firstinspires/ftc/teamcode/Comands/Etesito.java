package org.firstinspires.ftc.teamcode.Comands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Etesito {

    public PIDFController.PIDCoefficients armCoefficients = new PIDFController.PIDCoefficients(0.0015, 0, 0.0017);
    public PIDFController.PIDCoefficients rodeCoefficients = new PIDFController.PIDCoefficients(0.015, 0, 0.007);

    public PIDFController.PIDCoefficients climbCoefficients = new PIDFController.PIDCoefficients(0.015, 0, 0.0017);

    public static PIDFController.PIDCoefficients chassisCoefficients = new PIDFController.PIDCoefficients(0.06, 0, 0.035);

    PIDFController armController = new PIDFController(armCoefficients);
    PIDFController rodeController = new PIDFController(rodeCoefficients);
    PIDFController climbControllerRight = new PIDFController(climbCoefficients);
    PIDFController climbControllerLeft = new PIDFController(climbCoefficients);

    public DcMotorEx FL, BL, BR, FR;

    public DcMotorEx armMotor;
    public DcMotorEx rodeMotor;
    public DcMotorEx cR;
    public DcMotorEx cL;
    public Servo sC1;
    public Servo sC2;

    public Servo claw;

    public CRServo intake;

    public Servo wrist;

    public Servo light;

    public RevColorSensorV3 color;

    public RevTouchSensor touch;

    public Limelight3A limelight;

    public boolean wristIsMedium;

    public int ratioArm = 8;

    public double Pos_close = 1;
    public double Pos_open = 0.05;

    public double initWristPos = 0.8;

    public double downWristPos = 0.6;

    public double pickSpecimenWristPos = 0.63;
    public double downMWristPos = 0.53;
    public double contractWristPos = 0.37;
    public double mediumWristPos = 0.3;
    public double upWristPos = 0.2;
    public double specimenWristPos = 0.3;

    public int downArmPos = 0;
    public int initArmpos = -950;
    public int lowBasketArmpos = -1200;
    public int specimenArmPos = -1900;
    public int highBasketArmpos = -1900;

    public int highRodePos = -2050;
    public int specimenRodePos = -1600;
    public int lowBasketRodePos = -1400;
    public int downRodePos = -1200;
    public int climbingRodePos2 = -500;
    public int climbingRodePos1 = -200;
    public int specimenRodePosPrueba = -700;
    public int specimenDownRodePos = -780;

    public double servosUpingPos = 0.2;
    public double servosClimbingPos = 0.1;
    public double servosInitPos = -0.05;

    public int red, green, blue;

    public int scaleFactor = 2;

    public double colorTarget = 0;

    public double redTarget = 400;
    public double greenTarget = 700;
    public double blueTarget = 450;

    public IMU imu;

    public void init(HardwareMap hardwareMap) {

        //limelight.setPollRateHz(150);
        //limelight.pipelineSwitch(1);

        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rodeMotor = hardwareMap.get(DcMotorEx.class, "rd");

        rodeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rodeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        claw = hardwareMap.servo.get("cw");
        intake = hardwareMap.get(CRServo.class, "in");
        wrist = hardwareMap.servo.get("wr");

        light = hardwareMap.get(Servo.class, "rgb");

        color = hardwareMap.get(RevColorSensorV3.class, "color");

        touch = hardwareMap.get(RevTouchSensor.class, "tch");

        cR = hardwareMap.get(DcMotorEx.class, "mc1");
        cL = hardwareMap.get(DcMotorEx.class, "mc2");
        cR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        cR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        cL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        cL.setDirection(DcMotorSimple.Direction.REVERSE);

        sC1 = hardwareMap.get(Servo.class,"sc1");
        sC2 = hardwareMap.get(Servo.class,"sc2");

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
    }

    public void pickSpecimen(){
        claw.setPosition(Pos_close);
    }

    public void dropSpecimen(){
        claw.setPosition(Pos_open);
    }

    public void pickSample(){
        intake.setPower(-1);
    }

    public void pickSampleSlow(){
        intake.setPower(-0.1);
    }

    public void dropSample(){
        intake.setPower(1);
    }

    public void intake0(){
        intake.setPower(0);
    }

    public void wrist_down(){
        wrist.setPosition(downWristPos);
        wristIsMedium = false;
    }

    public void wrist_Contract(){
        wrist.setPosition(contractWristPos);
        wristIsMedium = true;
    }

    public void wrist_Medium(){
        wrist.setPosition(mediumWristPos);
        wristIsMedium = true;
    }


    public void wrist_up(){
        wrist.setPosition(upWristPos);
        wristIsMedium = false;
    }

    public void wrist_Specimen(){
        wrist.setPosition(specimenWristPos);
        wristIsMedium = false;
    }

    public void wrist_Init(){
        wrist.setPosition(initWristPos);
        wristIsMedium = false;
    }

    public void wrist_ManualUp(){
        wrist.setPosition(wrist.getPosition() + 0.03);
    }

    public void wrist_ManualDown(){
        wrist.setPosition(wrist.getPosition() - 0.03);
    }

    public void wrist_ManualMantener(){
        wrist.setPosition(wrist.getPosition());
    }

    public void servosOff(){
        sC1.getController().pwmDisable();
        sC2.getController().pwmDisable();
    }

    public void servos_Uping(){

        sC1.setPosition(0.5 + servosUpingPos);
        sC2.setPosition(0.5 - servosUpingPos);
    }

    public void servos_Climbing(){

        sC1.setPosition(0.5 + servosClimbingPos);
        sC2.setPosition(0.5 - servosClimbingPos);
    }

    public void servos_down() {
        ElapsedTime timer = new ElapsedTime();

        sC1.setPosition(0);
        sC2.setPosition(1);

        if (timer.seconds() > 0.2){
            servosOff();

        }
    }

    public void setLight(String color){
        switch (color){
            case "purple": light.setPosition(0.72);
                break;
            case "blue": light.setPosition(0.62);
                break;
            case "green": light.setPosition(0.5);
                break;
            case "yellow": light.setPosition(0.388);
                break;
            case "orange": light.setPosition(0.33);
                break;
            case "red": light.setPosition(0.28);
                break;
        }
    }

    public int getColorRed(){
        red = (color.red() / scaleFactor);
        return red;
    }
    public int getColorGreen(){
        green = (color.green() / scaleFactor);
        return green;
    }
    public int getColorBlue(){
        blue = (color.blue() / scaleFactor);
        return blue;
    }
    public boolean isSampleDetected(String color){

        boolean isDetected = false;
        switch (color){
            case "red": isDetected = getColorRed() > redTarget;
                break;
            case "blue": isDetected = getColorBlue() > blueTarget;
                break;
        }

        return isDetected;
    }

    public void colorTelemetry(Telemetry telemetry){
        telemetry.addData("red", getColorRed());
        telemetry.addData("green", getColorGreen());
        telemetry.addData("blue", getColorBlue());
    }

    public Action pickSampleAction (){
        return new CRservoAction(intake, -1);
    }

    public Action pickSampleSlowAction (){
        return new CRservoAction(intake, -0.5);
    }

    public Action dropSampleAction (){
        return new CRservoAction(intake, 1);
    }

    public Action mantenerSampleAction (){
        return new CRservoAction(intake, 0);

    }

    public Action servosInit(){
        return new ParallelAction(
                new ServoAction(sC1, 0.5 + servosInitPos),
                new ServoAction(sC2, 0.5 - servosInitPos)
        );
    }

    public Action servosClimbing(){
        return new ParallelAction(
                new ServoAction(sC1, 0.5 + servosUpingPos),
                new ServoAction(sC2, 0.5 - servosUpingPos)
        );
    }

    public Action pickSpecimenAction(){
        return new ServoAction(claw, Pos_close);
    }

    public Action dropSpecimenAction(){
        return new ServoAction(claw, Pos_open);
    }

    public Action wristInit(){
        return new ServoAction(wrist, initWristPos);
    }

    public Action wristDownM(){
        return new ServoAction(wrist, downMWristPos);
    }

    public Action wristDown(){
        return new ServoAction(wrist, downWristPos);
    }

    public Action pickSpecimenWrist(){
        return new ServoAction(wrist, pickSpecimenWristPos);
    }


    /*public Action wristDownM(){
        return new ServoAction(wrist, Down_M_wrist);

    }*/

    public Action wristContract(){
        return new ServoAction(wrist, contractWristPos);
    }

    public Action wristUp(){
        return new ServoAction(wrist, upWristPos);
    }

    public Action wristSpecimen(){
        return new ServoAction(wrist, specimenWristPos);
    }

    public void resetArmEncoder(){
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetRodeEncoder(){
        rodeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rodeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setDrivePower(double fL, double bL, double fR, double bR){
        FL.setPower(fL);
        BL.setPower(bL);
        FR.setPower(fR);
        BR.setPower(bR);
    }

    public double lerp(double start, double end, double t) {
        return start * (1 - t) + end * t;
    }

    public double ArmToPosSmooth(double timeSeconds, int currentTicks,  int ticks, ElapsedTime timer){
        double ArmTarget;

        double t = Range.clip(timer.seconds() / timeSeconds, 0, 1);

        ArmTarget = (int) lerp(currentTicks, ticks, t);

            /*if (timer.seconds() > timeSeconds){
                timer.reset();
            }*/
        return ArmTarget;
    }
}