package org.firstinspires.ftc.teamcode.Comands;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Etesito {

    public PIDFController.PIDCoefficients armCoefficients = new PIDFController.PIDCoefficients(0.0015, 0, 0.0017);
    public PIDFController.PIDCoefficients rodeCoefficients = new PIDFController.PIDCoefficients(0.015, 0, 0.007);

    public PIDFController.PIDCoefficients climbCoefficients = new PIDFController.PIDCoefficients(0.015, 0, 0.0017);

    PIDFController armController = new PIDFController(armCoefficients);
    PIDFController rodeController = new PIDFController(rodeCoefficients);
    PIDFController climbControllerRight = new PIDFController(climbCoefficients);
    PIDFController climbControllerLeft = new PIDFController(climbCoefficients);

    public DcMotorEx FL, BL, BR, FR;

    public DcMotorEx armMotor;
    public DcMotorEx rodeMotor;
    public DcMotorEx cR;
    public DcMotorEx cL;

    public Servo servoC1;
    public Servo servoC2;

    public Servo claw;

    public CRServo intake;

    public Servo wrist;

    public Servo light;

    public RevColorSensorV3 color;

    public boolean wristIsMedium;

    private double ArmTarget = 0;
    private double RodeTarget = 0;

    public int ratio = 8;

    private double powerArm = 0;

    public double Pos_close = 1;
    public double Pos_open = 0.05;

    public double Init_wrist = 0.8;
    public double Down_wrist = 0.6;
    public double Down_M_wrist = 0.48;
    public double Contract_wrist = 0.37;
    public double Medium_wrist = 0.3;
    public double Up_wrist = 0.2;
    public double Specimen_wristTeleop = 0.3;
    public double Climb_Wrist = 0.5;

    public int downArmPos = 0;
    public int initArmpos = -950;
    public int lowBasketArmpos = -1200;
    public int specimenArmPos = -1900;
    public int highBasketfrontArmpos = -1550;
    public int highBasketArmpos = -1900;

    public int highRodePos = -2050;
    public int specimenRodePos = -1600;
    public int lowBasketRodePos = -1400;
    public int downRodePos = -1200;
    public int climbingRodePos2 = -500;
    public int climbingRodePos1 = -200;
    public int specimenDownRodePos = -850;

    public int red, green, blue;

    public int scaleFactor = 10;

    public double redTarget = 400;
    public double greenTarget = 700;
    public double blueTarget = 400;

    public IMU imu;

    public void init(HardwareMap hardwareMap) {

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

        cR = hardwareMap.get(DcMotorEx.class, "mc1");
        cL = hardwareMap.get(DcMotorEx.class, "mc2");
        cR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        cR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        cL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        cL.setDirection(DcMotorSimple.Direction.REVERSE);

        servoC1 = hardwareMap.get(Servo.class,"sc1");
        servoC2 = hardwareMap.get(Servo.class,"sc2");

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        FL = hardwareMap.get(DcMotorEx.class, "fl");
        BL = hardwareMap.get(DcMotorEx.class, "bl");
        BR = hardwareMap.get(DcMotorEx.class, "br");
        FR = hardwareMap.get(DcMotorEx.class, "fr");

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);

        wristIsMedium = true;

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
        wrist.setPosition(Down_wrist);
        wristIsMedium = false;

    }

    public void wrist_downM(){
        wrist.setPosition(Down_M_wrist);
        wristIsMedium = false;

    }

    /*public void wrist_Down_M() {
        wrist.setPosition(Down_M_wrist);
        wristIsMedium = false;

    }*/

    public void wrist_Contract(){
        wrist.setPosition(Contract_wrist);

        wristIsMedium = true;

    }

    public void wrist_Medium(){
        wrist.setPosition(Medium_wrist);

        wristIsMedium = true;

    }


    public void wrist_up(){
        wrist.setPosition(Up_wrist);

        wristIsMedium = false;

    }

    public void wrist_Specimen(){
        wrist.setPosition(Specimen_wristTeleop);
        wristIsMedium = false;


    }

    public void wrist_Init(){
        wrist.setPosition(Init_wrist);

        wristIsMedium = false;

    }

    public void wrist_Climb(){
        wrist.setPosition(Climb_Wrist);
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

        servoC1.getController().pwmDisable();
        servoC2.getController().pwmDisable();
    }

    public void servos_Uping(){

        servoC1.setPosition(0.7);
        servoC2.setPosition(0.3);
    }

    public void servos_Climbing(){

        servoC1.setPosition(0.6);
        servoC2.setPosition(0.4);
    }

    public void servos_down(){

        ElapsedTime timer = new ElapsedTime();

        servoC1.setPosition(0);
        servoC2.setPosition(1);

        timer.reset();

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
            case "yellos": light.setPosition(0.388);
            break;
            case "orange": light.setPosition(0.33);
            break;
            case "red": light.setPosition(0.28);
            break;

        }

    }

    public void orangeLight(){

        light.setPosition(0.35);

    }

    public void greenLight(){

        light.setPosition(0.5);

    }

    public void blueLight(){

        light.setPosition(0.66);

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
            case "any": isDetected = getColorRed() > redTarget || getColorBlue() > blueTarget || getColorGreen() > greenTarget;
            case "red": isDetected = getColorRed() > redTarget;
            break;
            case "blue": isDetected = getColorBlue() > blueTarget;
            break;
            case "yellow": isDetected = getColorGreen() > greenTarget;
            break;
        }

        return isDetected;

    }

    public void colorTelemetry(Telemetry telemetry){
        telemetry.addData("red", getColorRed());
        telemetry.addData("green", getColorGreen());
        telemetry.addData("blue", getColorBlue());
        telemetry.update();
    }



    public Action pickSampleAction (){
        return new CRservoAction(intake, -1);

    }

    public Action pickSampleSlowAction (){
        return new CRservoAction(intake, -0.1);

    }

    public Action dropSampleAction (){
        return new CRservoAction(intake, 1);

    }

    public Action mantenerSampleAction (){
        return new CRservoAction(intake, 0);

    }

    public Action servosInit(){
        return new ParallelAction(
                new ServoAction(servoC1, 0.5),
                new ServoAction(servoC2, 0.5)

                );
    }

    public Action servosClimbing(){
        return new ParallelAction(
                new ServoAction(servoC1, 0.7),
                new ServoAction(servoC2, 0.3)

        );
    }

    public Action pickSpecimenAction(){
        return new ServoAction(claw, Pos_close);

    }

    public Action dropSpecimenAction(){
        return new ServoAction(claw, Pos_open);

    }

    public Action wristInit(){
        return new ServoAction(wrist, Init_wrist);

    }

    public Action wristDown(){
        return new ServoAction(wrist, Down_wrist);

    }

    /*public Action wristDownM(){
        return new ServoAction(wrist, Down_M_wrist);

    }*/

    public Action wristContract(){
        return new ServoAction(wrist, Contract_wrist);

    }

    public Action wristUp(){
        return new ServoAction(wrist, Up_wrist);

    }

    public Action wristSpecimen(){
        return new ServoAction(wrist, Specimen_wristTeleop);

    }



}
