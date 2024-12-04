package org.firstinspires.ftc.teamcode.Comands;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RR.Localizer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;

public class Etesito {

    public DcMotorEx FL, BL, BR, FR;

    public DcMotorEx armMotor;
    public DcMotorEx rodeMotor;
    public DcMotorEx cR;
    public DcMotorEx cL;

    public Servo servoC1;
    public Servo servoC2;

    public Servo claw;

    public Servo wrist;

    public RevColorSensorV3 color;

    public double Pos_close = 0.6;
    public double Pos_open = 1;

    public double Init_wrist = 0.8;
    public double Down_wrist = 0.6;
    public double Down_M_wrist = 0.5;
    public double Medium_wrist = 0.3;
    public double Up_wrist = 0.2;
    public double Climb_Wrist = 0;

    public double down_ArmPos = 0;
    public double lowBasket_Armpos = -1200;
    public double specimen_ArmPos = -1700;
    public double high_Armpos = -1900;
    public double rode_High = -2100;
    public double rode_specimen = -1500;
    public double rode_medium = -1400;
    public double rode_down = -1200;

    float hsv[] = {0.0f, 0.0f, 0.0f};
    float SCALE_FACTOR = 255;

    public int ratio = 8;

    public float red, green, blue;

    public double scaleFactor = 10;

    public double redTarget = 200;
    public double greenTarget = 100;
    public double blueTarget = 100;
    public double TargetValue = 1000;

    public IMU imu;

    public OpenCvCamera Camera;
    public VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    public Localizer localizer;

    public void init(HardwareMap hardwareMap) {

        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rodeMotor = hardwareMap.get(DcMotorEx.class, "rd");

        rodeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rodeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        claw = hardwareMap.servo.get("cw");
        wrist = hardwareMap.servo.get("wr");

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
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        FL = hardwareMap.get(DcMotorEx.class, "fl");
        BL = hardwareMap.get(DcMotorEx.class, "bl");
        BR = hardwareMap.get(DcMotorEx.class, "br");
        FR = hardwareMap.get(DcMotorEx.class, "fr");

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);

    }


    public void pick(){
        claw.setPosition(Pos_close);


    }

    public void open(){
        claw.setPosition(Pos_open);


    }

    public void wrist_down(){
        wrist.setPosition(Down_wrist);

    }

    public void wrist_Down_M(){
        wrist.setPosition(Down_M_wrist);

    }

    public void wrist_Medium(){
        wrist.setPosition(Medium_wrist);

    }


    public void wrist_up(){
        wrist.setPosition(Up_wrist);

    }

    public void wrist_Climb(){
        wrist.setPosition(Climb_Wrist);

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

    public void getColors(){

        red = color.red();
        green = color.green();
        blue = color.blue();

        Color.RGBToHSV((int) (red * SCALE_FACTOR), (int) (green * SCALE_FACTOR), (int) (blue * SCALE_FACTOR), hsv);

    }

    public void colorTelemetry(Telemetry telemetry){
        telemetry.addData("red", red);
        telemetry.addData("green", green);
        telemetry.addData("blue", blue);
        telemetry.addData("Hue", hsv[0]);

    }




}
