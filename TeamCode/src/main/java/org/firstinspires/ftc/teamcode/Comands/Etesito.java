package org.firstinspires.ftc.teamcode.Comands;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RR.Localizer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;

public class Etesito {

    public DcMotorEx FL, BL, BR, FR;

    public DcMotorEx armMotor;
    public DcMotorEx rodeMotor;
    public DcMotorEx C1;
    public DcMotorEx C2;

    public Servo servoC1;
    public Servo servoC2;

    public Servo claw;

    public Servo wrist;

    public double Pos_close = 1;
    public double Pos_open = 0.8;

    public double Down_wrist = 0.6;
    public double Medium_wrist = 0.5;
    public double Up_wrist = 0.35;

    public double down_ArmPos = 0;
    public double medium_Armpos = -1300;
    public double high_Armpos = -1900;

    public double rode_High = -2100;
    public double rode_medium = -1000;

    public int ratio = 8;


    public RevColorSensorV3 color;

    public IMU imu;

    public OpenCvCamera Camera;
    public VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    public Localizer localizer;

    private Etesito instance = null;

    public void init(HardwareMap hardwareMap) {


        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        C1 = hardwareMap.get(DcMotorEx.class, "mc2");
        C2 = hardwareMap.get(DcMotorEx.class, "mc1");


        servoC1 = hardwareMap.get(Servo.class,"sc1");
        servoC2 = hardwareMap.get(Servo.class,"sc2");

        imu = hardwareMap.get(IMU.class, "imu");

        color = hardwareMap.get(RevColorSensorV3.class, "color");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        rodeMotor = hardwareMap.get(DcMotorEx.class, "rd");

        rodeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rodeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL = hardwareMap.get(DcMotorEx.class, "fl");
        BL = hardwareMap.get(DcMotorEx.class, "bl");
        BR = hardwareMap.get(DcMotorEx.class, "br");
        FR = hardwareMap.get(DcMotorEx.class, "fr");

        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);

        claw = hardwareMap.servo.get("cw");

        wrist = hardwareMap.servo.get("wr");

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

    public void wrist_medium(){
        wrist.setPosition(Medium_wrist);

    }

    public void wrist_up(){
        wrist.setPosition(Up_wrist);

    }

}
