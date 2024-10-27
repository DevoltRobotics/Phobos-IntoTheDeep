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

    public double Pos_close = 0;
    public double Pos_open = 1;

    public double Down_doll = 0;
    public double Back_doll = 1;

    public boolean picked = false;

    public RevColorSensorV3 color;

    public IMU imu;

    public OpenCvCamera Camera;
    public VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    public Localizer localizer;

    private Etesito instance = null;

    public double Yellow_r = 0, Yellow_b = 0, Yellow_g = 0;

    public double Blue_r = 0, Blue_b = 0, Blue_g = 0;

    public double Red_r = 0, Rede_b = 0, Red_g = 0;



    public void init(HardwareMap hardwareMap) {

        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        C1 = hardwareMap.get(DcMotorEx.class, "mc2");
        C2 = hardwareMap.get(DcMotorEx.class, "mc1");


        servoC1 = hardwareMap.get(Servo.class,"sc1");
        servoC2 = hardwareMap.get(Servo.class,"sc2");

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        rodeMotor = hardwareMap.get(DcMotorEx.class, "rd");

        rodeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rodeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL = hardwareMap.get(DcMotorEx.class, "lf");
        BL = hardwareMap.get(DcMotorEx.class, "lb");
        BR = hardwareMap.get(DcMotorEx.class, "rb");
        FR = hardwareMap.get(DcMotorEx.class, "rf");

        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);

        claw = hardwareMap.servo.get("cw");

        wrist = hardwareMap.servo.get("wr");

    }

    public void pick(){

        claw.setPosition(Pos_close);
        picked = true;

    }

    public void open(){

        claw.setPosition(Pos_open);
        picked = false;

    }

    public void doll_down(){

        wrist.setPosition(Down_doll);

    }

    public void doll_up(){

        wrist.setPosition(Back_doll);

    }

}
