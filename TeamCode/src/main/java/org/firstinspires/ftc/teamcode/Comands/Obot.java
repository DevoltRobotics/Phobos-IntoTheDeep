package org.firstinspires.ftc.teamcode.Comands;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RR.Localizer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;

public class Obot {

    public static DcMotorEx FL, BL, BR, FR;

    public static DcMotorEx armMotor;
    public static DcMotorEx rodeMotor;
    public static DcMotorEx C1;
    public static DcMotorEx C2;

    public static Servo claw1;
    public static Servo claw2;
    public static Servo doll;

    public static double Pos_close = 0;
    public static double Pos_open = 1;

    public static IMU imu;

    public static RevTouchSensor magnetic;

    public static OpenCvCamera Camera;
    public VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    public Localizer localizer;

    private static Obot instance = null;

    public boolean enabled;

    private HardwareMap hardwareMap;

    public static double arm_pos = armMotor.getCurrentPosition();
    public static double extension = rodeMotor.getCurrentPosition();

    public static Obot getInstance() {
        if (instance == null) {
            instance = new Obot();
        }
        instance.enabled = true;
        return instance;
    }

    public static void init(HardwareMap hardwareMap) {

        FL = hardwareMap.get(DcMotorEx.class, "lf");
        BL = hardwareMap.get(DcMotorEx.class, "lb");
        BR = hardwareMap.get(DcMotorEx.class, "rb");
        FR = hardwareMap.get(DcMotorEx.class, "rf");

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rodeMotor = hardwareMap.get(DcMotorEx.class, "rode");
        rodeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        C1 = hardwareMap.get(DcMotorEx.class, "c1");
        C2 = hardwareMap.get(DcMotorEx.class, "c2");

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rodeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        C1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        C2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        magnetic = hardwareMap.get(RevTouchSensor.class, "mag");

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

    }

    public static void ARM_RWE(){

        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public static void ARM_SRE(){

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public static boolean tope(){

        return magnetic.isPressed();
    }

    public static void pick(){

        claw1.setPosition(Pos_close);
        claw2.setPosition(Pos_close);

    }

    public static void open(){

        claw1.setPosition(Pos_open);
        claw2.setPosition(Pos_open);

    }

}
