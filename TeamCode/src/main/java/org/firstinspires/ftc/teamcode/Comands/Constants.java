package org.firstinspires.ftc.teamcode.Comands;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

@Config
public class Constants {
    public static PIDFController.PIDCoefficients armCoefficients = new PIDFController.PIDCoefficients(0.00185, 0, 0.017);
    public static PIDFController.PIDCoefficients rodeCoefficients = new PIDFController.PIDCoefficients(0.07, 0, 0.007);

    public static PIDFController.PIDCoefficients climbCoefficients = new PIDFController.PIDCoefficients(0.015, 0, 0.0017);

    public static PIDFController.PIDCoefficients chassisCoefficients = new PIDFController.PIDCoefficients(0.14, 0, 0.02);

    public static double ScaleFactor = 80;

    public static int xPixels = 320;
    public static int xFov = 25;
    public static double xDegreesPerPixel = (double) (xPixels) / (xFov);


    public static int yPixels = 240;
    public static int yFov = 10;

    public static double hCamera = 5.1;
    public static double intakeAngle = 30.0; // en grados;

    public static double intakeAngleRode = 25.0; // en grados;

    public static int correction = 100; // en grados;

    public static double getTargetExtensionFromY(double tY, double currentRodeTicks){
        double yDegreesPerPixel = (double) (yFov) / (yPixels);
        double offsetFromCenter  = (yPixels) - tY;
        double viewAngle  = offsetFromCenter  * yDegreesPerPixel;

        double totalAngleDeg = viewAngle + ((viewAngle / yFov) * intakeAngleRode);
        double totalAngleRad = Math.toRadians(totalAngleDeg);

        double extensionInches = hCamera * Math.tan(totalAngleRad);

        double rodeTarget = (currentRodeTicks - (extensionInches * rodeInToTicks) - correction);

        int ticks = Range.clip((int)(rodeTarget), extensionLimit, preSubmRodePos);

        return ticks;
    }

    public static double getTargetAngleY(double tY){
        double yDegreesPerPixel = (double) (yFov) / (yPixels);
        double offsetFromCenter  = (yPixels) - tY;
        double viewAngle  = offsetFromCenter  * yDegreesPerPixel;

        return viewAngle;
    }

    public static final int ratioArm = 20;

    public static double openAbramPos = 0.05;
    public static double midOpenAbramPos = 0.25;
    public static double contractAbramPos = 0.5;

    public static double closeClawPos = 1;
    public static double openClawPos = 0;

    public static double preSubWristPos = 1;
    public static double initWristPos = 0.8;
    public static double SubWristPos = 1;
    public static double pickSubWristPos = 0.83;
    public static double downWristPos = 0.6;
    public static double pickSpecimenWristPos = 0.64;
    public static double specimenWristPos = 0.43;
    public static double contractWristPos = 0.27;
    public static double mediumWristPos = 0.3;
    public static double firstSpecimenWristPos = 0.47;
    public static double postSpecimenWristPos = 0.15;
    public static double basketWristPos = 0;

    public static int submArmPos = -300;
    public static int initArmPos = -950;
    public static int firstSpecimenArmPos = -860;
    public static int parkArmPos = -1100;
    public static int specimenArmPos = -2100;
    public static int postSpecimenArmPos = -1200;
    public static int basketArmPos = -1950;
    public static int lowBasketArmPos = -1950;


//435 RPM 384 res
    /*public static int highRodePos = -1300; //
    public static int highRodePosAuto = -1250; //
    public static int preSubmRodePos = -500; //mn holqa mundo
    public static int postSubmRodePos = -1050; //
    public static int firstSpecimenRodePos = -1100;
    public static int specimenRodePos = -700;
    public static int preSpecimenRodePos = -370;
    public static int preSpecimenTeleOpRodePos = -300;
    public static int downRodePos = -750; //z
    public static int extensionLimit = -1000; //
    public static int climbingRodePos2 = -390;
    public static int climbingRodePos1 = -200;
    public static int parkRodePos = -600;*/

    //312 RPM 537 res

    public static int rodeExtended = -550; //
    public static int pickSpecimenAutoRodePos = -350; //
    public static int highRodePos = -1820; //
    public static int highRodePosAuto = -1758; //
    public static int preSubmRodePos = -700; //
    public static int postSubmRodePos = -1470; //
    public static int firstSpecimenRodePos = -1570;
    public static int specimenRodePos = -980;
    public static int preSpecimenRodePos = -450;
    public static int preSpecimenTeleOpRodePos = -430;
    public static int downRodePos = -1050; //z
    public static int extensionLimit = -1350;
    //
    public static int climbingRodePos1 = -280;
    public static int parkRodePos = -840;

    public static double servosHangingPos = 0.2;
    public static double servosClimbingPos = 0.15;
    public static double servosInitPos = -0.05;
    public static double servosTestPost = 0;

    public static double launchArmsPos = 0.5;
    public static double supportArmsPos = -0.4;

    public static double rodeTick = 384.5;
    public static double rodeCircIn = 3.71;
    public static double rodeTicksToIn = rodeCircIn / rodeTick;
    public static double rodeInToTicks = rodeTick / rodeCircIn;

}



