package org.firstinspires.ftc.teamcode.tests;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Comands.Etesito;
import org.firstinspires.ftc.teamcode.Comands.PIDFController;

import java.util.List;

@Config
public class TrackSampleSubsystemCOPY {

    Etesito etesito = new Etesito();
    double Power;

    public PIDFController chassisController = new PIDFController(Etesito.chassisCoefficients);

    public void init(HardwareMap hardwareMap){
        etesito.init(hardwareMap);
        chassisController.reset();


    }

    public void trackearSample(double target){

            ElapsedTime timer = new ElapsedTime();

            chassisController.targetPosition = 0;

            chassisController.targetPosition =
                    (chassisController.targetPosition) - chassisController.update(target) * timer.seconds();

            Power = chassisController.update(target) * 0.8;

            etesito.setDrivePower(Power, Power, -Power, -Power);

            timer.reset();


    }

}






