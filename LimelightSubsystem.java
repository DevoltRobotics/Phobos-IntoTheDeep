package org.firstinspires.ftc.teamcode.Comands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LimelightSubsystem {
    private Limelight3A limelight;

    public LimelightSubsystem (HardwareMap hardwareMap, Telemetry telemetry){

        limelight = hardwareMap.get(Limelight3A.class, "limelight");


        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.
         */
        limelight.start();
    }

    class FindTarget implements Action {

        public FindTarget(int ticks) {
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            LLResult result = limelight.getLatestResult();

            return true;
        }

    }


}
