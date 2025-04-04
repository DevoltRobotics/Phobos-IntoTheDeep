package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Comands.Etesito;
import org.firstinspires.ftc.teamcode.subsystems.Vision.CrosshairVision;
import org.opencv.core.RotatedRect;

@Autonomous
public class VisionTest extends OpMode {
    Etesito hdw = new Etesito();
    CrosshairVision vision;

    @Override
    public void init() {
        hdw.init(hardwareMap, false, false);
        vision = new CrosshairVision(hdw.webcam);
        vision.init();

        hdw.wrist.getController().pwmDisable();
    }

    @Override
    public void loop() {
        hdw.wrist.getController().pwmDisable();

        RotatedRect[] rects = vision.getLastRects();

        for(int i = 0 ; i < rects.length ; i++) {
            telemetry.addData("detection #" + i, rects[i].center);
        }

        vision.updateExposure();
    }
}

