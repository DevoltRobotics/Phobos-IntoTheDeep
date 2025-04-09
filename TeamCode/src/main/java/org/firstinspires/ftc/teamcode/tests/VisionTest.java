package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.Comands.Constants.preSubWristPos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Comands.Etesito;
import org.firstinspires.ftc.teamcode.subsystems.Vision.CrosshairVision;
import org.opencv.core.RotatedRect;

@Autonomous
public class VisionTest extends OpMode {
    Etesito hdw = new Etesito();
    CrosshairVision vision;

    int targetX = 0;
    int targetY = 0;

    @Override
    public void init() {
        hdw.init(hardwareMap, false, false);
        vision = new CrosshairVision(hdw.webcam);
        vision.init();

        hdw.wrist.setPosition(preSubWristPos);
    }

    @Override
    public void loop() {
        hdw.wrist.setPosition(preSubWristPos);
        RotatedRect[] rects = vision.getLastRects();

        for(int i = 0 ; i < rects.length ; i++) {
            telemetry.addData("detection #" + i, rects[i].center);
        }


        RotatedRect rect = new RotatedRect();
        if (rects.length > 0) {
            rect = rects[0];

            targetX = (int)(rect.center.x);
            targetY = (int)rect.size.area();
        }

        telemetry.addData("x", targetX);
        telemetry.addData("size", targetY);




        vision.updateExposure();
    }
}

