package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.Comands.Constants.getTargetAngleY;
import static org.firstinspires.ftc.teamcode.Comands.Constants.preSubWristPos;
import static org.firstinspires.ftc.teamcode.Comands.Constants.xDegreesPerPixel;
import static org.firstinspires.ftc.teamcode.Comands.Constants.xFov;

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
        RotatedRect rect = vision.getRect();

        if(rect != null) {
            telemetry.addData("detection", rect);
        }

        if (rect != null) {
            targetX = (int)(rect.center.x);
            targetY = (int)rect.size.area();
        }

        double targetYAngl = getTargetAngleY(targetY);

        double pixelErrorFromCenterX = targetX - 160;
        double targetXAngl = pixelErrorFromCenterX / xDegreesPerPixel;

        double tAngl = targetXAngl - ((targetXAngl/xFov) * targetYAngl);

        telemetry.addData("xAngl", targetXAngl);
        telemetry.addData("yAngl", targetYAngl);
        telemetry.addData("tAngl", targetXAngl);




        vision.updateExposure();
    }
}

