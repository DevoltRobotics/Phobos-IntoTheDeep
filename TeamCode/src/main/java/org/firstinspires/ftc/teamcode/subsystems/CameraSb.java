package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Comands.Etesito;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

public class CameraSb extends OpMode {

    Etesito etesito;
    double FCL = 1;
    double fx = 238.722 * FCL;
    double fy = 238.722 * FCL;

    double cx = 323.204;
    double cy = 228.638;

    private Mat boundingImage = new Mat();

    OpenCvWebcam camera;

    MatOfPoint3f cameraMatrix = new MatOfPoint3f();
    MatOfDouble distCoeffs = new MatOfDouble(0.01, 0.01, 0.01, 0.01, 0.01);

    double objectWidth = 3.5;
    double objectHeight = 1.5;

    MatOfPoint3f objectPoints = new MatOfPoint3f(
            new Point3(objectWidth / 2, objectHeight / 2, 0),
            new Point3(-objectWidth / 2, objectHeight / 2, 0),
            new Point3(-objectWidth / 2, -objectHeight / 2, 0),
            new Point3(objectWidth / 2, -objectHeight / 2, 0));

    MatOfPoint2f imagePoints = new MatOfPoint2f(objectPoints);

    MatOfPoint3f rvec = new MatOfPoint3f(
            new Point3(objectWidth / 2, objectHeight / 2, 0),
            new Point3(-objectWidth / 2, objectHeight / 2, 0),
            new Point3(-objectWidth / 2, -objectHeight / 2, 0),
            new Point3(objectWidth / 2, -objectHeight / 2, 0));

    MatOfPoint3f tvec = new MatOfPoint3f(
            new Point3(objectWidth / 2, objectHeight / 2, 0),
            new Point3(-objectWidth / 2, objectHeight / 2, 0),
            new Point3(-objectWidth / 2, -objectHeight / 2, 0),
            new Point3(objectWidth / 2, -objectHeight / 2, 0));

    boolean succes = Calib3d.solvePnP(
            objectPoints,
            imagePoints,
            cameraMatrix,
            distCoeffs,
            rvec,
            tvec,
            false,
            Calib3d.SOLVEPNP_P3P

    );

    @Override
    public void init() {

        etesito.init(hardwareMap, true, true);

        cameraMatrix.put(0, 0,
                fx, 0, cx,
                0, fy, cy,
                0, 0, 1);
    }
    
    @Override
    public void loop() {

        if (succes){
            double[] coords = new double[3];
            tvec.get(0,0,coords);

        }


    }


}
