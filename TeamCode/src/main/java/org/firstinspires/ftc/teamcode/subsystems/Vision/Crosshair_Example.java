package org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.*;
import org.openftc.easyopencv.OpenCvPipeline;

public class Crosshair_Example extends OpenCvPipeline {

    public Scalar lowerYCrCb = new Scalar(0.0, 0.0, 0.0, 0.0);
    public Scalar upperYCrCb = new Scalar(255.0, 255.0, 94.0, 0.0);
    private Mat ycrcbMat = new Mat();
    private Mat ycrcbBinaryMat = new Mat();

    public Scalar lowerHSV = new Scalar(91.0, 0.0, 0.0, 0.0);
    public Scalar upperHSV = new Scalar(255.0, 255.0, 255.0, 0.0);
    private Mat hsvMat = new Mat();
    private Mat hsvBinaryMat = new Mat();

    public Mat bitwiseNOTMat = new Mat();

    public Mat bitwiseANDMat = new Mat();

    public int erodeValue = ((int) (40));
    public int dilateValue = ((int) (40));
    private Mat element = null;
    private Mat bitwiseANDMatErodedDilated = new Mat();

    private ArrayList<MatOfPoint> contours = new ArrayList<>();
    private Mat hierarchy = new Mat();

    public int minArea = 2000;
    public int maxArea = 30000;
    private ArrayList<MatOfPoint> contoursByArea = new ArrayList<>();

    public int minRatio = 0;
    public int maxRatio = 100;
    private ArrayList<MatOfPoint> contoursByAreaByRatio = new ArrayList<>();
    private MatOfPoint2f contoursByArea2f = new MatOfPoint2f();

    public Scalar lineColor = new Scalar(181.0, 0.0, 255.0, 0.0);
    public int lineThickness = 3;

    private ArrayList<MatOfPoint> crosshair = new ArrayList<>();
    private Mat crosshairImage = new Mat();
    public int crosshairSize = 5;

    public double vectorX = 0.0;
    public double vectorY = 0.0;

    public Scalar lineColor1 = new Scalar(0.0, 255.0, 0.0, 0.0);
    public int lineThickness1 = 3;

    private MatOfPoint2f crosshair2f = new MatOfPoint2f();
    private ArrayList<RotatedRect> crosshairRotRects = new ArrayList<>();

    private RotatedRect biggestRotRect = null;

    private HashMap<String, Rect> rectTargets = new HashMap<>();
    private HashMap<String, RotatedRect> rotRectTarget = new HashMap<>();

    private Mat crosshairImageRotRects = new Mat();

    private MatOfPoint biggestContour = null;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);
        Core.inRange(ycrcbMat, lowerYCrCb, upperYCrCb, ycrcbBinaryMat);

        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvMat, lowerHSV, upperHSV, hsvBinaryMat);

        bitwiseNOTMat.release();
        Core.bitwise_not(hsvBinaryMat, bitwiseNOTMat);

        bitwiseANDMat.release();
        Core.bitwise_and(ycrcbBinaryMat, bitwiseNOTMat, bitwiseANDMat);

        bitwiseANDMat.copyTo(bitwiseANDMatErodedDilated);
        if(erodeValue > 0) {
            this.element = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(erodeValue, erodeValue));
            Imgproc.erode(bitwiseANDMatErodedDilated, bitwiseANDMatErodedDilated, element);

            element.release();
        }

        if(dilateValue > 0) {
            this.element = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(dilateValue, dilateValue));
            Imgproc.dilate(bitwiseANDMatErodedDilated, bitwiseANDMatErodedDilated, element);

            element.release();
        }

        contours.clear();
        hierarchy.release();
        Imgproc.findContours(bitwiseANDMatErodedDilated, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        contoursByArea.clear();
        for(MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if((area >= minArea) && (area <= maxArea)) {
                contoursByArea.add(contour);
            }
        }

        contoursByAreaByRatio.clear();
        for(MatOfPoint contour : contoursByArea) {
            contoursByArea2f.release();
            contour.convertTo(contoursByArea2f, CvType.CV_32F);
            RotatedRect rect = Imgproc.minAreaRect(contoursByArea2f);

            double width = ((double) (rect.size.width));
            double height = ((double) (rect.size.height));
            if(height > width) {
                double temp = width;
                width = height;
                height = temp;
            }

            double ratio = height / width;

            if((ratio >= minRatio / 100.0) && (ratio <= maxRatio / 100.0)) {
                contoursByAreaByRatio.add(contour);
            }
        }

        input.copyTo(crosshairImage);

        Point crosshairPoint = new Point((((double) (input.cols())) / 2) + vectorX, (((double) (input.rows())) / 2) + vectorY);
        int scaleFactor = (input.rows() + input.cols()) / 2;

        int adjustedCrosshairSize = (crosshairSize * scaleFactor) / 100;

        Imgproc.line(crosshairImage, new Point(crosshairPoint.x - adjustedCrosshairSize, crosshairPoint.y), new Point(crosshairPoint.x + adjustedCrosshairSize, crosshairPoint.y), lineColor, lineThickness);
        Imgproc.line(crosshairImage, new Point(crosshairPoint.x, crosshairPoint.y - adjustedCrosshairSize), new Point(crosshairPoint.x, crosshairPoint.y + adjustedCrosshairSize), lineColor, lineThickness);

        crosshair.clear();

        double currDist = 0.0;
        MatOfPoint closestContour = null;

        for(MatOfPoint contour : contoursByAreaByRatio) {
            Rect boundingRect = Imgproc.boundingRect(contour);

            double newDist = Math.sqrt(Math.pow(crosshairPoint.x - (boundingRect.x + (boundingRect.width / 2)), 2) + Math.pow(crosshairPoint.y - (boundingRect.y + (boundingRect.height / 2)), 2));
            if((closestContour == null) || (Math.abs(newDist) <= currDist)) {
                currDist = Math.abs(newDist);
                closestContour = contour;
            }
        }
        if(closestContour != null) {
            crosshair.add(closestContour);
        }

        crosshairRotRects.clear();
        for(MatOfPoint points : crosshair) {
            crosshair2f.release();
            points.convertTo(crosshair2f, CvType.CV_32F);

            crosshairRotRects.add(Imgproc.minAreaRect(crosshair2f));
        }

        this.biggestRotRect = null;
        for(RotatedRect rect : crosshairRotRects) {
            if(rect != null) {
                if((biggestRotRect == null) || (rect.size.area() > biggestRotRect.size.area())) {
                    this.biggestRotRect = rect;
                }
            }
        }

        clearTargets();

        addRotRectTarget("sample", biggestRotRect);

        crosshairImage.copyTo(crosshairImageRotRects);
        for(RotatedRect rect : crosshairRotRects) {
            if(rect != null) {
                Point[] rectPoints = new Point[4];
                rect.points(rectPoints);
                MatOfPoint matOfPoint = new MatOfPoint(rectPoints);

                Imgproc.polylines(crosshairImageRotRects, Collections.singletonList(matOfPoint), true, lineColor1, lineThickness1);
            }
        }

        this.biggestContour = null;
        for(MatOfPoint contour : crosshair) {
            if((biggestContour == null) || (Imgproc.contourArea(contour) > Imgproc.contourArea(biggestContour))) {
                this.biggestContour = contour;
            }
        }

        if(biggestContour != null) {
            Rect boundingRect = Imgproc.boundingRect(biggestContour);

            double centroidX = (boundingRect.tl().x + boundingRect.br().x) / 2;
            double centroidY = (boundingRect.tl().y + boundingRect.br().y) / 2;

            Point centroid = new Point(centroidX, centroidY);
            double contourArea = Imgproc.contourArea(biggestContour);

            Scalar crosshairCol = new Scalar(0.0, 255.0, 0.0);

            Imgproc.line(crosshairImageRotRects, new Point(centroidX - 10, centroidY), new Point(centroidX + 10, centroidY), crosshairCol, 5);
            Imgproc.line(crosshairImageRotRects, new Point(centroidX, centroidY - 10), new Point(centroidX, centroidY + 10), crosshairCol, 5);
        }

        return crosshairImageRotRects;
    }

    private synchronized void clearTargets() {
        rectTargets.clear();
        rotRectTarget.clear();
    }

    private synchronized void addRectTarget(String label, Rect rect) {
        rectTargets.put(label, rect);
    }
    private synchronized void addRotRectTarget(String label, RotatedRect rotRect) {
        rotRectTarget.put(label, rotRect);
    }

    public synchronized Rect getRectTarget(String label) {
        return ((Rect) (rectTargets.get(label)));
    }
    public synchronized RotatedRect getRotRectTarget(String label) {
        return ((RotatedRect) (rotRectTarget.get(label)));
    }

}