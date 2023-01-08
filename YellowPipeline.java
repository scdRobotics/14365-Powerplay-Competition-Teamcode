package org.firstinspires.ftc.teamcode;

import static org.opencv.imgproc.Imgproc.THRESH_BINARY_INV;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;
import java.util.ArrayList;
import java.util.List;

import org.opencv.core.RotatedRect;
import org.opencv.core.Point;
import org.opencv.core.MatOfPoint2f;

public class YellowPipeline extends OpenCvPipeline {

    List<MatOfPoint> contoursList = new ArrayList<>();

    enum Stage
    {
        YCbCr,
        YCbCr_CHAN1,
        YCbCr_CHAN2,
        THRESHOLD,
        CONTOURS_OVERLAYED_ON_FRAME,
        RAW_IMAGE
    }

    Mat ycbcrMat = new Mat();

    Mat ycbcrChan2Mat = new Mat();

    Mat thresholdMat = new Mat();

    Mat contoursMat = new Mat();

    Mat ycbcrChan1Mat = new Mat();

    ArrayList<RectData> rects = new ArrayList<RectData>();

    int numContoursFound;

    private Stage stageToRenderToViewport = Stage.CONTOURS_OVERLAYED_ON_FRAME;
    private Stage[] stages = Stage.values();

    @Override
    public void onViewportTapped()
    {
        /*
         * Note that this method is invoked from the UI thread
         * so whatever we do here, we must do quickly.
         */

        int currentStageNum = stageToRenderToViewport.ordinal();

        int nextStageNum = currentStageNum + 1;

        if(nextStageNum >= stages.length)
        {
            nextStageNum = 0;
        }

        stageToRenderToViewport = stages[nextStageNum];
    }

    public static void drawRotatedRect(Mat image, RotatedRect rotatedRect, Scalar color, int thickness) {
        Point[] vertices = new Point[4];
        rotatedRect.points(vertices);
        MatOfPoint points = new MatOfPoint(vertices);
        Imgproc.drawContours(image, Arrays.asList(points), -1, color, thickness);
    }

    public ArrayList<RectData> getRects(){
        return rects;
    }

    @Override
    public Mat processFrame(Mat inputMat) {

        Scalar white = new Scalar(255, 255, 255);

        contoursList.clear();

        Imgproc.cvtColor(inputMat, ycbcrMat, Imgproc.COLOR_RGB2YCrCb);

        Core.extractChannel(ycbcrMat, ycbcrChan2Mat, 2);

        Imgproc.threshold(ycbcrChan2Mat, thresholdMat, 95, 255, THRESH_BINARY_INV);

        Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        inputMat.copyTo(contoursMat);

        Imgproc.drawContours(contoursMat, contoursList, -1, white, 3, 8);

        numContoursFound = contoursList.size();

        for (MatOfPoint contour: contoursList){
            Imgproc.fillPoly(thresholdMat, Arrays.asList(contour), white);
        }

        Scalar black = new Scalar(0, 0, 0);

        rects.clear();

        for (MatOfPoint contour: contoursList) {
            RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));

            double fixedAngle;

            if(rotatedRect.size.width < rotatedRect.size.height){
                fixedAngle = rotatedRect.angle+180;
            }else{
                fixedAngle = rotatedRect.angle+90;
            }

            if(fixedAngle>=170 && fixedAngle<=190){
                if(rotatedRect.size.width>5 && rotatedRect.size.height>25){
                    drawRotatedRect(contoursMat, rotatedRect, black, 10);
                    if(rotatedRect.size.width>rotatedRect.size.height) {
                        rects.add(new RectData(rotatedRect.size.height, rotatedRect.size.width, rotatedRect.center.x, rotatedRect.center.y));
                    }
                    else{
                        rects.add(new RectData(rotatedRect.size.width, rotatedRect.size.height, rotatedRect.center.x, rotatedRect.center.y));
                    }
                }
            }
        }





        switch (stageToRenderToViewport)
        {
            case YCbCr:
            {
                return ycbcrMat;
            }

            case YCbCr_CHAN1:
            {
                return ycbcrChan1Mat;
            }

            case YCbCr_CHAN2:
            {
                return ycbcrChan2Mat;
            }

            case THRESHOLD:
            {
                return thresholdMat;
            }

            case CONTOURS_OVERLAYED_ON_FRAME:
            {
                return contoursMat;
            }

            case RAW_IMAGE:
            {
                return inputMat;
            }

            default:
            {
                return inputMat;
            }
        }
    }

}