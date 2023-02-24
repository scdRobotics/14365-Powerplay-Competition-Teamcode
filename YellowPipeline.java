package org.firstinspires.ftc.teamcode;

import static org.opencv.imgproc.Imgproc.THRESH_BINARY;
import static org.opencv.imgproc.Imgproc.THRESH_BINARY_INV;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
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
        THRESH,
        MORPH,
        CONTOURS,
        RAW_IMAGE
    }

    Mat ycbcrMat = new Mat();
    Mat ycbcrThresh = new Mat();
    Mat contoursMat = new Mat();
    Mat ycbcrMorph = new Mat();

    ArrayList<RectData> rects = new ArrayList<RectData>();

    Mat thresholdMat = new Mat();


    Scalar white = new Scalar(255, 255, 255);
    Scalar pink = new Scalar(255, 105, 180);
    Scalar black = new Scalar(0, 0, 0);


    Scalar lowThresh = new Scalar(0, 130, 0);
    Scalar highThresh = new Scalar(255, 170, 90);

    Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(15, 25)); //width was 50, 25

    Mat poles = new Mat();


    private Stage stageToRenderToViewport = Stage.CONTOURS;
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
        return new ArrayList<RectData>(rects);
    }

    @Override
    public Mat processFrame(Mat inputMat) {



        contoursList.clear();

        Imgproc.cvtColor(inputMat, ycbcrMat, Imgproc.COLOR_RGB2YCrCb);

        Core.inRange(ycbcrMat, lowThresh, highThresh, ycbcrThresh);



        Imgproc.morphologyEx(ycbcrThresh, ycbcrMorph, Imgproc.MORPH_OPEN, kernel);

        //Imgproc.findContours(ycbcrThresh, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.findContours(ycbcrMorph, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        inputMat.copyTo(contoursMat);

        Imgproc.drawContours(contoursMat, contoursList, -1, white, 3, 4);

        //Imgproc.HoughLinesP(ycbcrThresh, poles, )


        for (MatOfPoint contour: contoursList){
            Imgproc.fillPoly(contoursMat, Arrays.asList(contour), white);
            for(Point p: contour.toArray()){
                Imgproc.circle(contoursMat, p, 10, pink);



            }

        }



        rects.clear();

        for (MatOfPoint contour: contoursList) {
            RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));

            double fixedAngle;

            if (rotatedRect.size.width < rotatedRect.size.height) {
                fixedAngle = rotatedRect.angle + 180;
            } else {
                fixedAngle = rotatedRect.angle + 90;
            }

            /*if(fixedAngle>=170 && fixedAngle<=190){
                if(rotatedRect.size.width>16  && rotatedRect.size.height>200){
                    if(rotatedRect.size.width>rotatedRect.size.height) {
                        drawRotatedRect(thresholdMat, rotatedRect, yellow, 10);
                        rects.add(new RectData(rotatedRect.size.height, rotatedRect.size.width, rotatedRect.center.x, rotatedRect.center.y));
                    }
                    else{
                        drawRotatedRect(thresholdMat, rotatedRect, yellow, 10);
                        rects.add(new RectData(rotatedRect.size.width, rotatedRect.size.height, rotatedRect.center.x, rotatedRect.center.y));
                    }
                }
            }*/

            double x = rotatedRect.center.x;

            double y = rotatedRect.center.y;

            if (rotatedRect.size.width > rotatedRect.size.height) {
                //if ( (fixedAngle >= 160 && fixedAngle <= 200) && (x > (1080*0.3) && x < (1080*0.7)) ) {
                if ( (fixedAngle >= 160 && fixedAngle <= 200) && rotatedRect.size.height>350) {
                    //if ( (x > (1080*0.3) && x < (1080*0.7)) ) {
                    double correctWidth = rotatedRect.size.height;
                    double correctHeight = rotatedRect.size.width;
                    drawRotatedRect(contoursMat, rotatedRect, black, 10);
                    rects.add(new RectData(correctHeight, correctWidth, rotatedRect.center.x, rotatedRect.center.y));
                }
            } else {
                if ( (fixedAngle >= 160 && fixedAngle <= 200) && rotatedRect.size.height>55 && (rotatedRect.size.height>350)) {
                    //if ( (x > (1080*0.3) && x < (1080*0.7)) ) {
                    double correctWidth = rotatedRect.size.width;
                    double correctHeight = rotatedRect.size.height;
                    drawRotatedRect(contoursMat, rotatedRect, black, 10);
                    rects.add(new RectData(correctHeight, correctWidth, rotatedRect.center.x, rotatedRect.center.y));
                }
            }
        }





        switch (stageToRenderToViewport)
        {
            case YCbCr:
            {
                return ycbcrMat;
            }

            case THRESH:
            {
                return ycbcrThresh;
            }

            case MORPH:
            {
                return ycbcrMorph;
            }

            case CONTOURS:
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