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

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.ArrayList;
import java.util.List;

import org.opencv.core.RotatedRect;
import org.opencv.core.Point;
import org.opencv.core.MatOfPoint2f;

public class YellowPipeline extends OpenCvPipeline {

    enum Stage
    {
        YCbCr,
        THRESH,
        MORPH,
        ERODE,
        EDGE,
        DST,
        RAW_IMAGE
    }

    private boolean pause = false;

    public void setPause(boolean pause){
        this.pause=pause;
    }

    Mat ycbcrMat = new Mat();
    Mat ycbcrThresh = new Mat();
    Mat ycbcrMorph = new Mat();
    Mat ycbcrErode = new Mat();
    Mat ycbcrEdge = new Mat();
    Mat dst = new Mat();

    Mat thresholdMat = new Mat();


    Scalar white = new Scalar(255, 255, 255);
    Scalar pink = new Scalar(255, 105, 180);
    Scalar black = new Scalar(0, 0, 0);


    Scalar lowThresh = new Scalar(0, 130, 0);
    Scalar highThresh = new Scalar(255, 170, 108);

    Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(15, 25)); //width was 50, 25
    Mat kernel2 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(15, 15)); //width was 15, 15

    Mat polesEdges = new Mat();
    Mat polesErode = new Mat();



    double currentCenterX = -1;


    private Stage stageToRenderToViewport = Stage.DST;
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

    public double getCurrentCenterX(){
        return currentCenterX;
    }

    @Override
    public Mat processFrame(Mat inputMat) {

        if(!pause){
            Imgproc.cvtColor(inputMat, ycbcrMat, Imgproc.COLOR_RGB2YCrCb);

            Core.inRange(ycbcrMat, lowThresh, highThresh, ycbcrThresh);


            Imgproc.morphologyEx(ycbcrThresh, ycbcrMorph, Imgproc.MORPH_OPEN, kernel);

            Imgproc.erode(ycbcrMorph, ycbcrErode, kernel2, new Point(-1,-1),4);

            Imgproc.Canny(ycbcrMorph, ycbcrEdge, 300, 600, 5, true);


            Imgproc.HoughLines(ycbcrEdge, polesEdges, 1, Math.PI/180, 120, 0, 0, -5*Math.PI/180, 5*Math.PI/180);

            Imgproc.HoughLines(ycbcrErode, polesErode, 1, Math.PI/180, 275, 0, 0, -2*Math.PI/180, 2*Math.PI/180);


            inputMat.copyTo(dst);



            double[] rovioListEroded = new double[polesErode.rows()];
            double[] rovioListEdges = new double[polesEdges.rows()];

            for (int x = 0; x < polesErode.rows(); x++) {
                double theta = polesErode.get(x, 0)[1];
                double rho = polesErode.get(x, 0)[0];
                double a = Math.cos(theta), b = Math.sin(theta);
                double x0 = a*rho, y0 = b*rho;
                Point pt1 = new Point(Math.round(x0 + 1000*(-b)), Math.round(y0 + 1000*(a)));
                Point pt2 = new Point(Math.round(x0 - 1000*(-b)), Math.round(y0 - 1000*(a)));
                Imgproc.line(dst, pt1, pt2, new Scalar(0, 0, 255), 3, Imgproc.LINE_AA, 0);
                rovioListEroded[x] = x0;
            }

            for (int x = 0; x < polesEdges.rows(); x++) {
                double theta = polesEdges.get(x, 0)[1];
                double rho = polesEdges.get(x, 0)[0];
                double a = Math.cos(theta), b = Math.sin(theta);
                double x0 = a*rho, y0 = b*rho;
                Point pt1 = new Point(Math.round(x0 + 1000*(-b)), Math.round(y0 + 1000*(a)));
                Point pt2 = new Point(Math.round(x0 - 1000*(-b)), Math.round(y0 - 1000*(a)));
                Imgproc.line(dst, pt1, pt2, new Scalar(0, 255, 0), 3, Imgproc.LINE_AA, 0);
                rovioListEdges[x] = x0;
            }


            Arrays.sort(rovioListEroded);
            Arrays.sort(rovioListEdges);


            double xErodeCenter = 0;
            if (rovioListEroded.length==0) {
                xErodeCenter = -1;
            }
            else if (rovioListEroded.length % 2 == 0)
                xErodeCenter = (rovioListEroded[rovioListEroded.length/2] + rovioListEroded[rovioListEroded.length/2-1])/2;
            else
                xErodeCenter = rovioListEroded[rovioListEroded.length/2];


            currentCenterX = -1;

            for(int i = 0; i<rovioListEdges.length; i++){
                if(rovioListEdges[i]>xErodeCenter && i!=0){
                    currentCenterX = (rovioListEdges[i] + rovioListEdges[i-1])/2;
                    break;
                }
            }

            Imgproc.line(dst, new Point(currentCenterX, 1000), new Point(currentCenterX, -10000), new Scalar(255, 0, 0), 3, Imgproc.LINE_AA, 0);



            System.out.println(currentCenterX);
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
            case ERODE:
            {
                return ycbcrErode;
            }

            case EDGE:
            {
                return ycbcrEdge;
            }

            case RAW_IMAGE:
            {
                return inputMat;
            }

            case DST:
            {
                return dst;
            }

            default:
            {
                return inputMat;
            }
        }
    }

}