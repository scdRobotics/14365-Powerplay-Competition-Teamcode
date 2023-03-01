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

    List<MatOfPoint> contoursList = new ArrayList<>();

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

    Mat ycbcrMat = new Mat();
    Mat ycbcrThresh = new Mat();
    Mat ycbcrMorph = new Mat();
    Mat ycbcrErode = new Mat();
    Mat ycbcrEdge = new Mat();
    Mat dst = new Mat();

    ArrayList<RectData> rects = new ArrayList<RectData>();

    Mat thresholdMat = new Mat();


    Scalar white = new Scalar(255, 255, 255);
    Scalar pink = new Scalar(255, 105, 180);
    Scalar black = new Scalar(0, 0, 0);


    Scalar lowThresh = new Scalar(0, 130, 0);
    Scalar highThresh = new Scalar(255, 170, 108);

    Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(15, 25)); //width was 50, 25
    Mat kernel2 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(15, 15)); //width was 15, 15

    Mat poles = new Mat();
    Mat polesErode = new Mat();


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

    public ArrayList<RectData> getRects(){
        return new ArrayList<RectData>(rects);
    }

    @Override
    public Mat processFrame(Mat inputMat) {



        contoursList.clear();

        Imgproc.cvtColor(inputMat, ycbcrMat, Imgproc.COLOR_RGB2YCrCb);

        Core.inRange(ycbcrMat, lowThresh, highThresh, ycbcrThresh);

        Imgproc.morphologyEx(ycbcrThresh, ycbcrMorph, Imgproc.MORPH_OPEN, kernel);

        Imgproc.erode(ycbcrMorph, ycbcrErode, kernel2, new Point(-1,-1),4);

        Imgproc.Canny(ycbcrMorph, ycbcrEdge, 300, 600, 5, true);



        //Imgproc.findContours(ycbcrThresh, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        //Imgproc.HoughLines(ycbcrMorph, poles, 1, Math.PI/180, 200, 300);

        Imgproc.HoughLines(ycbcrEdge, poles, 1, Math.PI/180, 120, 0, 0, -5*Math.PI/180, 5*Math.PI/180);

        Imgproc.HoughLines(ycbcrErode, polesErode, 1, Math.PI/180, 120, 0, 0, -2*Math.PI/180, 2*Math.PI/180);

        //Imgproc.findContours(ycbcrMorph, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        inputMat.copyTo(dst);

        //Imgproc.drawContours(dst, contoursList, -1, white, 3, 4);

        //Imgproc.HoughLinesP(ycbcrThresh, poles, )



        double[] rovioList = new double[polesErode.rows()];
        //ArrayList<Double> rovioList1 = new ArrayList<>;
        //ArrayList<Double> rovioList2 = new ArrayList<>();

        for (int x = 0; x < polesErode.rows(); x++) {
            double theta = polesErode.get(x, 0)[1];
            double rho = polesErode.get(x, 0)[0];
            double a = Math.cos(theta), b = Math.sin(theta);
            double x0 = a*rho, y0 = b*rho;
            Point pt1 = new Point(Math.round(x0 + 1000*(-b)), Math.round(y0 + 1000*(a)));
            Point pt2 = new Point(Math.round(x0 - 1000*(-b)), Math.round(y0 - 1000*(a)));
            Imgproc.line(dst, pt1, pt2, new Scalar(0, 0, 255), 3, Imgproc.LINE_AA, 0);
            rovioList[x] = x0;
        }

        double xErodeCenter = 0;
        if (rovioList.length % 2 == 0)
            xErodeCenter = (rovioList[rovioList.length/2] + rovioList[rovioList.length/2-1])/2;
        else
            xErodeCenter = rovioList[rovioList.length/2];

        /*for (int x = 0; x < poles.rows(); x++) {
            double theta = poles.get(x, 0)[1];
            double rho = poles.get(x, 0)[0];
            double a = Math.cos(theta), b = Math.sin(theta);
            double x0 = a*rho, y0 = b*rho;
            Point pt1 = new Point(Math.round(x0 + 1000*(-b)), Math.round(y0 + 1000*(a)));
            Point pt2 = new Point(Math.round(x0 - 1000*(-b)), Math.round(y0 - 1000*(a)));
            Imgproc.line(dst, pt1, pt2, new Scalar(0, 0, 255), 3, Imgproc.LINE_AA, 0);
            rovioList[x] = x0;
        }*/

        //Arrays.sort(rovioList);

        int count = 0;
        for(double d: rovioList){
            System.out.println("X #" + d + ", Count #" + count);
            count++;
        }

        /*if(Math.abs(rovioList[0] - rovioList[poles.rows()) <)
        {
            double xCenter = (rovioList[0] + rovioList[poles.rows() - 1]) / 2;
            Imgproc.line(dst, new Point(xCenter, 720), new Point(xCenter, 0), new Scalar(0, 0, 255), 3, Imgproc.LINE_AA, 0);
        }*/



        /*for(int i = 0; i < poles.rows(); i++)
        {
            //rovioList1.add();
        }


        /*for (MatOfPoint contour: contoursList){
            Imgproc.fillPoly(dst, Arrays.asList(contour), white);
            for(Point p: contour.toArray()){
                Imgproc.circle(dst, p, 10, pink);



            }

        }*/



        rects.clear();

        /*for (MatOfPoint contour: contoursList) {
            RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));

            double fixedAngle;

            if (rotatedRect.size.width < rotatedRect.size.height) {
                fixedAngle = rotatedRect.angle + 180;
            } else {
                fixedAngle = rotatedRect.angle + 90;
            }

            if(fixedAngle>=170 && fixedAngle<=190){
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
            }

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
        }*/





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