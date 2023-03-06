package org.firstinspires.ftc.teamcode;

import static org.opencv.imgproc.Imgproc.THRESH_BINARY;
import static org.opencv.imgproc.Imgproc.THRESH_BINARY_INV;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class AprilTagYellowPipeline extends OpenCvPipeline {

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

    private boolean runAprilTag = true;

    private boolean pause = false;

    public void setRunAprilTag(boolean runAprilTag){
        this.runAprilTag=runAprilTag;
    }

    public void setPause(boolean pause){
        this.pause=pause;
    }

    private long nativeApriltagPtr;
    private Mat grey = new Mat();
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();

    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
    private final Object detectionsUpdateSync = new Object();

    Mat cameraMatrix;

    Scalar blue = new Scalar(7,197,235,255);
    Scalar red = new Scalar(255,0,0,255);
    Scalar green = new Scalar(0,255,0,255);
    Scalar white = new Scalar(255,255,255,255);

    double fx;
    double fy;
    double cx;
    double cy;

    // UNITS ARE METERS
    double tagsize;
    double tagsizeX;
    double tagsizeY;

    private float decimation;
    private boolean needToSetDecimation;
    private final Object decimationSync = new Object();

    Mat ycbcrMat = new Mat();
    Mat ycbcrThresh = new Mat();
    Mat ycbcrMorph = new Mat();
    Mat ycbcrErode = new Mat();
    Mat ycbcrEdge = new Mat();
    Mat dst = new Mat();

    Mat thresholdMat = new Mat();


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



    public AprilTagYellowPipeline(double tagsize, double fx, double fy, double cx, double cy)
    {
        this.tagsize = tagsize;
        this.tagsizeX = tagsize;
        this.tagsizeY = tagsize;
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;

        constructMatrix();

        // Allocate a native context object. See the corresponding deletion in the finalizer
        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    }

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

    @Override
    public void finalize()
    {
        // Might be null if createApriltagDetector() threw an exception
        if(nativeApriltagPtr != 0)
        {
            // Delete the native context we created in the constructor
            AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
            nativeApriltagPtr = 0;
        }
        else
        {
            System.out.println("AprilTagDetectionPipeline.finalize(): nativeApriltagPtr was NULL");
        }
    }

    public double getCurrentCenterX(){
        return currentCenterX;
    }

    @Override
    public Mat processFrame(Mat inputMat) {
        // Convert to greyscale

        if(!pause){
            if (runAprilTag) {
                Imgproc.cvtColor(inputMat, grey, Imgproc.COLOR_RGBA2GRAY);

                synchronized (decimationSync) {
                    if (needToSetDecimation) {
                        AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation);
                        needToSetDecimation = false;
                    }
                }

                // Run AprilTag
                detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, tagsize, fx, fy, cx, cy);

                synchronized (detectionsUpdateSync) {
                    detectionsUpdate = detections;
                }

                // For fun, use OpenCV to draw 6DOF markers on the image. We actually recompute the pose using
                // OpenCV because I haven't yet figured out how to re-use AprilTag's pose in OpenCV.
                for (AprilTagDetection detection : detections) {
                    Pose pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagsizeX, tagsizeY);
                    drawAxisMarker(inputMat, tagsizeY / 2.0, 6, pose.rvec, pose.tvec, cameraMatrix);
                    draw3dCubeMarker(inputMat, tagsizeX, tagsizeX, tagsizeY, 5, pose.rvec, pose.tvec, cameraMatrix);
                }
            }






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



    public void setDecimation(float decimation)
    {
        synchronized (decimationSync)
        {
            this.decimation = decimation;
            needToSetDecimation = true;
        }
    }

    public ArrayList<AprilTagDetection> getLatestDetections()
    {
        return detections;
    }

    public ArrayList<AprilTagDetection> getDetectionsUpdate()
    {
        synchronized (detectionsUpdateSync)
        {
            ArrayList<AprilTagDetection> ret = detectionsUpdate;
            detectionsUpdate = null;
            return ret;
        }
    }

    void constructMatrix()
    {
        //     Construct the camera matrix.
        //
        //      --         --
        //     | fx   0   cx |
        //     | 0    fy  cy |
        //     | 0    0   1  |
        //      --         --
        //

        cameraMatrix = new Mat(3,3, CvType.CV_32FC1);

        cameraMatrix.put(0,0, fx);
        cameraMatrix.put(0,1,0);
        cameraMatrix.put(0,2, cx);

        cameraMatrix.put(1,0,0);
        cameraMatrix.put(1,1,fy);
        cameraMatrix.put(1,2,cy);

        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2,1,0);
        cameraMatrix.put(2,2,1);
    }

    /**
     * Draw a 3D axis marker on a detection. (Similar to what Vuforia does)
     *
     * @param buf the RGB buffer on which to draw the marker
     * @param length the length of each of the marker 'poles'
     * @param rvec the rotation vector of the detection
     * @param tvec the translation vector of the detection
     * @param cameraMatrix the camera matrix used when finding the detection
     */
    void drawAxisMarker(Mat buf, double length, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
    {
        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        MatOfPoint3f axis = new MatOfPoint3f(
                new Point3(0,0,0),
                new Point3(length,0,0),
                new Point3(0,length,0),
                new Point3(0,0,-length)
        );

        // Project those points
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        // Draw the marker!
        Imgproc.line(buf, projectedPoints[0], projectedPoints[1], red, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[2], green, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[3], blue, thickness);

        Imgproc.circle(buf, projectedPoints[0], thickness, white, -1);
    }

    void draw3dCubeMarker(Mat buf, double length, double tagWidth, double tagHeight, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
    {
        //axis = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],
        //       [0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3] ])

        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        MatOfPoint3f axis = new MatOfPoint3f(
                new Point3(-tagWidth/2, tagHeight/2,0),
                new Point3( tagWidth/2, tagHeight/2,0),
                new Point3( tagWidth/2,-tagHeight/2,0),
                new Point3(-tagWidth/2,-tagHeight/2,0),
                new Point3(-tagWidth/2, tagHeight/2,-length),
                new Point3( tagWidth/2, tagHeight/2,-length),
                new Point3( tagWidth/2,-tagHeight/2,-length),
                new Point3(-tagWidth/2,-tagHeight/2,-length));

        // Project those points
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        // Pillars
        for(int i = 0; i < 4; i++)
        {
            Imgproc.line(buf, projectedPoints[i], projectedPoints[i+4], blue, thickness);
        }

        // Base lines
        //Imgproc.line(buf, projectedPoints[0], projectedPoints[1], blue, thickness);
        //Imgproc.line(buf, projectedPoints[1], projectedPoints[2], blue, thickness);
        //Imgproc.line(buf, projectedPoints[2], projectedPoints[3], blue, thickness);
        //Imgproc.line(buf, projectedPoints[3], projectedPoints[0], blue, thickness);

        // Top lines
        Imgproc.line(buf, projectedPoints[4], projectedPoints[5], green, thickness);
        Imgproc.line(buf, projectedPoints[5], projectedPoints[6], green, thickness);
        Imgproc.line(buf, projectedPoints[6], projectedPoints[7], green, thickness);
        Imgproc.line(buf, projectedPoints[4], projectedPoints[7], green, thickness);
    }

    /**
     * Extracts 6DOF pose from a trapezoid, using a camera intrinsics matrix and the
     * original size of the tag.
     *
     * @param points the points which form the trapezoid
     * @param cameraMatrix the camera intrinsics matrix
     * @param tagsizeX the original width of the tag
     * @param tagsizeY the original height of the tag
     * @return the 6DOF pose of the camera relative to the tag
     */
    Pose poseFromTrapezoid(Point[] points, Mat cameraMatrix, double tagsizeX , double tagsizeY)
    {
        // The actual 2d points of the tag detected in the image
        MatOfPoint2f points2d = new MatOfPoint2f(points);

        // The 3d points of the tag in an 'ideal projection'
        Point3[] arrayPoints3d = new Point3[4];
        arrayPoints3d[0] = new Point3(-tagsizeX/2, tagsizeY/2, 0);
        arrayPoints3d[1] = new Point3(tagsizeX/2, tagsizeY/2, 0);
        arrayPoints3d[2] = new Point3(tagsizeX/2, -tagsizeY/2, 0);
        arrayPoints3d[3] = new Point3(-tagsizeX/2, -tagsizeY/2, 0);
        MatOfPoint3f points3d = new MatOfPoint3f(arrayPoints3d);

        // Using this information, actually solve for pose
        Pose pose = new Pose();
        Calib3d.solvePnP(points3d, points2d, cameraMatrix, new MatOfDouble(), pose.rvec, pose.tvec, false);

        return pose;
    }

    /*
     * A simple container to hold both rotation and translation
     * vectors, which together form a 6DOF pose.
     */
    class Pose
    {
        Mat rvec;
        Mat tvec;

        public Pose()
        {
            rvec = new Mat();
            tvec = new Mat();
        }

        public Pose(Mat rvec, Mat tvec)
        {
            this.rvec = rvec;
            this.tvec = tvec;
        }
    }
}
