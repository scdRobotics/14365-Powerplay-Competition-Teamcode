package org.firstinspires.ftc.teamcode;
//Package is a VERY important step! Required to do basically anything with the robot

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
//Most imports are automatically handled by Android Studio as you program



public class Vision extends Subsystem {
    public OpenCvCamera webcam1;
    //TODO: Figure out which of these is upside down and account for that accordingly in the respective pipeline
    public OpenCvCamera webcam2;

    AprilTagYellowPipeline aprilTagYellowPipeline = new AprilTagYellowPipeline(1, 578.272, 578.272, 402.145, 221.506);

    YellowPipeline yellowPipeline = new YellowPipeline();

    public ArrayList<RectData> same = new ArrayList<RectData>(); //Only global because I want to access it in VisionTest.java; SHOULD CHANGE TO BE INSIDE FUNCTION findClosePoleDTheta() ONCE TESTING IS DONE !!

    //"Constructor" object for Vision
    public Vision(OpenCvCamera webcam1, OpenCvCamera webcam2, Telemetry telemetry, HardwareMap hardwareMap, ElapsedTime timer){
        super(telemetry, hardwareMap, timer);
        this.webcam1=webcam1;
        this.webcam2=webcam2;


    }

    public void activateAprilTagYellowPipelineCamera1(){
        webcam1.setPipeline(aprilTagYellowPipeline);
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam1.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("Camera Opened! ", "");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    public void activateYellowPipelineCamera2(){
        webcam2.setPipeline(yellowPipeline);
        webcam2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam2.startStreaming(1280,720, OpenCvCameraRotation.UPSIDE_DOWN);
                telemetry.addData("Camera Opened! ", "");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    public void runAprilTag(boolean runAprilTag){
        aprilTagYellowPipeline.setRunAprilTag(runAprilTag);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public double findClosePoleDTheta(){
        ArrayList<RectData> viewCam1 = aprilTagYellowPipeline.getRects();
        ArrayList<RectData> viewCam2 = yellowPipeline.getRects();

        if(!viewCam1.isEmpty() && !viewCam2.isEmpty()){

            viewCam1.sort(Comparator.comparing(RectData::getWidth));
            
            viewCam2.sort(Comparator.comparing(RectData::getWidth));



            RectData widest1 = null;
            RectData widest2 = null;

            widest1 = viewCam1.get(viewCam1.size()-1);
            widest2 = viewCam2.get(viewCam2.size()-1);
            telemetry.addData("Widest 1: ", widest1);
            telemetry.addData("Widest 2: ", widest2);
            same.add(widest1);
            same.add(widest2);


        }


        else{
            //TODO: If same.size() is >2, isolate which equal rects within "same" are the pole we want to be looking at (AKA, the widest pair that still is a pole and not some strange background object/interference) so that same.size()==2
            return -1; //failure case, nothing detected more than likely (or flaw in selecting "same" poles)
        }

        //TODO: Implement cool Rovio formula to find and return dTheta to turn
        double theta1 = Math.toRadians(142.5  - (same.get(0).getX()*5.5/128)); //Camera 1 Theta
        double theta2 = Math.toRadians(92.5 - (same.get(1).getX()*5.5/128)); //Camera 2 Theta

        telemetry.addData("Theta 1 (Radians): ", theta1);
        telemetry.addData("Theta 2 (Radians): ", theta2);

        double c1;
        double c2;
        double c3;
        double dx;
        double dy;
        double phi;
        double a = 6.825;
        double b = 2;

        c1 = theta1;
        c2 = theta2;

        dx = a * (Math.tan(c1) + Math.tan(c2))/(Math.tan(c1)-Math.tan(c2));
        dy = a * (Math.tan(c1) * Math.tan(c2))/(Math.tan(c1)-Math.tan(c2));

        c3 = Math.atan((dy - b)/dx);

        //double dTheta = Math.atan((13.5*Math.sin(theta2)*Math.sin(theta1)/Math.sin(theta1 - theta2) - 2.25) /(6.75 + (13.5*Math.sin(theta2)*Math.cos(theta1)/(Math.sin(theta1 - theta2))))) - (3.14159265358979323846264338327950/2);

        double dTheta = c3 - Math.PI/2;

        if(dTheta < -Math.PI/2) {
            dTheta = Math.PI+dTheta;
        }

        return dTheta;

    }

    /*public void deactivateAprilTagPipelineCamera1(){
        webcam1.closeCameraDevice();
    }*/

    /*public void activateYellowPipelineBothCameras(){
        webcam1.setPipeline(yellowPipeline);
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
                telemetry.addData("Camera 1 Opened! ", "");
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        webcam2.setPipeline(yellowPipeline);
        webcam2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam2.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
                telemetry.addData("Camera 2 Opened! ", "");
            }

            @Override
            public void onError(int errorCode) {

            }
        });


    }*/

    public int readAprilTagCamera1() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagYellowPipeline.getLatestDetections();

        if (currentDetections.size() != 0) {

            for (AprilTagDetection tag : currentDetections) {
                return tag.id;
            }
        }
        else{
            return -1;
        }
        return -1;

    }
}
