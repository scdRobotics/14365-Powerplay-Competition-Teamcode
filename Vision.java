package org.firstinspires.ftc.teamcode;
//Package is a VERY important step! Required to do basically anything with the robot

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
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
                webcam1.startStreaming(1280,720, OpenCvCameraRotation.UPSIDE_DOWN);
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
                //webcam2.startStreaming(1280,720, OpenCvCameraRotation.UPSIDE_DOWN);
                telemetry.addData("Camera Opened! ", "");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    public double findClosePoleDTheta(){
        ArrayList<RectData> viewCam1 = dropLowWidths(aprilTagYellowPipeline.getRects(), 3);
        ArrayList<RectData> viewCam2 = dropLowWidths(yellowPipeline.getRects(), 3);

        for(int i = 0; i< viewCam1.size(); i++){
            for(int j = 0; j<viewCam2.size(); j++){
                if(viewCam1.get(i).equals(viewCam2.get(j))){
                    same.add(viewCam1.get(i));
                    same.add(viewCam2.get(j));
                }
            }
        }

        if(same.size()==2){
            //TODO: Implement cool Rovio formula to find and return dTheta to turn
            double theta0 = 110 - (same.get(0).getX()*9/128);
            double theta1 = 160 - (same.get(1).getX()*9/128);

            double dTheta = Math.atan((20*Math.sin(theta0)*Math.sin(theta1)/Math.sin(theta0 - theta1) + 10) /(10 + (20*Math.sin(theta0)*Math.cos(theta1)/(Math.sin(theta0 - theta1)))));

            return dTheta;
        }
        else{
            //TODO: If same.size() is >2, isolate which equal rects within "same" are the pole we want to be looking at (AKA, the widest pair that still is a pole and not some strange background object/interference) so that same.size()==2
            return -1; //failure case, nothing detected more than likely (or flaw in selecting "same" poles)
        }



    }

    public ArrayList<RectData> dropLowWidths(ArrayList<RectData> rects, int numToKeep){
        //Sort Array
        int i;
        double key;
        int j;

        for (i = 1; i < rects.size(); i++) {
            key = rects.get(i).getWidth();
            j = i - 1;

            // Move elements of arr[0..i-1],
            // that are greater than key, to one
            // position ahead of their
            // current position
            while (j >= 0 && (rects.get(j).getWidth() > key)) {
                rects.get(j+1).setWidth(rects.get(j).getWidth());
                j = j - 1;
            }
            rects.get(j + 1).setWidth(key);
        }

        for(int k = rects.size()-numToKeep-1; k>=0; k--){
            /*if(k<0){
                return rects;
            }*/
            rects.remove(k);
        }

        return rects;

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
            boolean tagFound = false;

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
