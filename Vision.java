package org.firstinspires.ftc.teamcode;
//Package is a VERY important step! Required to do basically anything with the robot

import android.annotation.SuppressLint;
import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
//Most imports are automatically handled by Android Studio as you program


//TODO: Have build fix/replace broken webcam and undo camera pipeline switch hotfix
public class Vision extends Subsystem {
    public OpenCvCamera webcam1;
    public OpenCvCamera webcam2;

    AprilTagYellowPipeline aprilTagYellowPipeline = new AprilTagYellowPipeline(1, 578.272, 578.272, 402.145, 221.506);

    YellowPipeline yellowPipeline = new YellowPipeline();

    //public ArrayList<RectData> same = new ArrayList<RectData>(); //Only global because I want to access it in VisionTest.java; SHOULD CHANGE TO BE INSIDE FUNCTION findClosePoleDTheta() ONCE TESTING IS DONE !!

    //"Constructor" object for Vision
    public Vision(OpenCvCamera webcam1, OpenCvCamera webcam2, Telemetry telemetry, HardwareMap hardwareMap, ElapsedTime timer){
        super(telemetry, hardwareMap, timer);
        this.webcam1=webcam1;
        this.webcam2=webcam2;


    }

    public void activateYellowPipelineCamera1(){
        webcam1.setPipeline(yellowPipeline);
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

    public void activateAprilTagYellowPipelineCamera2(){
        webcam2.setPipeline(aprilTagYellowPipeline);
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

    @SuppressLint("NewApi")
    public double findClosePoleDTheta(){
        
        ArrayList<RectData> viewCam1 = yellowPipeline.getRects();
        //May need to adjust?
        ArrayList<RectData> viewCam2 = aprilTagYellowPipeline.getRects();

        if(viewCam1.isEmpty() || viewCam2.isEmpty()){
            return -1;
        }

        //viewCam1.sort(Comparator.comparing(RectData::getWidth));

        Collections.sort(viewCam1, new sortByWidth());
        Collections.sort(viewCam2, new sortByWidth());

        //viewCam2.sort(Comparator.comparing(RectData::getWidth));

        RectData widest1 = null;
        RectData widest2 = null;


        widest1 = viewCam1.get(viewCam1.size()-1);
        widest2 = viewCam2.get(viewCam2.size()-1);
        //telemetry.addData("Widest 1: ", widest1);
        //telemetry.addData("Widest 2: ", widest2);
        //same.add(widest1);
        //same.add(widest2);


        //else{
            //TODO: If same.size() is >2, isolate which equal rects within "same" are the pole we want to be looking at (AKA, the widest pair that still is a pole and not some strange background object/interference) so that same.size()==2
            //return -1;
        //}

        //TODO: Refine formula to ensure accurate dTheta

        double theta1 = Math.toRadians(142.5  - (widest1.getX()*5.5/128)); //Camera 1 Theta
        double theta2 = Math.toRadians(92.5 - (widest2.getX()*5.5/128)); //Camera 2 Theta

        double c1;
        double c2;
        double c3;
        double dx;
        double dy;
        double phi;
        double a = 6.825;
        double b = 1.1;

        c1 = theta1;
        c2 = theta2;

        dx = a * (Math.tan(c1) + Math.tan(c2))/(Math.tan(c1)-Math.tan(c2));
        dy = (2 * a) * (Math.tan(c1) * Math.tan(c2))/(Math.tan(c1)-Math.tan(c2)); //Test to see if it's 2*a or a

        c3 = Math.atan((dy - b)/dx);

        double dTheta = c3 - Math.PI/2;

        if(dTheta < -Math.PI/2) {
            dTheta = Math.PI+dTheta;
        }

        return dTheta;

    }

    @SuppressLint("NewApi")
    public double findClosePoleDist(){

        //same.clear();

        ArrayList<RectData> viewCam1 = yellowPipeline.getRects();
        //May need to adjust?
        ArrayList<RectData> viewCam2 = aprilTagYellowPipeline.getRects();

        if( (viewCam1.isEmpty() || viewCam2.isEmpty()) || (viewCam1==null || viewCam2==null) ){
            return -1;
        }

        //viewCam1.sort(Comparator.comparing(RectData::getWidth));

        Collections.sort(viewCam1, new sortByWidth());
        Collections.sort(viewCam2, new sortByWidth());

        //viewCam2.sort(Comparator.comparing(RectData::getWidth));



        RectData widest1 = null;
        RectData widest2 = null;

        widest1 = viewCam1.get(viewCam1.size()-1);
        widest2 = viewCam2.get(viewCam2.size()-1);
        //telemetry.addData("Widest 1: ", widest1);
        //telemetry.addData("Widest 2: ", widest2);


        //else{
        //TODO: If same.size() is >2, isolate which equal rects within "same" are the pole we want to be looking at (AKA, the widest pair that still is a pole and not some strange background object/interference) so that same.size()==2
        //return -1;
        //}

        //TODO: Refine formula to ensure accurate dTheta

        /*if(same.isEmpty()){
            return -1;
        }*/

        //TODO: Refine formula to ensure accurate dTheta
        double theta1 = Math.toRadians(142.5  - (widest1.getX()*5.5/128)); //Camera 1 Theta
        double theta2 = Math.toRadians(92.5 - (widest2.getX()*5.5/128)); //Camera 2 Theta

        double c1;
        double c2;
        double c3;
        double dx;
        double dy;
        //double phi;
        double a = 6.825;
        double b = 1.1;

        c1 = theta1;
        c2 = theta2;

        dx = a * (Math.tan(c1) + Math.tan(c2))/(Math.tan(c1)-Math.tan(c2));
        dy = (2 * a) * (Math.tan(c1) * Math.tan(c2))/(Math.tan(c1)-Math.tan(c2)); //Test to see if it's 2*a or a

        //c3 = Math.atan((dy - b)/dx);

        double dist = Math.sqrt(Math.pow(dy-b, 2) + Math.pow(dx, 2));

        return dist;

    }




    public int readAprilTagCamera2() {
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

class sortByWidth implements Comparator<RectData>{
    public int compare(RectData a, RectData b){
        if(a!=null && b!= null){
            return (int) (a.getWidth() - b.getWidth()); //i don't like this cast here tbh but what else can we do?
        }
        return -1;
    }
}