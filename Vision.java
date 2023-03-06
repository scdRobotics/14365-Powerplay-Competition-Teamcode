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

    public void pauseCamera(){
        //yellowPipeline.setPause(true);
        //aprilTagYellowPipeline.setPause(true);
    }

    public void resumeCamera(){
        //yellowPipeline.setPause(false);
        //aprilTagYellowPipeline.setPause(false);
    }

    @SuppressLint("NewApi")
    public double findClosePoleDTheta(){

        double cam1CurrentX = yellowPipeline.getCurrentCenterX();
        double cam2CurrentX = aprilTagYellowPipeline.getCurrentCenterX();

        if(cam1CurrentX==-1 || cam2CurrentX==-1){
            return -1;
        }

        double theta1 = Math.toRadians(142.5 - (cam1CurrentX*5.5/128)); //Camera 1 Theta
        double theta2 = Math.toRadians(92.5 - (cam2CurrentX*5.5/128)); //Camera 2 Theta

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

        c3 = Math.atan((dy - b)/dx);

        double dTheta = c3 - Math.PI/2;

        if(dTheta < -Math.PI/2) {
            dTheta = Math.PI+dTheta;
        }

        return dTheta;

    }

    @SuppressLint("NewApi")
    public double findClosePoleDist(){

        double cam1CurrentX = yellowPipeline.getCurrentCenterX();
        double cam2CurrentX = aprilTagYellowPipeline.getCurrentCenterX();

        if(cam1CurrentX==-1 || cam2CurrentX==-1){
            return -1;
        }

        double theta1 = Math.toRadians(142.5 - (cam1CurrentX*5.5/128)); //Camera 1 Theta
        double theta2 = Math.toRadians(92.5 - (cam2CurrentX*5.5/128)); //Camera 2 Theta

        double c1;
        double c2;
        //double c3;
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