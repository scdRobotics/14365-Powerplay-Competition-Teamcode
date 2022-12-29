package org.firstinspires.ftc.teamcode;
//Package is a VERY important step! Required to do basically anything with the robot

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import java.util.ArrayList;
import java.util.List;
//Most imports are automatically handled by Android Studio as you program



public class Vision extends Subsystem {
    public OpenCvCamera webcam1;

    public OpenCvCamera webcam2;

    AprilTagPipeline aprilTagDetectionPipeline = new AprilTagPipeline(1, 578.272, 578.272, 402.145, 221.506);

    //"Constructor" object for Vision
    public Vision(OpenCvCamera webcam1, OpenCvCamera webcam2, Telemetry telemetry, HardwareMap hardwareMap, ElapsedTime timer){
        super(telemetry, hardwareMap, timer);
        this.webcam1=webcam1;
        this.webcam2=webcam2;


    }

    public void activateAprilTagPipelineCamera1(){
        webcam1.setPipeline(aprilTagDetectionPipeline);
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

    public int readAprilTagCamera1() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

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
