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
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import java.util.ArrayList;
import java.util.List;
//Most imports are automatically handled by Android Studio as you program



public class Vision extends Subsystem {
    //NOTE: Can generate free access key from Vuforia website
    private final String VUFORIA_KEY =
            "Aba+gBH/////AAABma/0sYDZakYVhtjb1kH5oBVmYfYsDZXTuEZL9m7EdnFKZN/0v/LvE/Yr0NsXiJo0mJmznKAA5MK6ojvgtV1e1ODodBaMYZpgE1YeoAXYpvvPGEsdGv3xbvgKhvwOvqDToPe3x5w6gsq7a4Ullp76kLxRIoZAqaRpOuf1/tiJJQ7gTBFf8MKgbCDosmMDj7FOZsclk7kos4L46bLkVBcD9E0l7tNR0H0ShiOvxBwq5eDvzvmzsjeGc1aPgx9Br5AbUwN1T+BOvqwvZH2pM2HDbybgcWQJKH1YvXH4O62ENsYhD9ubvktayK8hSuu2CpUd1FVU3YQp91UrCvaKPYMiMFu7zeQCnoc7UOpG1P/kdFKP";
    private final float mmPerInch        = 25.4f; //Set Value for Vuf Tracking
    private final float mmTargetHeight   = 6 * mmPerInch; //Set Value for Vuf Tracking
    private final float halfField        = 72 * mmPerInch; //Set Value for Vuf Tracking
    private final float halfTile         = 12 * mmPerInch; //Set Value for Vuf Tracking
    private final float oneAndHalfTile   = 36 * mmPerInch; //Set Value for Vuf Tracking

    private OpenGLMatrix lastLocation   = null; //Necessary init for Vuf Tracking
    private VuforiaLocalizer vuforia    = null; //Necessary init for Vuf Init
    private VuforiaTrackables targets   = null; //Necessary init for Vuf Tracking
    public WebcamName webcamName; //Necessary init for Webcam

    public boolean targetVisible       = false; //Necessary init for for Vuf Tracking

    public VectorF translationG; //Necessary init for Vuf Tracking
    public Orientation rotationG; //Necessary init for Vuf Tracking

    private List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>(); //Necessary init for Vuf Tracking

    //NOTE: For more info on custom Tensorflow tracking, talk to Logan and he'll give more resources on it
    private final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/model_20220130_191400.tflite"; //Necessary init for Custom Tensorflow Detection

    private final String[] LABELS = { //Necessary init for Vuf Tracking
            "team_element"
    };
    private String labelName; //Necessary init for Tensorflow Tracking
    private int noLabel; //Necessary init for Tensorflow Tracking
    private TFObjectDetector tfod; //Necessary init for Tensorflow Init

    //NOTE: The following values are the actual readouts we access in other programs for Vuf Tracking
    public double VufXPos; //Necessary init for Vuf Tracking
    public double VufYPos; //Necessary init for Vuf Tracking
    public double VufHeading; //Necessary init for Vuf Tracking
    public boolean VufVisible; //Necessary init for Vuf Tracking

    //NOTE: The following values are the actual readouts we access in other programs for Tensorflow Tracking
    public double TFodLeft; //Necessary init for Tensorflow Tracking
    public double TFodRight; //Necessary init for Tensorflow Tracking

    //"Constructor" object for Vision
    public Vision(WebcamName webcameName, Telemetry telemetry, HardwareMap hardwareMap, ElapsedTime timer){
        super(telemetry, hardwareMap, timer);
        this.webcamName=webcameName;
    }

    //Initialize Vuforia Display and Detection Engine
    public void vuforiaInit(){

        webcamName = hardwareMap.get(WebcamName.class, "Webcam"); //Webcam map

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC screen);
         * If no camera-preview is desired, use the parameter-less constructor instead (commented out below).
         * Note: A preview window is required if you want to view the camera stream on the Driver Station Phone.
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        //Set Vuforia License Key Parameter
        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // We also indicate which camera we wish to use.
        parameters.cameraName = webcamName;

        //Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target. (Not recommended)
        parameters.useExtendedTracking = false;

        //Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        //Load the data sets for the trackable objects. These particular data
        //sets are stored in the 'assets' part of our application.
        targets = this.vuforia.loadTrackablesFromAsset("FreightFrenzy");

        //For convenience, gather together all the trackable objects in one easily-iterable collection
        allTrackables.addAll(targets);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Name and locate each trackable object
        identifyTarget(0, "Blue Storage",       -halfField,  oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "Blue Alliance Wall",  halfTile,   halfField,      mmTargetHeight, 90, 0, 0);
        identifyTarget(2, "Red Storage",        -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Red Alliance Wall",   halfTile,  -halfField,      mmTargetHeight, 90, 0, 180);

        /*
         * Create a transformation matrix describing where the camera is on the robot.
         *
         * Info:  The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * For a WebCam, the default starting orientation of the camera is looking UP (pointing in the Z direction),
         * with the wide (horizontal) axis of the camera aligned with the X axis, and
         * the Narrow (vertical) axis of the camera aligned with the Y axis
         *
         * But, this example assumes that the camera is actually facing forward out the front of the robot.
         * So, the "default" camera position requires two rotations to get it oriented correctly.
         * 1) First it must be rotated +90 degrees around the X axis to get it horizontal (its now facing out the right side of the robot)
         * 2) Next it must be be rotated +90 degrees (counter-clockwise) around the Z axis to face forward.
         *
         * Finally the camera can be translated to its actual mounting position on the robot.
         *      In this example, it is centered on the robot (left-to-right and front-to-back), and 6 inches above ground level.
         */

        final float CAMERA_FORWARD_DISPLACEMENT  = 0.0f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.9f * mmPerInch;   // eg: Camera is 6 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0.0f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens

        //Create Matrix system used for navigation target tracking
        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        /**  Let all the trackable listeners know where the camera is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
        }

        /*
         * WARNING:
         * In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
         * This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
         * CONSEQUENTLY do not put any driving commands in this loop.
         * To restore the normal opmode structure, just un-comment the following line:
         */

        // waitForStart();

        /* Note: To use the remote camera preview:
         * AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
         * Tap the preview window to receive a fresh image.
         * It is not permitted to transition to RUN while the camera preview window is active.
         * Either press STOP to exit the OpMode, or use the "options menu" again, and select "Camera Stream" to close the preview window.
         */

        targets.activate();
    }

    //NOTE: This is the function to Track Vuforia targets. While we don't currently use this,
    public void vuforiaTrack(){

        //Check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                //telemetry("Visible Target", trackable.getName());
                targetVisible = true;

                //getUpdatedRobotLocation() will return null if no new information is available since
                //the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        VufVisible=targetVisible;

        //Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            //Express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            translationG = translation;
            //telemetry.addData("Pos (inches)", "{X, Y, Z} = %.1f, %.1f, %.1f", translation.get(0) / mmPerInch + translation.get(1) / mmPerInch + translation.get(2) / mmPerInch);
            VufXPos = translation.get(0) / mmPerInch;
            VufYPos = translation.get(1) / mmPerInch;

            //Express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            rotationG = rotation;
            //telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            VufHeading = rotation.thirdAngle;
        }
        else {
            //telemetry.addData("Visible Target", "none");
        }
        //telemetry.update();
    }

    //Necessary provided Vuf function
    void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }

    //Initialize Tensorflow Detection Engine (WITH Confidence Value of Tracker)
    public void initTfod(double conf) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName()); //Use previously established monitoring setup through Vuf display
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId); //Use previously established monitoring setup through Vuf display
        tfodParameters.minResultConfidence = (float) conf; //Set confidence value (% of certainty that detected object is correct)
        tfodParameters.isModelTensorFlow2 = true; //Make sure version of trained model and engine lines up
        tfodParameters.inputSize = 320; //Default input size- matches with custom model & camera resolution
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia); //Create TFOD engine instance that runs through established Vuf display
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS); //Load custom file into engine
    }

    //Track Tensorflow Objects
    public void tfodTrack(){
        if (tfod != null) { //If tfod is properly initialized
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions(); //Get recognitions present within model
            if (updatedRecognitions != null) { //If there is ANY of these recognitions detected
                //telemetry.addData("# Object Detected", updatedRecognitions.size());
                noLabel = updatedRecognitions.size(); //Get size of recognized objects
                int i = 0;
                for (Recognition recognition : updatedRecognitions) { //For each recognized object
                    //telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    //telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            //recognition.getLeft(), recognition.getTop());
                    //telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            //recognition.getRight(), recognition.getBottom());
                    labelName = recognition.getLabel(); //Get what detected object is called

                    if(recognition.getLabel()=="team_element"){ //If recognition is the team element
                        TFodLeft=recognition.getLeft(); //Update left pixel value
                        TFodRight=recognition.getRight(); //Update right pixel value
                    }

                }
            }
        }
    }

}
