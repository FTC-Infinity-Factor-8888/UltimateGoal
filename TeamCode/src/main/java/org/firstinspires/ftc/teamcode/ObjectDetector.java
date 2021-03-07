/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
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
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

public class ObjectDetector {
    private static final String VUFORIA_LICENSE_FILE = "/sdcard/data/vuforia-license.txt";
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    public static final String QUAD = "Quad";
    public static final String SINGLE = "Single";

    // Constants
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch; // The height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    private String licenseKey;
    private HardwareMap hardwareMap;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    private VuforiaLocalizer.Parameters parameters;

    // This is the navigation target trackables
    private VuforiaTrackables targetsUltimateGoal;

    private List<VuforiaTrackable> allTrackables = new ArrayList<>();

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    /**
     * {@link #openCvPassthrough} is the variable we will use to store our instance of a virtual
     * camera that we will use with OpenCV.
     */
    private OpenCvCamera openCvPassthrough;

    private SingleRingDeterminationPipeline pipeline;

    // Phone orientation for navigation.
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    //empty constructor
    public void ObjectDetector() {
    }

    public void init(Robot robot) {
        System.out.println("Initializing ObjectDetector");

        hardwareMap = robot.getHardwareMap();

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initNavigation();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0 / 9.0);
            tfod.setClippingMargins(10, 200, 10, 0);
        } else {
            throw new IllegalStateException("tfod did NOT start");
        }
    }

    public List<Recognition> getUpdatedRecognitions() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            return tfod.getUpdatedRecognitions();
        }
        return null;
    }

    public void stopTfod() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }
    public void stopNavigation() {
        if (targetsUltimateGoal != null) {
            // Disable Tracking when we are done;
            targetsUltimateGoal.deactivate();
        }
    }

    /**
     * Tells you which box sees a ring
     * Or if no ring is seen
     *
     * @return 1 if ring is in box one, 2 if box is in box 2, 0 if there is no ring visible
     */
    public int whichBoxSeen() {
        return pipeline.boxSeen;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        // Find license key in local storage
        try (BufferedReader br = new BufferedReader(new FileReader(VUFORIA_LICENSE_FILE))) {
            licenseKey = br.readLine();
        } catch (Exception e) {
            throw new IllegalStateException("Vuforia key NOT found", e);
        }

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        parameters = new VuforiaLocalizer.Parameters(viewportContainerIds[0]);
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = licenseKey;
        parameters.cameraDirection   = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.

        // Create a Vuforia passthrough virtual camera.
        openCvPassthrough = OpenCvCameraFactory.getInstance().createVuforiaPassthrough(vuforia, parameters, viewportContainerIds[1]);

        System.out.println("Done with Vuforia initialization");
    }

    private void initNavigation() {
        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsUltimateGoal = vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection

        allTrackables.addAll(targetsUltimateGoal);

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

        //Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        //TODO: Replace these numbers with actual numbers measured from robot.

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 11.25f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
        targetsUltimateGoal.activate();
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        //tfodParameters.useObjectTracker = false;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, QUAD, SINGLE);
    }

    /**
     * Initializes OpenCv using the split viewport from Vuforia
     */
    public void initOpenCv() {
        stopTfod();
        System.out.println("Starting OpenCV pipeline");
        openCvPassthrough.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                pipeline = new SingleRingDeterminationPipeline();
                // Using GPU acceleration can be particularly helpful when using Vuforia passthrough
                // mode, because Vuforia often chooses high resolutions (such as 720p) which can be
                // very CPU-taxing to rotate in software. GPU acceleration has been observed to cause
                // issues on some devices, though, so if you experience issues you may wish to disable it.
                openCvPassthrough.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                //openCvPassthrough.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
                openCvPassthrough.setPipeline(pipeline);

                // We don't get to choose resolution, unfortunately. The width and height parameters
                // are entirely ignored when using Vuforia passthrough mode. However, they are left
                // in the method signature to provide interface compatibility with the other types
                // of cameras.
                openCvPassthrough.startStreaming(0, 0, OpenCvCameraRotation.UPRIGHT);
            }
        });
        int frameCount = 0;
        do {
            try{
                Thread.sleep(35);
            }
            catch(InterruptedException e){
                //Do nothing. Wake up.
            }
        } while(getInitialized() == false && frameCount++ < 30);
        System.out.println("Done initializing, frameCount = " + frameCount);
    }

    /**
     * Getting the most recent information about the robots rotation and location.
     * If there is no updated information, or if no targets are visible, then nothing will be returned.
     * @return List of orientation and translation of the robot, as calculated by each visible target,
     *      or null if no information is known.
     */
    public List<NavigationInfo> getNavigationInfo() {
        List<NavigationInfo> allVisibleTargets = new ArrayList<>();
        // check all the trackable targets to see which one (if any) is visible.
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable
                        .getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    NavigationInfo targetInfo = new NavigationInfo();
                    targetInfo.targetName = trackable.getName();

                    // express position (translation) of robot in inches.
                    VectorF translation = robotLocationTransform.getTranslation();
                    VectorF positionInInches = new VectorF(new float[]{translation.get(0) / mmPerInch,
                            translation.get(1) / mmPerInch, translation.get(2) / mmPerInch});
                    targetInfo.translation = positionInInches;

                    // express the rotation of the robot in degrees.
                    targetInfo.rotation = Orientation.getOrientation(robotLocationTransform, EXTRINSIC, XYZ, DEGREES);

                    allVisibleTargets.add(targetInfo);
                }
                break;
            }
        }
        if (allVisibleTargets.size() == 0) {
            return null;
        }
        else {
            return allVisibleTargets;
        }
    }

    public static class SingleRingDeterminationPipeline extends OpenCvPipeline {
        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        // view from startLine1
        // topLeft = (1500, 730)
        // width = 320
        // height = 250

        /*
         * we have determined the actual screen size to be 800x * 448y
         * 29-30 fps
         * We want to look in either the lower-left-corner, or the lower-right-corner for the ring(s)
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(420,730);

        static final int REGION_WIDTH = 320;
        static final int REGION_HEIGHT = 250;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 130;

        /*Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(

        );*/
        Point box1_pointA;
        Point box1_pointB;
        Point box2_pointA;
        Point box2_pointB;

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat region2_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();

        private volatile boolean initialized = false;
        private volatile int avg1 = 0;
        private volatile int avg2 = 0;

        // Volatile since accessed by OpMode thread w/o synchronization
        //private volatile boolean isRingSeen = false;
        private volatile int boxSeen = 0; //0 means no rings seen

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            int width = firstFrame.cols();
            int height = firstFrame.rows();
            int margin = width / 40;
            int halfWidth = width / 2;
            int halfHeight = height / 2;
            int quarterHeight = halfHeight / 2;

            box1_pointA = new Point(
                    margin * 6, halfHeight + quarterHeight);
            box1_pointB = new Point(
                    halfWidth - margin * 2, height - margin * 2);
            box2_pointA = new Point(
                    halfWidth + margin * 8, halfHeight + quarterHeight);
            box2_pointB = new Point(
                    width, height - margin);

            System.out.println(firstFrame.toString());
            System.out.println(String.format("box1 (%f, %f) - (%f, %f)", box1_pointA.x, box1_pointA.y, box1_pointB.x, box1_pointB.y));
            System.out.println(String.format("box2 (%f, %f) - (%f, %f)", box2_pointA.x, box2_pointA.y, box2_pointB.x, box2_pointB.y));
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(box1_pointA, box1_pointB));
            region2_Cb = Cb.submat(new Rect(box2_pointA, box2_pointB));

        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            initialized = true;

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    box1_pointA, // First point which defines the rectangle
                    box1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    4); // Thickness of the rectangle lines
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    box2_pointA, // First point which defines the rectangle
                    box2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    4); // Thickness of the rectangle lines

           if (avg1 >= ONE_RING_THRESHOLD && avg1 < FOUR_RING_THRESHOLD) {
               boxSeen = 1;
           }
           else if (avg2 >= ONE_RING_THRESHOLD && avg2 < FOUR_RING_THRESHOLD){
               boxSeen = 2;
        }
           else {
               boxSeen = 0;
           }

            /*Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
            */
            return input;
        }
    }
    public boolean getInitialized(){
        return pipeline.initialized;
    }

    public int getAvg1(){
        return pipeline.avg1;
    }

    public int getAvg2(){
        return pipeline.avg2;
    }
}
