package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.BufferedReader;
import java.io.FileReader;

/**
 * In this sample, we demonstrate how to use EasyOpenCV in
 * Vuforia passthrough mode. In this mode, EasyOpenCV does not
 * take direct control over the camera. Instead, it pulls frames
 * out of a VuforiaLocalizer's frame queue. This allows you to
 * run both OpenCV and Vuforia simultaneously on the same camera.
 * The downside is that you do not get to choose the resolution
 * of frames delivered to your pipeline, and you do not get any
 * sort of manual control over sensor parameters such as exposure,
 * gain, ISO, or frame rate.
 */
@TeleOp
public class OpenCvAndVuforiaOnSameCameraExample extends LinearOpMode
{
    private static final String VUFORIA_LICENSE_FILE = "/sdcard/data/vuforia-license.txt";

    VuforiaLocalizer vuforia = null;
    OpenCvCamera openCvPassthrough;

    @Override
    public void runOpMode()
    {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCV,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);

        /*
         * Setup Vuforia
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(viewportContainerIds[0]);
        // Find license key in local storage
        try (BufferedReader br = new BufferedReader(new FileReader(VUFORIA_LICENSE_FILE))) {
            parameters.vuforiaLicenseKey = br.readLine();
        } catch (Exception e) {
            throw new IllegalStateException("Vuforia key NOT found", e);
        }

        parameters.cameraDirection   = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Create a Vuforia passthrough "virtual camera"
        openCvPassthrough = OpenCvCameraFactory.getInstance().createVuforiaPassthrough(vuforia, parameters, viewportContainerIds[1]);

        openCvPassthrough.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Using GPU acceleration can be particularly helpful when using Vuforia passthrough
                // mode, because Vuforia often chooses high resolutions (such as 720p) which can be
                // very CPU-taxing to rotate in software. GPU acceleration has been observed to cause
                // issues on some devices, though, so if you experience issues you may wish to disable it.
                openCvPassthrough.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                openCvPassthrough.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
                openCvPassthrough.setPipeline(new UselessColorBoxDrawingPipeline(new Scalar(255,0,0,255)));

                // We don't get to choose resolution, unfortunately. The width and height parameters
                // are entirely ignored when using Vuforia passthrough mode. However, they are left
                // in the method signature to provide interface compatibility with the other types
                // of cameras.
                openCvPassthrough.startStreaming(0,0, OpenCvCameraRotation.UPRIGHT);
            }
        });

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Passthrough FPS", openCvPassthrough.getFps());
            telemetry.addData("Frame count", openCvPassthrough.getFrameCount());
            telemetry.update();

            sleep(100);
        }
    }

    class UselessColorBoxDrawingPipeline extends OpenCvPipeline
    {
        Scalar color;

        UselessColorBoxDrawingPipeline(Scalar color)
        {
            this.color = color;
        }

        @Override
        public Mat processFrame(Mat input)
        {
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()/4,
                            input.rows()/4),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    color, 4);

            return input;
        }
    }
}
