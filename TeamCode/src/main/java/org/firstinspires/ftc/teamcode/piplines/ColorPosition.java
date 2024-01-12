package org.firstinspires.ftc.teamcode.piplines;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagCanvasAnnotator;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class ColorPosition implements VisionProcessor {

    public static final String TAG = "AprilTagProcessorImpl";

    private Mat blue = new Mat();
    private final Object detectionsUpdateSync = new Object();
    private boolean drawAxes;
    private boolean drawCube;
    private boolean drawOutline;
    private boolean drawTagID;

    private Mat cameraMatrix;

    private double fx;
    private double fy;
    private double cx;
    private double cy;
    private float decimation;
    private boolean needToSetDecimation;
    private final Object decimationSync = new Object();

    private volatile AprilTagProcessor.PoseSolver poseSolver = AprilTagProcessor.PoseSolver.OPENCV_ITERATIVE;

    public ColorPosition(double fx, double fy, double cx, double cy, boolean drawAxes, boolean drawCube, boolean drawOutline, boolean drawTagID, int threads)
    {
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;

        this.drawAxes = drawAxes;
        this.drawCube = drawCube;
        this.drawOutline = drawOutline;
        this.drawTagID = drawTagID;

    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        fx = calibration.focalLengthX;
        fy = calibration.focalLengthY;
        cx = calibration.principalPointX;
        cy = calibration.principalPointY;

        constructMatrix();

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

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
}
