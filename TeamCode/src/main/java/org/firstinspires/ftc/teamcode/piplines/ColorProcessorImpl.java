package org.firstinspires.ftc.teamcode.piplines;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.piplines.Interfaces.ColorProcessor;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public class ColorProcessorImpl extends ColorProcessor {

    private Mat colorProcessor = new Mat();
    private Mat cameraMatrix;

    private double fx;
    private double fy;
    private double cx;
    private double cy;
    private float decimation;
    private boolean needToSetDecimation;
    private final Object decimationSync = new Object();

    public ColorProcessorImpl(double fx, double fy, double cx, double cy, int threads)
    {
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;

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

        Imgproc.cvtColor(frame, colorProcessor, Imgproc.COLOR_RGBA2GRAY);
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

    @Override
    public void setDecimation(float decimation) {

    }
}
