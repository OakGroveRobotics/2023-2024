package org.firstinspires.ftc.teamcode.piplines;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;


import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.piplines.Interfaces.ColorProcessor;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class ColorProcessorImpl extends ColorProcessor {

    Mat submat = new Mat();
    Mat hsvMat = new Mat();
    private Mat cameraMatrix;

    private double fx;
    private double fy;
    private double cx;
    private double cy;
    private float decimation;
    private boolean needToSetDecimation;
    private final Object decimationSync = new Object();

    public Rect rectLeft = new Rect(10, 42, 200, 400);
    public Rect rectMiddle = new Rect(220, 42, 200, 400);
    public Rect rectRight = new Rect(430, 42, 200, 400);
    Selected selection = Selected.NONE;


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

        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        double satRectLeft = getAvgSaturation(hsvMat, rectLeft);
        double satRectMiddle = getAvgSaturation(hsvMat, rectMiddle);
        double satRectRight = getAvgSaturation(hsvMat, rectRight);

        if ((satRectLeft > satRectMiddle) && (satRectLeft > satRectRight)) {
             return Selected.LEFT;
            } else if ((satRectMiddle > satRectLeft) && (satRectMiddle > satRectRight)) {
            return Selected.MIDDLE;
            }
        return Selected.RIGHT;
    }
    protected double getAvgSaturation(Mat input, Rect rect) {
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[1];
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

        @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

            Paint selectedPaint = new Paint();
            selectedPaint.setColor(Color.RED);
            selectedPaint.setStyle(Paint.Style.STROKE);
            selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

            Paint nonSelectedPaint = new Paint(selectedPaint);
            nonSelectedPaint.setColor(Color.GREEN);

            android.graphics.Rect drawRectangleLeft = makeGraphicsRect(rectLeft, scaleBmpPxToCanvasPx);
            android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx);
            android.graphics.Rect drawRectangleRight = makeGraphicsRect(rectRight, scaleBmpPxToCanvasPx);

        switch (selection) {
             case LEFT:
                 canvas.drawRect(drawRectangleLeft, selectedPaint);
                 canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                 canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                 break;
             case MIDDLE:
                 canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                 canvas.drawRect(drawRectangleMiddle, selectedPaint);
                 canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                 break;
             case RIGHT:
                 canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                 canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                 canvas.drawRect(drawRectangleRight, selectedPaint);
                 break;
             case NONE:
                 canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                 canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                 canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                 break;
             }
         }

     public enum Selected {
         NONE,
         LEFT,
         MIDDLE,
         RIGHT
     }

     public Selected getSelected(){
        return selection;
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
