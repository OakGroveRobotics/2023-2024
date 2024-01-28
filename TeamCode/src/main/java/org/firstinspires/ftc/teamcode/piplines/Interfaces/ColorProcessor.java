package org.firstinspires.ftc.teamcode.piplines.Interfaces;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.piplines.ColorProcessorImpl;
import org.firstinspires.ftc.vision.VisionProcessor;

public abstract class ColorProcessor implements VisionProcessor {
    public static final int THREADS_DEFAULT = 3;

    public static org.firstinspires.ftc.vision.apriltag.AprilTagProcessor easyCreateWithDefaults() {
        return new org.firstinspires.ftc.vision.apriltag.AprilTagProcessor.Builder().build();
    }

    public static class Builder {
        private double fx, fy, cx, cy;
        private DistanceUnit outputUnitsLength = DistanceUnit.INCH;
        private AngleUnit outputUnitsAngle = AngleUnit.DEGREES;
        private int threads = THREADS_DEFAULT;

        public ColorProcessor.Builder setLensIntrinsics(double fx, double fy, double cx, double cy) {
            this.fx = fx;
            this.fy = fy;
            this.cx = cx;
            this.cy = cy;
            return this;
        }

        public ColorProcessor.Builder setCamera(double fx, double fy, double cx, double cy) {
            this.fx = fx;
            this.fy = fy;
            this.cx = cx;
            this.cy = cy;
            return this;
        }

        public ColorProcessor.Builder setNumThreads(int threads) {
            this.threads = threads;
            return this;
        }

        public ColorProcessor build() {

            return new ColorProcessorImpl(
                    fx, fy, cx, cy, threads
            );
        }
    }

    /**
     * Set the detector decimation
     * <p>
     * Higher decimation increases frame rate at the expense of reduced range
     *
     * @param decimation detector decimation
     */
    public abstract void setDecimation(float decimation);

    public abstract ColorProcessorImpl.Selected getSelected();
    public abstract ColorProcessorImpl.Side getSide();
}

