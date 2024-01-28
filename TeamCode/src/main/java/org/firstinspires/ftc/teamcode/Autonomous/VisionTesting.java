package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.piplines.ColorProcessorImpl;
import org.firstinspires.ftc.teamcode.piplines.Interfaces.ColorProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@Config
@Autonomous

public class VisionTesting extends OpMode {

    private ColorProcessor itemFinder;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public void init(){
        AprilTagDetection TagOfInterest = null;
        ArrayList<AprilTagDetection> detections = null;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        aprilTag = new AprilTagProcessor.Builder()
        .setLensIntrinsics(775.79f, 775.79f,400.898f, 300.79f)
        .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
        .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
        .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
        .setNumThreads(3)
        .setDrawAxes(true)
        .setDrawTagID(true)
        .setDrawTagOutline(true)
        .build();

        itemFinder = new ColorProcessor.Builder()
        .setLensIntrinsics(775.79f, 775.79f,400.898f, 300.79f)
        .setNumThreads(3)
        .build();


        VisionPortal myVisionPortal = new VisionPortal.Builder()
        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
        .setCameraResolution(new Size(800, 600))
        .enableLiveView(true)
        .addProcessors(aprilTag, itemFinder)
        .build();

        TelemetryPacket packet2 = new TelemetryPacket();
}

    public void init_loop() {
    }

    public void start() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        while(itemFinder.getSelected() == ColorProcessorImpl.Selected.NONE){}

        if((itemFinder.getSelected() == ColorProcessorImpl.Selected.LEFT)){
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .splineTo(new Vector2d(-6, 0), 0)
                            .build());
        }
        else if ((itemFinder.getSelected() == ColorProcessorImpl.Selected.MIDDLE)) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .splineTo(new Vector2d(6, 0),0)
                            .build());
        }
        else if ((itemFinder.getSelected() == ColorProcessorImpl.Selected.RIGHT)) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .splineTo(new Vector2d(0, 6), 0)
                            .build());
        }
    }

    public void loop() {

        telemetry.addData("Identified", itemFinder.getSelected());
    }

}

