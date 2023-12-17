package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@Config
@Autonomous
public class VisionTesting extends LinearOpMode {


    @Override
    public void runOpMode() {

        AprilTagDetection TagOfInterest = null;

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        FtcDashboard dashboard = FtcDashboard.getInstance();
        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(775.79f, 775.79f,400.898f, 300.79f)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setNumThreads(3)
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
        VisionPortal myVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(800, 600))
                .enableLiveView(true)
                .addProcessors(aprilTag)
                .build();


        ArrayList<AprilTagDetection> detections = null;
        detections = aprilTag.getDetections();
        waitForStart();

        while (opModeIsActive()) {

            detections = aprilTag.getDetections();

            for(AprilTagDetection detection :detections){
                if(detection.metadata != null){
                    TagOfInterest = detection;
                }
            }

            while(TagOfInterest.metadata != null) {
                drive.setDrivePowers(
                        new PoseVelocity2d(
                                new Vector2d(
                                        TagOfInterest.ftcPose.y,
                                        TagOfInterest.ftcPose.x)
                                , TagOfInterest.ftcPose.yaw
                        ));


                drive.updatePoseEstimate();

                TelemetryPacket packet2 = new TelemetryPacket();
                packet2.fieldOverlay()
                        .drawImage("/dash/ftc.jpg", (drive.pose.position.x), (drive.pose.position.y), 13, 13.25, drive.pose.heading.log(), 6.5, 6.625, false);
                dashboard.sendTelemetryPacket(packet2);

                aprilTag.getDetections();

                if (TagOfInterest.metadata != null) {
                    telemetry.addData("TagOfInterest ID", TagOfInterest.id);
                    telemetry.addData("TagOfInterest X offset", TagOfInterest.ftcPose.x);
                    telemetry.addData("TagOfInterest Y offset", TagOfInterest.ftcPose.y);
                    telemetry.addData("TagOfInterest yaw", TagOfInterest.ftcPose.yaw);

                }

                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading", drive.pose.heading);
                telemetry.update();
            }



        }
    }
}