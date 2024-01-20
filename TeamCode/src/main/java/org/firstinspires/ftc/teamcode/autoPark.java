package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@Config
@Autonomous(name="autoPark", group="Linear Opmode")
public class autoPark extends LinearOpMode {


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

//        ColorPosition colorPosition = new ColorPosition(775.79f, 775.79f,400.898f, 300.79f);

        VisionPortal myVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(800, 600))
                .enableLiveView(true)
                .addProcessors(aprilTag)
                .build();

        TelemetryPacket packet2 = new TelemetryPacket();

        ArrayList<AprilTagDetection> detections = null;

        waitForStart();
        telemetry.addData("here",drive.pose.position.x);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .lineToX(48)
                        .build());

        while (opModeIsActive()) {

                drive.updatePoseEstimate();

                packet2.fieldOverlay()
                        .drawImage("/dash/ftc.jpg", (drive.pose.position.x), (drive.pose.position.y), 13, 13.25, drive.pose.heading.log(), 6.5, 6.625, false);
                dashboard.sendTelemetryPacket(packet2);

                    telemetry.addData("x", drive.pose.position.x);
                    telemetry.addData("y", drive.pose.position.y);
                    telemetry.addData("heading", drive.pose.heading);
                    telemetry.addData("Par0", drive.rightFront.getCurrentPosition());
                    telemetry.addData("Par1", drive.leftBack.getCurrentPosition());
                    telemetry.addData("Perp", drive.leftFront.getCurrentPosition());
                    telemetry.addData("Par0In", drive.rightFront.getCurrentPosition() * 0.0005347752);
                    telemetry.addData("Per1In", drive.leftBack.getCurrentPosition()* 0.0005347752);
                    telemetry.addData("PerpIn", drive.leftFront.getCurrentPosition()* -0.00039120696508401873);
                    telemetry.update();


            }




        }
    }


