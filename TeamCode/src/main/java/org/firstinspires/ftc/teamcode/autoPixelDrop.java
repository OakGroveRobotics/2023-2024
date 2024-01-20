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
import com.qualcomm.robotcore.hardware.Servo;

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
@Autonomous(name="RoadRunnerTest", group="Linear Opmode")
public class autoPixelDrop extends LinearOpMode {
    private Servo clawFlip1 = null;
    private Servo clawFlip2 = null;
    private Servo clawTilt = null;
    private Servo claw1 = null;
    private Servo claw2 = null;

    @Override
    public void runOpMode() {

        AprilTagDetection TagOfInterest = null;
        clawFlip1 = hardwareMap.get(Servo.class, "clawFlip1");
        clawFlip2 = hardwareMap.get(Servo.class, "clawFlip2");
        clawTilt = hardwareMap.get(Servo.class, "clawTilt");
        claw1 = hardwareMap.get(Servo.class, "claw1");
        claw2 = hardwareMap.get(Servo.class, "claw2");
        claw1.setPosition(1);
        claw2.setPosition(0);
        clawFlip1.setPosition(0.1);
        clawFlip2.setPosition(0.9);
        clawTilt.setPosition(0.3);

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
        clawFlip1.setPosition(.875);
        clawFlip2.setPosition(.123);
        clawTilt.setPosition(.7478);
        sleep(500);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .lineToX(-12)
                        .build());
        sleep(500);
        claw1.setPosition(0.4);
        sleep(500);
        clawFlip1.setPosition(0.1);
        clawFlip2.setPosition(0.9);
        clawTilt.setPosition(0.3);

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


