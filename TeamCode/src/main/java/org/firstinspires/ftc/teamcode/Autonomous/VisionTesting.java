package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.piplines.ColorProcessorImpl;
import org.firstinspires.ftc.teamcode.piplines.Interfaces.ColorProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Objects;

@Config
@Autonomous

public class VisionTesting extends OpMode {

    private ColorProcessor itemFinder;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private ColorProcessorImpl.Side side;

    private Claw claw = null;
    private MecanumDrive drive;

    Action Left;
    Action Middle;
    Action Right;
    Action toBoard;

    int TARGET_TAG_ID;
    AprilTagDetection targetTag = null;
    ArrayList<AprilTagDetection> detections = null;

    double BACKBOARD_OFFSET = 6.0;


    public void init(){

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        claw = new Claw(hardwareMap);
        claw.clawLeftClose();
        claw.clawRightClose();

        Left = drive.actionBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(10,10),Math.toRadians(0))
                .build();
        Middle = drive.actionBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(19,0),Math.toRadians(0))
                .build();
        Right = drive.actionBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(10,-10),Math.toRadians(0))
                .build();

        toBoard = drive.actionBuilder(drive.pose)
                .turn(Math.toRadians(90))
                .strafeTo(new Vector2d(10,-40))
                .build();

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

        visionPortal = new VisionPortal.Builder()
        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
        .setCameraResolution(new Size(800, 600))
        .enableLiveView(true)
        .addProcessors(aprilTag, itemFinder)
        .build();
}

    public void init_loop() {
        while(itemFinder.getSelected() == ColorProcessorImpl.Selected.NONE){}
        telemetry.addData("Identified", itemFinder.getSelected());

    }

    public void start() {

        side = itemFinder.getSide();

        claw.goToFloor();

        if((itemFinder.getSelected() == ColorProcessorImpl.Selected.LEFT)){
            Actions.runBlocking(Left);
            if(side == ColorProcessorImpl.Side.BLUE){
                TARGET_TAG_ID = 1;
            } else{
                TARGET_TAG_ID = 4;
            }
        }
        else if ((itemFinder.getSelected() == ColorProcessorImpl.Selected.MIDDLE)) {
            Actions.runBlocking(Middle);
            if(side == ColorProcessorImpl.Side.BLUE){
                TARGET_TAG_ID = 2;
            } else{
                TARGET_TAG_ID = 5;
            }
        }
        else if ((itemFinder.getSelected() == ColorProcessorImpl.Selected.RIGHT)) {
            Actions.runBlocking(Right);
            if(side == ColorProcessorImpl.Side.BLUE){
                TARGET_TAG_ID = 3;
            } else{
                TARGET_TAG_ID = 6;
            }
        }

        claw.clawLeftOpen();

        claw.goToPos1();

        Actions.runBlocking(toBoard);

        while(Objects.isNull(targetTag)){
            detections = aprilTag.getFreshDetections();
            for (AprilTagDetection detection : detections) {
                if (Objects.nonNull(detection.metadata)) {
                    if(detection.id == TARGET_TAG_ID){
                        targetTag = detection;
                        break;
                    }
                }
            }
        }

        while (targetTag.ftcPose.y > 6) {
            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(
                                    Range.clip(targetTag.ftcPose.range, -0.5, 0.5),
                                    Range.clip(-targetTag.ftcPose.yaw, -0.5, 0.5))
                            , 0
                    )
            );
        }

    }

    public void loop() {


        telemetry.addData("Red", itemFinder.getRGB().val[0]);
        telemetry.addData("Green", itemFinder.getRGB().val[1]);
        telemetry.addData("Blue", itemFinder.getRGB().val[2]);
        telemetry.addData("?", itemFinder.getRGB().val[3]);
    }

}

