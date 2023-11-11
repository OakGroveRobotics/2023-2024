package org.firstinspires.ftc.teamcode.RoadRunner.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;

public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(72, 72, 0));
            FtcDashboard dashboard = FtcDashboard.getInstance();

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                gamepad1.left_stick_y,
                                gamepad1.left_stick_x
                        ),
                        gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay()
                    .drawImage("/dash/ftc.jpg", drive.pose.position.x, drive.pose.position.y, 20, 20, Math.toRadians(90), 10, 10, false);
                dashboard.sendTelemetryPacket(packet);




                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading", drive.pose.heading);
                telemetry.update();
            }
    }
}