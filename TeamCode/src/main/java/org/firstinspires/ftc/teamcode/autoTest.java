package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
public class autoTest {
    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    FtcDashboard dashboard = FtcDashboard.getInstance();



    drive.updatePoseEstimate();

    TelemetryPacket packet2 = new TelemetryPacket();
    packet2.fieldOverlay()
            .drawImage("/dash/ftc.jpg", drive.pose.position.y, drive.pose.position.x, 13, 13.25, drive.pose.heading.log(), 6.625, 6.5, false);
    dashboard.sendTelemetryPacket(packet2);
}
