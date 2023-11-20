package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
@TeleOp(name="autoTest", group="Linear Opmode")
public class autoTest<Drive> extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet1 = new TelemetryPacket();
        dashboard.sendTelemetryPacket(packet1);
 drive.updatePoseEstimate();

    TelemetryPacket packet2 = new TelemetryPacket();
                packet2.fieldOverlay()
                        .drawImage("/dash/ftc.jpg", drive.pose.position.y, drive.pose.position.x, 13, 13.25, drive.pose.heading.log(), 6.625, 6.5, false);
                dashboard.sendTelemetryPacket(packet2);

        while (opModeIsActive()) {
            drive.moveToPoint(5, 0).runBlocking();
        }

    }


    }



