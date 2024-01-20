/*package org.firstinspires.ftc.teamcode;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;

public class RRfolowingTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        int rotationalModifier = 0;
        int translationalModifier = 1;

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.dpad_up){ translationalModifier = 2;}
            else if (gamepad1.dpad_down) {translationalModifier = -1;}
            else{translationalModifier = 1;}
            if(gamepad1.right_bumper){ rotationalModifier = 45;}
            else if (gamepad1.left_bumper) {rotationalModifier = 90;}
            else{rotationalModifier = 0;}



            if(gamepad1.a) {
                drive.pose = new Pose2d(0,0,0);
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .splineTo(
                                        new Vector2d(
                                                24 * translationalModifier * cos(Math.toRadians(rotationalModifier)),
                                                24 * translationalModifier * sin(Math.toRadians(rotationalModifier))),
                                        0),
                        new TranslationalVelConstraint(5));
            }
            drive.updatePoseEstimate();

//            packet.fieldOverlay()
//                    .drawImage("/dash/ftc.jpg", (drive.pose.position.x), (drive.pose.position.y), 13, 13.25, drive.pose.heading.log(), 6.5, 6.625, false);
//            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", drive.pose.heading);
            telemetry.addData("Tick Par0", drive.rightFront.getCurrentPosition());
            telemetry.addData("Tick Par1", drive.leftBack.getCurrentPosition());
            telemetry.addData("Tick Perp", drive.leftFront.getCurrentPosition());
            telemetry.addData("in Par0", drive.rightFront.getCurrentPosition()*drive.PARAMS.inPerTick);
            telemetry.addData("in Par1", drive.leftBack.getCurrentPosition()*drive.PARAMS.inPerTick);
            telemetry.addData("in Perp", drive.leftFront.getCurrentPosition()*drive.PARAMS.lateralInPerTick);
            telemetry.update();
        }
    }
}

 */