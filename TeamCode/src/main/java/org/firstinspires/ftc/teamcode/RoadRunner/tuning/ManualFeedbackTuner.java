package org.firstinspires.ftc.teamcode.RoadRunner.tuning;

import com.acmerobotics.roadrunner.Pose2d;
// TODO: remove Actions from the core module?
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
//import org.firstinspires.ftc.teamcode.TankDrive;

public final class ManualFeedbackTuner extends LinearOpMode {
    public static double DISTANCE = 66;

    @Override
    public void runOpMode() throws InterruptedException {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .lineToX(DISTANCE)
                                .lineToX(0)
                                .build());
            }
    }
}