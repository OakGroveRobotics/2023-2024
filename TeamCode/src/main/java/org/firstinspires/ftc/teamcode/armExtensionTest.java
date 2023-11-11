
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;

@TeleOp(name="armExtensionTest", group="Linear Opmode")

public class armExtensionTest extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.

private DcMotor armExtend = null;
    @Override
    public void runOpMode() {
        armExtend = hardwareMap.get(DcMotor.class, "armExtend");
        armExtend.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
             armExtend.setPower(-gamepad1.left_stick_y / 2 );










        }
    }}
