/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;

@TeleOp(name="Robert", group="Linear Opmode")

public class Robert extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    private LynxModule[] modules= null;

    MecanumDrive drive = null;
    private DcMotor armExtend1 = null;
    private DcMotor armExtend2 = null;
    private DcMotor armRaise = null;
    private CRServo intake = null;
    private CRServo hoist1 = null;
    private CRServo hoist2 = null;
    private Servo pixelLatch = null;
    private Servo planeSwitch = null;
    private Servo clawFlip = null;
    private Servo clawTilt = null;
    private double flipPosition = 0;
    private double tiltPosition = 0;




    @Override
    public void runOpMode() {
        armExtend1 = hardwareMap.get(DcMotor.class, "armExtend1");
        armExtend2 = hardwareMap.get(DcMotor.class, "armExtend2");
        armExtend2.setDirection(DcMotorSimple.Direction.REVERSE);
        armRaise = hardwareMap.get(DcMotor.class, "armRaise");
        intake = hardwareMap.get(CRServo.class, "intake");
        hoist1 = hardwareMap.get(CRServo.class, "hoist1");
        hoist2 = hardwareMap.get(CRServo.class, "hoist2");
        pixelLatch = hardwareMap.get(Servo.class, "pixelLatch");
        planeSwitch = hardwareMap.get(Servo.class, "planeSwitch");
        clawFlip = hardwareMap.get(Servo.class, "clawFlip");
        clawTilt = hardwareMap.get(Servo.class, "clawTilt");
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)){
            telemetry.addData("Device Name", module.getDeviceName());
            telemetry.addData("Firmware Version", module.getFirmwareVersionString());
            sleep(1000);
        }
        DcMotor armExtend1 = hardwareMap.dcMotor.get("armExtend1");
        armExtend1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        armExtend1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done

        DcMotor armRaise = hardwareMap.dcMotor.get("armRaise");
        armRaise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRaise.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DcMotor armExtend2 = hardwareMap.dcMotor.get("armExtend2");
        armExtend2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtend2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hoist2.setDirection(CRServo.Direction.REVERSE);
        planeSwitch.setPosition(.21);
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.y){

            }


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     = gamepad1.right_stick_x;

            int extendPosition1 = armExtend1.getCurrentPosition();
            int extendPosition2 = armExtend2.getCurrentPosition();
            extendPosition2 = -extendPosition2;
            int raisePosition = armRaise.getCurrentPosition();

            hoist1.setPower(gamepad2.left_stick_y);
            hoist2.setPower(gamepad2.right_stick_y);
            if(gamepad1.dpad_right){
                flipPosition += .1;
            }
            else if(gamepad1.dpad_left){
                flipPosition -= .1;
            }
            clawFlip.setPosition(flipPosition);
            if(gamepad1.dpad_up){
                tiltPosition += .1;
            }
            else if(gamepad1.dpad_down){
                tiltPosition -= .1;
            }
            clawTilt.setPosition(tiltPosition);
            if(gamepad1.y){
                planeSwitch.setPosition(.54);
            }
            if(gamepad1.x){
                pixelLatch.setPosition(0);
            }
            else if(gamepad1.a){
                pixelLatch.setPosition(.255);
            }
            else if(gamepad1.b){
                pixelLatch.setPosition(.42);
            }
            if(gamepad1.right_bumper){
                intake.setPower(1);
            }
            else if(gamepad1.left_bumper){
                intake.setPower(0);
            }
            if(gamepad1.right_trigger > 0 && extendPosition1 < 2150){
                armExtend1.setPower(gamepad1.right_trigger);
            }
            else if(gamepad1.left_trigger > 0 && extendPosition1 > 150){
                armExtend1.setPower(-gamepad1.left_trigger);
            }
            else{
                armExtend1.setPower(0);
            }
            if(gamepad1.right_trigger > 0 && extendPosition2 < 2150){
                armExtend2.setPower(-gamepad1.right_trigger);
            }
            else if(gamepad1.left_trigger > 0 && extendPosition2 > 150){
                armExtend2.setPower(gamepad1.left_trigger);
            }
            else{
                armExtend2.setPower(0);
            }
            if(-gamepad1.right_stick_y > 0 && raisePosition < 1400){
                armRaise.setPower(-gamepad1.right_stick_y);
            }
            else if(-gamepad1.right_stick_y < 0 && raisePosition > 0){
                armRaise.setPower(-gamepad1.right_stick_y);
            }
            else{
                armRaise.setPower(0);
            }

            drive.setDrivePowers(
                    new PoseVelocity2d(
                        new Vector2d(
                                axial,
                                lateral)
                        , yaw
                    )
            );



            telemetry.addData("Extend Position 1", extendPosition1);
            telemetry.addData("Extend Position 2", extendPosition2);
            telemetry.addData("Arm Raise Position", raisePosition);
            telemetry.addData("Par0", drive.rightFront.getCurrentPosition());
            telemetry.addData("Par1", drive.leftBack.getCurrentPosition());
            telemetry.addData("perp", drive.leftFront.getCurrentPosition());
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }}
