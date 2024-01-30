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

package org.firstinspires.ftc.teamcode.DriverControl;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;

@TeleOp(name="Robert", group="Driver")

public class Robert extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    private LynxModule[] modules= null;

    MecanumDrive drive = null;
    private DcMotor armExtend1 = null;
    private DcMotor armExtend2 = null;
    private DcMotor armRaise = null;
    private CRServo hoist1 = null;
    private CRServo hoist2 = null;
    private CRServo hoist3 = null;
    private CRServo hoist4 = null;
    private Servo planeSwitch = null;
    private Servo clawFlip1 = null;
    private Servo clawFlip2 = null;
    private Servo clawTilt = null;
    private Servo claw1 = null;
    private Servo claw2 = null;
    private boolean planeToggle = true;
    private boolean claw1Toggle = false;
    private boolean claw2Toggle = false;
    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad currentGamepad2 = new Gamepad();
    private Gamepad previousGamepad1 = new Gamepad();
    private Gamepad previousGamepad2 = new Gamepad();
    private Boolean alreadyRumble = false;



    @Override
    public void runOpMode() {
        armExtend1 = hardwareMap.get(DcMotor.class, "armExtend1");
        armExtend2 = hardwareMap.get(DcMotor.class, "armExtend2");

        armExtend2.setDirection(DcMotorSimple.Direction.FORWARD);

        armRaise = hardwareMap.get(DcMotor.class, "armRaise");

        hoist1 = hardwareMap.get(CRServo.class, "hoist1");
        hoist2 = hardwareMap.get(CRServo.class, "hoist2");
        hoist3 = hardwareMap.get(CRServo.class, "hoist3");
        hoist4 = hardwareMap.get(CRServo.class, "hoist4");

        planeSwitch = hardwareMap.get(Servo.class, "planeSwitch");

        clawFlip1 = hardwareMap.get(Servo.class, "clawFlip1");
        clawFlip2 = hardwareMap.get(Servo.class, "clawFlip2");

        clawTilt = hardwareMap.get(Servo.class, "clawTilt");

        claw1 = hardwareMap.get(Servo.class, "claw1");
        claw2 = hardwareMap.get(Servo.class, "claw2");
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
        hoist4.setDirection(CRServo.Direction.REVERSE);

        planeSwitch.setPosition(.5);
        clawFlip1.setPosition(0.1);
        clawFlip2.setPosition(0.9);
        clawTilt.setPosition(0.3);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if(gamepad1.x){
                clawFlip1.setPosition(.95);
                clawFlip2.setPosition(.0494);
                clawTilt.setPosition(.65);
            } else if (gamepad1.a) {
                clawFlip1.setPosition(.875);
                clawFlip2.setPosition(.123);
                clawTilt.setPosition(.7478);
            }else if (gamepad1.b) {
                clawFlip1.setPosition(.75);
                clawFlip2.setPosition(.2488);
                clawTilt.setPosition(.7488);
            } else if (gamepad1.y){
                clawFlip1.setPosition(.70);
                clawFlip2.setPosition(.2994);
                clawTilt.setPosition(.7494);
            }else if (gamepad1.right_stick_button){
                clawFlip1.setPosition(.6494);
                clawFlip2.setPosition(.3494);
                clawTilt.setPosition(.6494);
            }
            if(currentGamepad2.y && !previousGamepad2.y) {
                planeToggle = !planeToggle;
            }
            if(currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
                claw1Toggle = !claw1Toggle;
            }
            if(currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
                claw2Toggle = !claw2Toggle;
            }
            if(currentGamepad1.dpad_left && !previousGamepad1.dpad_left){
                clawFlip1.setPosition(clawFlip1.getPosition() + 0.05);
                clawFlip2.setPosition(clawFlip2.getPosition() - 0.05);
            } else if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
                clawFlip1.setPosition(clawFlip1.getPosition() - 0.05);
                clawFlip2.setPosition(clawFlip2.getPosition() + 0.05);
            }
            if(currentGamepad1.dpad_up && !previousGamepad1.dpad_up){
                clawTilt.setPosition(clawTilt.getPosition() + 0.05);
            }else if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                clawTilt.setPosition(clawTilt.getPosition() - 0.05);
            }

            hoist1.setPower(gamepad2.left_stick_y);
            hoist2.setPower(gamepad2.right_stick_y);
            hoist3.setPower(gamepad2.left_stick_y);
            hoist4.setPower(gamepad2.right_stick_y);

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.

            double axial   = -gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double yaw     = -gamepad1.right_stick_x;

            if(runtime.time() > 120 && !alreadyRumble){
                gamepad1.rumble(2000);
                gamepad2.rumble(2000);
                alreadyRumble = true;
            }
            if(gamepad2.a){
                gamepad1.rumble(1000);
            }
            int extendPosition1 = armExtend1.getCurrentPosition();
            int extendPosition2 = armExtend2.getCurrentPosition();
            extendPosition2 = -extendPosition2;
            int raisePosition = armRaise.getCurrentPosition();

            if(planeToggle){
                planeSwitch.setPosition(.5);
            }
            else{
                planeSwitch.setPosition(.1);
            }
            if(claw1Toggle){
                claw1.setPosition(0.4);
            }
            else{
                claw1.setPosition(1);
            }
            if(claw2Toggle){
                claw2.setPosition(0.8);
            }
            else{
                claw2.setPosition(0);
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
            telemetry.addData("flip position1", clawFlip1.getPosition());
            telemetry.addData("flip position 2", clawFlip2.getPosition());
            telemetry.addData("tilt position", clawTilt.getPosition());
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }}
