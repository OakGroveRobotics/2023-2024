/* Copyright (c) 2017 FIRST. All rights reserved.
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

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="autoTest", group="Linear OpMode")

public class autoTest extends LinearOpMode {

    private Servo clawFlip = hardwareMap.get(Servo.class, "clawFlip");
    private Servo clawTilt = hardwareMap.get(Servo.class, "clawTilt");
    private Servo pixelLatch = hardwareMap.get(Servo.class, "pixelLatch");

    public Action dropPixel(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                packet.put("drive X pos", drive.pose.position.x);
                packet.put("drive Y pos", drive.pose.position.y);
                clawFlip.setPosition(.45); //adjust
                sleep(1000); //time between actions
                clawTilt.setPosition(.83);//adjust
                sleep(1000); //time between actions
                pixelLatch.setPosition(.42); //adjust
                return true;
            }
        };
    }

  MecanumDrive drive = null;
  double axial = 0;
  double lateral = 0;
  double yaw = 0;



    @Override
    public void runOpMode() {

        double spikeMarkX = 24;
        double parkingMarkX = 24;
        double parkingMarky = 24;


        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        waitForStart();
        while (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .splineTo(new Vector2d(spikeMarkX, 0), 0)
                            .afterDisp(spikeMarkX, dropPixel())
                            .splineTo(new Vector2d(parkingMarkX, parkingMarky), 0)
                            .build());

            telemetry.update();
        }
    }
}
