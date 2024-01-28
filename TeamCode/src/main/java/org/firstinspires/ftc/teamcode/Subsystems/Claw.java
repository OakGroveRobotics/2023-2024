package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    ClawFlip clawFlip;
    public Servo clawTilt;

    public Servo claw[];

    public Claw(HardwareMap hardwareMap){

        ClawFlip clawFlip = new ClawFlip(hardwareMap);

        clawTilt = hardwareMap.get(Servo.class, "clawTilt");

        claw[0] = hardwareMap.get(Servo.class, "claw1");
        claw[1] = hardwareMap.get(Servo.class, "claw2");
    }

    public class ClawFlip {
        private Servo flipper[];

        public ClawFlip(HardwareMap hardwareMap) {
            flipper[0] = hardwareMap.get(Servo.class, "clawFlip1");
            flipper[1] = hardwareMap.get(Servo.class, "clawFlip2");
            flipper[1].setDirection(Servo.Direction.REVERSE);
        }

        public void setPosition(double position) {
            for (Servo claw : flipper) {
                claw.setPosition(position);
            }

        }
    }

    public void init() {
        clawFlip.setPosition(.1);
        clawTilt.setPosition(.3);
    }




}
