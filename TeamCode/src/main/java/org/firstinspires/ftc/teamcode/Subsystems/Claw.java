package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    ClawFlip clawFlip;
    ClawGrab clawGrab;
    public Servo clawTilt;

    public Claw(HardwareMap hardwareMap){

        clawFlip = new ClawFlip(hardwareMap);
        clawGrab = new ClawGrab(hardwareMap);
        clawTilt = hardwareMap.get(Servo.class, "clawTilt");

        init();
    }

    public class ClawFlip {
        private Servo flipper[];

        public ClawFlip(HardwareMap hardwareMap) {

            flipper = new Servo[2];

            flipper[0] = hardwareMap.get(Servo.class, "clawFlip1");
            flipper[1] = hardwareMap.get(Servo.class, "clawFlip2");
            flipper[1].setDirection(Servo.Direction.REVERSE);
        }

        public void setPosition(double position) {
            for (Servo claw : flipper) {
                claw.setPosition(position);
            }
        }
        public void goTopos1() {
            for (Servo claw : flipper) {
                claw.setPosition(.95);
            }
        }

        public void goTopos2() {
            for (Servo claw : flipper) {
                claw.setPosition(.875);
            }
        }

        public void goTopos3() {
            for (Servo claw : flipper) {
                claw.setPosition(.75);
            }
        }
        public void goTopos4() {
            for (Servo claw : flipper) {
                claw.setPosition(.70);
            }
        }
        public void goTopos5() {
            for (Servo claw : flipper) {
                claw.setPosition(.6494);
            }
        }

    }

    public class ClawGrab{

        public Servo claw[];

        public ClawGrab(HardwareMap hardwareMap){
            claw = new Servo[2];

            claw[0] = hardwareMap.get(Servo.class, "claw1");
            claw[1] = hardwareMap.get(Servo.class, "claw2");
        }

        public void clawRightOpen(){
            claw[1].setPosition(0.8);
        }

        public void clawLeftOpen(){
            claw[0].setPosition(0.4);
        }

        public void clawRightClose(){
            claw[1].setPosition(0);

        }

        public void clawLeftClose(){
            claw[0].setPosition(1);

        }

    }

    public void init() {
        clawFlip.setPosition(.1);
        clawTilt.setPosition(.3);
    }

    public void goToPos1(){
        clawFlip.goTopos1();
        clawTilt.setPosition(.65);
    }
    public void goToPos2(){
        clawFlip.goTopos2();
        clawTilt.setPosition(.7);
    }
    public void goToPos3(){
        clawFlip.goTopos3();
        clawTilt.setPosition(.7488);
    }
    public void goToPos4(){
        clawFlip.goTopos4();
        clawTilt.setPosition(.7494);
    }
    public void goToPos5(){
        clawFlip.goTopos5();
        clawTilt.setPosition(.6494);
    }

    public void clawRightOpen(){
        clawGrab.clawRightOpen();
    }
    public void clawLeftOpen(){
        clawGrab.clawLeftOpen();
    }
    public void clawRightClose(){
        clawGrab.clawRightClose();
    }
    public void clawLeftClose(){
        clawGrab.clawLeftClose();
    }



}
