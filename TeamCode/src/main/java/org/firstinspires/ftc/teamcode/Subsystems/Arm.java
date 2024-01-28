package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    private DcMotor armRaise;
    public ArmExtend armExtend;

    public Arm(HardwareMap hardwareMap){
        armExtend = new ArmExtend(hardwareMap);
        armRaise = hardwareMap.get(DcMotor.class, "armRaise");

    }

    public static class ArmExtend{
        private DcMotor extenders[];
        public ArmExtend(HardwareMap hardwareMap){
            extenders[0] = hardwareMap.get(DcMotor.class, "armExtend1");
            extenders[1] = hardwareMap.get(DcMotor.class, "armExtend2");
        }

        public void run(double power){
            for(DcMotor extender : extenders){
                extender.setPower(power);
            }
        }

    }

    public void runArmRaise(double power){
        armRaise.setPower(power);
    }



}
