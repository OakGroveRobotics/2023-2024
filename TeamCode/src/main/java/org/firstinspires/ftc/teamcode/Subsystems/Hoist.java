package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

public class Hoist {

    public CRServo[] hoist;

    public Hoist(HardwareMap hardwareMap){
        hoist[0] = hardwareMap.get(CRServo.class, "hoist1");
        hoist[1] = hardwareMap.get(CRServo.class, "hoist2");
        hoist[2] = hardwareMap.get(CRServo.class, "hoist3");
        hoist[3] = hardwareMap.get(CRServo.class, "hoist4");
    }

    public void run(double powerLeft, double powerRight){
        hoist[0].setPower(powerLeft);
        hoist[1].setPower(powerRight);
        hoist[2].setPower(powerLeft);
        hoist[3].setPower(powerRight);
    }


}
