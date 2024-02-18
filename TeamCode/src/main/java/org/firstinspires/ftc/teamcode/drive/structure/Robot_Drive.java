package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.autonomus.DetectionTest;

public class Robot_Drive {

    Gamepad gamepad;

    ChasisInit csint;

    public double Limit = 1;

    public Robot_Drive(HardwareMap hwmap, Gamepad gamepad)
    {
        this.gamepad = gamepad;
        csint = new ChasisInit(hwmap);
    }

    public boolean Chose;
    public boolean Chose2;


    public void Run()
    {
        double rx = Range.clip(gamepad.right_stick_x * 1.1,-Limit,Limit);
        double y = -Range.clip(gamepad.left_stick_y,-Limit,Limit);
        double x = Range.clip(gamepad.left_stick_x,-Limit,Limit);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double Drive3 = (y + x - rx) / denominator;
        double Drive1 = (y - x - rx) / denominator;
        double Drive2 = (y - x + rx) / denominator;
        double Drive4 = (y + x + rx) / denominator;

        if(gamepad.options) {
            if(Chose)Limit=Limit+0.1;
            if(Limit>1)Limit=1;
            Chose = false;
        } else Chose=true;

        if(gamepad.share) {
            if(Chose2)Limit=Limit-0.1;
            if(Limit<0.1)Limit=0.1;
            Chose2 = false;
        } else {Chose2=true; }

        MS(Drive1, Drive2, Drive3, Drive4);
    }

    public void MS(double x1, double x2, double x3, double x4){
        csint.BackLeft.setPower(x2);
        csint.FrontRight.setPower(x1);
        csint.FrontLeft.setPower(x4);
        csint.BackRight.setPower(x3);
    }

    public void frontRightPow(double p) {
        csint.FrontRight.setPower(p);
    }
    public void backLeftPow(double p) {
        csint.BackLeft.setPower(p);
    }
    public void backRightPow(double p) {
        csint.BackRight.setPower(p);
    }
    public void frontLeftPow(double p) {
        csint.FrontLeft.setPower(p);
    }

}
