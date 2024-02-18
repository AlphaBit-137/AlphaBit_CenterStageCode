package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Motor_Skeleton;

public class Arm {

    public DcMotorEx arm1;

    public Motor_Skeleton ArmMotor1 = new Motor_Skeleton(arm1);

    Gamepad Arm_Gamepad;

    double Kp = 0.002;
    double Ki = 0.000045;
    double Kd = 0.00085;

    double max_accel = 250;
    double max_vel = 250;

    double max_output = 0.05;

    public double pidPower=0;

    public double Reeference = 0;

    ElapsedTime timer = new ElapsedTime();

    ElapsedTime timer2 = new ElapsedTime();

    public double savedPower = 0;
    boolean firstTime = true;

    public double pow=0.2;

    double lastPosition;
    boolean toggle1=true;
    boolean toggle2=true;

    double max_pos = 1000;
    double min_pos = 100;
    double up_pos = 772;
    double down_pos = 10;
    static double encoder_position;
    int pos=1;
    boolean gpad=false;

    public Arm(HardwareMap ahwMap, Gamepad Arm_Gamepad) {

        this.Arm_Gamepad = Arm_Gamepad;

        ArmMotor1.init(ahwMap,"Arm",false,true, 1);

        ArmMotor1.setMaxAccelandVel(Kp,Kd,Ki,max_accel,max_vel);
        ArmMotor1.setPidCoefs(Kp,Kd,Ki);
    }

    public void SetPidCoefs(double kp, double ki, double kd) {
        Kp = kp;
        Ki = ki;
        Kd = kd;
        ArmMotor1.setPidCoefs(Kp,Ki,Kd);
    }
    public void SetMPidCoefs(double kp, double ki, double kd, double max_accell, double max_vello) {
        ArmMotor1.setMaxAccelandVel(kp,kd,ki,max_accell,max_vello);
    }

    public void update() {

        encoder_position = ArmMotor1.MotorCurrentPosition();
        if(Arm_Gamepad.right_bumper) { ArmMotor1.SetPower(pow); Reeference = encoder_position; }
        else if(Arm_Gamepad.left_bumper) { ArmMotor1.SetPower(-pow); Reeference = encoder_position; }
        else SetPidPower(Reeference);

        if(Arm_Gamepad.dpad_up) { Reeference = 767; }
        else if(Arm_Gamepad.dpad_down) { Reeference = 100; }

    }

    public void simpleUpdate() {


        if(Arm_Gamepad.x) {
            if(toggle1) {
                pow = pow + 0.05;
                toggle1=false;
            }
            if(pow>1) pow = 1;
        }
        else toggle1=true;

        if(Arm_Gamepad.y) {
            if(toggle2) {
                pow = pow - 0.05;
                toggle2=false;
            }
            if(pow<0) pow = 0;
        } else toggle2 = true;

        if(Arm_Gamepad.right_bumper)
        {
            ArmMotor1.SetPower(-pow);
        }
        else if(Arm_Gamepad.left_bumper)
        {
            ArmMotor1.SetPower(pow);
        }
        else SetPower(0);
    }

    public void tuneUpdate(boolean gpad) {
        SetPidPower(Reeference);
        this.gpad=gpad;
    }

    public void autoUpdate()
    {
        SetPidPower(Reeference);
        lastPosition = ArmMotor1.MotorCurrentPosition();
    }

    public double getArmPos()
    {
        return ArmMotor1.MotorCurrentPosition();
    }

    public void SetPower(double power) {
        ArmMotor1.SetPower(power);
    }

    public double getPidPower() {
        double pow;
        pow = ArmMotor1.getPidPower(Reeference);
        pow=addons(pow);
        return pow;
    }


    /*public void SetPidPower(double reference)
    {
        if(firstTime) {
            timer.reset();
            firstTime = false;
        }

        if(!checkSteady())
        {
            // savedPower = -ArmMotor.returnMpidPower(reference);

            savedPower = ArmMotor1.getPidPower(reference);

            savedPower = addons(savedPower);
        }


        ArmMotor1.SetPower(savedPower);
    }*/

    /*public void SetPidPower(double reference)
    {
        double pow = ArmMotor1.getPidPower(reference);

        pow = addons(pow);

        pidPower = pow;

        if(gpad) {
            ArmMotor1.SetPower(pow);
        }
        else {
            ArmMotor1.SetPower(0);
        }
    }*/

    public void SetPidPower(double reference)
    {
        double pow = ArmMotor1.getPidPower(reference);

        //pow = addons(pow);

        ArmMotor1.SetPower(pow);
    }

    public double getCalculatedPow() {
        return pidPower;
    }

    public double addons(double pow)
    {

        if(pow > max_output)pow = max_output;
        else if(pow < -max_output)pow = -max_output;
        return pow;
    }

    public double getReference()
    {
        return Reeference;
    }

    public double getArmPower()
    {
        return ArmMotor1.GetPower();
    }

    public boolean checkSteady() {
        if((lastPosition == ArmMotor1.MotorCurrentPosition()) && timer.seconds() > 0.3) {
            return true;
        }
        else {
            return false;
        }
    }

    public void setReference(double value)
    {
        Reeference = value;
        firstTime = true;
    }



}
