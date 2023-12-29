package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.Skeletal_Structures.Motor_Skeleton;

public class Slider {

    double Kp = 0.01;
    double Kd = 0;
    double Ki = 0;

    DcMotorEx slider;
    DcMotorEx slider2;

    Gamepad slider_gamepad;

    double max_Position = 22600;

    public Motor_Skeleton slider_motor = new Motor_Skeleton(slider);

    public Slider(HardwareMap hwmap , Gamepad slider_gamepad)
    {
        slider_motor.init(hwmap,"Slider",false,true);

        slider_motor.setPidCoefs(Kp,Kd,Ki);

        this.slider_gamepad = slider_gamepad;

        reference = 0;
    }

    double pow = 1;
    double reference;

    boolean toggle1 = true;
    boolean toggle2 = true;

    static double encoder_position;


    double up_Position = max_Position;
    double down_position = 50;


    public void Run()
    {
        encoder_position = slider_motor.MotorCurrentPosition();

        if(slider_gamepad.right_bumper)
        {
            SetPower(pow);
            reference = encoder_position;
        }
        else if(slider_gamepad.left_bumper)
        {
            SetPower(-pow);
            reference = encoder_position;
        }
        else {
            SetPower(0);
        }
         //else SetPidPower();

        /*if(slider_gamepad.dpad_up && toggle2) {
            if(toggle1) {
                pow += 0.1;
                toggle1=false;
            }
        }
        else {
            toggle1 = true;
        }

        if(slider_gamepad.dpad_up && toggle1) {
            if(toggle2) {
                pow -= 0.1;
                toggle2=false;
            }
        }
        else {
            toggle2 = true;
        }
        */


         if(slider_gamepad.dpad_up)
        {
             reference = up_Position;
        }
        else if(slider_gamepad.dpad_down)
        {
              reference = down_position;
        }
    }

    double max_output = 0.2;

    public void SetPidPower()
    {
        double power = slider_motor.getPidPower(reference);

        power = slider_motor.powerConstraints(max_output,power);

        slider_motor.SetPower(power);
    }

    public void SetToReference(double reference1)
    {
        encoder_position = slider_motor.MotorCurrentPosition();
         reference = reference1;
         SetPidPower();
    }

    public void SetPower(double pow) {
        slider_motor.SetPower(pow);
    }

    public double GetPosition() {
        return slider_motor.MotorCurrentPosition();
    }

    public double getPower() {
        return pow;
    }

}
