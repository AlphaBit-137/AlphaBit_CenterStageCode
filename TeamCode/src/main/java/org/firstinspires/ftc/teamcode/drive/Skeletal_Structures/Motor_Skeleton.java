package org.firstinspires.ftc.teamcode.drive.Skeletal_Structures;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Motor_Skeleton {

    public DcMotorEx ThisMotor;

    public Motor_Skeleton(DcMotorEx ThisMotor){
        this.ThisMotor = ThisMotor;
    }

    public double normalizer;

    double minimum;

    boolean isLinear = false;

    enum BusyStates{
        isBusy,
        isBusy2,
        notBusy
    }
    int multiplier=1;

    BusyStates bs = BusyStates.notBusy;

    Pid_Controller pid;
    MPid_Controller mpid;

    public void init(HardwareMap ahwMap,String MotorName,boolean IsReversed,boolean using_encoders,int multiplier) {

        ThisMotor = ahwMap.get(DcMotorEx.class, MotorName);

        ThisMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        this.multiplier=multiplier;

        RUN_WITH_ENCODERS(using_encoders);

        IsReversed(IsReversed);

        ThisMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ThisMotor.setPower(0);
    }

    public double MotorCurrentPosition()
    {
        return multiplier*ThisMotor.getCurrentPosition();
    }

    public void SetPower(double power)
    {
        ThisMotor.setPower(power);
    }

    public double GetPower() { return ThisMotor.getPower(); }

    public void SetVelocity(double velocity)
    {
        ThisMotor.setVelocity(velocity);
    }

    public double GetVelocity()
    {
       return ThisMotor.getVelocity();
    }

    public void IsReversed(boolean isreversed)
    {
        if(isreversed){
            ThisMotor.setDirection(DcMotor.Direction.REVERSE);
        }else ThisMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void RUN_WITH_ENCODERS(boolean encoders)
    {
        if(encoders)
        {
            ThisMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else ThisMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPidCoefs(double Kp,double Kd,double Ki)
    {
        pid = new Pid_Controller(Kp,Kd,Ki);
    }

    public void setMaxAccelandVel(double Kp, double Kd, double Ki, double maxAccel, double maxVel)
    {
       mpid = new MPid_Controller(Kp,Kd,Ki,maxAccel,maxVel);
    }

    public double powerConstraints(double maxOutput,double power)
    {
        if(power > maxOutput)power = maxOutput;
        else if(power < -maxOutput)power = -maxOutput;

        return power;
    }

    public void getNormalizer(double normalizer){
       this.normalizer = normalizer;
    }

    public void setPidPower(double Reference)
    {
        double power = pid.returnPower(Reference,MotorCurrentPosition());
        ThisMotor.setPower(power);
    }

    public double getPidPower(double Reference)
    {
        return pid.returnPower(Reference,MotorCurrentPosition());
    }

    public void setMPidPower(double reference)
    {
        double power = mpid.returnPower(reference,MotorCurrentPosition(),GetVelocity());
        ThisMotor.setPower(power);
    }

    public double returnMpidPower(double reference, double sign)
    {
        return mpid.returnPower(sign*reference,sign*MotorCurrentPosition(), sign * GetVelocity());
    }

    public void setMinimum(double minimum)
    {
        this.minimum = minimum;
    }


    public boolean isBusy(boolean Linear,double reference)
    {
        if(Linear)bs = BusyStates.isBusy;
        else bs = BusyStates.isBusy2;

        switch (bs)
        {
            case isBusy:
                return Math.abs(reference - ThisMotor.getCurrentPosition()) == 0;
            case isBusy2:
                return Math.abs(reference - ThisMotor.getCurrentPosition()) < minimum;
        }

        return false;
    }

    public boolean isSteady(double lastPosition)
    {
        return Math.abs(ThisMotor.getCurrentPosition() - lastPosition) == 0;
    }
}
