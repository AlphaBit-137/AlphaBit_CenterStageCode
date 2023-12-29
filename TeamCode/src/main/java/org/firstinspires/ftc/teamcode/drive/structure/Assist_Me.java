package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Assist_Me {

    Slider slider;
    Claw claw;
    Turret turret;

    enum States{
        START,
        BACK,
        END
    }

    States assist_states = States.START;

    Gamepad assist;

    public Assist_Me(HardwareMap map, Gamepad assist)
    {
        slider = new Slider(map,assist);
        //claw = new Claw(map,assist, "Claw");
        turret = new Turret(map,assist);

        this.assist = assist;
    }

    boolean startedExecuting = false;

    public void Run()
    {
        if(assist.dpad_up && ! startedExecuting)
        {
            startedExecuting = true;
        }

        if(startedExecuting)
        {
            Execute();
        }
    }

    double slider_ref = 0;
    double turret_ref = 0;
    double claw_pos = 0;

    double tolerance = 10;



    public void Execute()
    {
        switch (assist_states)
        {
            case START:

                slider_ref = slider.max_Position;

                if( (slider.slider_motor.MotorCurrentPosition()) > slider_ref  - tolerance)
                    turret_ref = turret.angle1;

                slider.SetToReference(slider_ref);
                turret.SetToReference(turret_ref);

                if(turret_ref != 0 && ( (turret.angle) < (turret_ref + tolerance) ))
                    assist_states = States.BACK;

                break;

            case BACK:

                turret_ref = turret.angle2;

                if( (turret.angle) > (turret_ref - tolerance))
                    slider_ref = slider.down_position;

                slider.SetToReference(slider_ref);
                turret.SetToReference(turret_ref);

                if(slider_ref == 0 && ( (slider.slider_motor.MotorCurrentPosition()) < (slider.down_position + tolerance)))
                    assist_states = States.END;

                break;

            case END:
                startedExecuting = false;
                break;
        }
    }

    public void WaitAndSetPosition(double position)
    {

        

        //claw.SetToPosition(position);
    }

}
