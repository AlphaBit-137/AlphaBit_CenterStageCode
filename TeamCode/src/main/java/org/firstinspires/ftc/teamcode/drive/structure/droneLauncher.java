package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class droneLauncher {

   public Servo launcher;

    Gamepad launcher_gamepad;

    public droneLauncher(HardwareMap hwmap, Gamepad gamepad)
    {
        launcher = hwmap.get(Servo.class,"Launcher");

        this.launcher_gamepad = gamepad;

        launcher.setPosition(closed_pos);
    }

    double closed_pos = 0.55;
    double open_pos = 0.4;

    boolean toggle = true;
    boolean open = false;

    public void Run()
    {

        if (launcher_gamepad.b) {
            if(toggle) {
                if (open) {
                    launcher.setPosition(closed_pos);
                    open=false;
                } else {
                    launcher.setPosition(open_pos);
                    open=true;
                }
                toggle=false;
            }
        }
        else {
            toggle=true;
        }
    }

    public void SetPositions(double pos) {
        launcher.setPosition(pos);
    }

}
