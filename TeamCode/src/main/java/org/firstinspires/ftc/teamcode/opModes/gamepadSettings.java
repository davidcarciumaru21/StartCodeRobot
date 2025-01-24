package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "gamepadSettings", group = "gamepad")
public class gamepadSettings extends OpMode {

    //***************Declaration-of-values***************

    final int colourGamepad1[] = {255, 0, 0, 15000};
    final int colourGamepad2[] = {0, 0, 255, 15000};

    Gamepad.RumbleEffect effect = new Gamepad.RumbleEffect.Builder()
            .addStep(1.0, 1.0, 500);

    //***************Main-methods***************

    @Override
    public void init(){

        gamepad1.runRumbleEffect(effect);
        gamepad2.runRumbleEffect(effect);
        gamepad1.setLedColor(colourGamepad1[0], colourGamepad1[1], colourGamepad1[2], colourGamepad1[3]);
        gamepad2.setLedColor(colourGamepad2[0], colourGamepad2[1], colourGamepad2[2], colourGamepad2[3]);
    }

    @Override
    public void loop(){

        //***************Telemetry***************

        telemetry.addLine("version 1.24.2025.7.21");

        telemetry.update();
    }

}
