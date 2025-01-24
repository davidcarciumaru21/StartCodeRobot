package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware;

@TeleOp(name = "servoTeleOP",  group = "TeleOP")
public class servoTeleOp extends OpMode {

    //***************Declaration-of-values***************

    hardware robot = new hardware(); // creeam obiectul responsabil de harware-ul robotului
    final double speecimenClose = 0.0;
    final double specimentOpen = 0.5;
    final double clawClose = 0.0;
    final double clasOpen = 0.5;
    boolean openSpecimen, openClaw;

    //***************Main-methods***************

    public void initServo(){
        // setam pozitia servo-urilor la inceput
        robot.specimen.setPosition(speecimenClose);
        robot.claw.setPosition(clawClose);
        openSpecimen = false;
        openClaw = false;
    }

    public void miscareServo(){
        // miscarea servo-ului ce se ocupa de specimene
        if(gamepad2.dpad_up){
            robot.specimen.setPosition(specimentOpen);
            openSpecimen = true;
        } else if(gamepad2.dpad_down){
            robot.specimen.setPosition(speecimenClose);
            openSpecimen = false;
        }

        // miscarea gharei
        if(gamepad2.dpad_right){
            robot.claw.setPosition(clasOpen);
            openClaw = true;
        } else if(gamepad2.dpad_left){
            robot.claw.setPosition(clawClose);
            openClaw = false;
        }
    }

    //***************Methods***************

    @Override
    public void init(){
        this.initServo(); // initializarea servo-urilor
    }

    @Override
    public void loop(){
        this.miscareServo(); // miscare servo-urilor

        //***************Telemetry***************

        telemetry.addData("specimentServo", openSpecimen);
        telemetry.addData("clawServo", openClaw);
        telemetry.addLine("version 1.23.2025.7.44");

        telemetry.update();
    }
}