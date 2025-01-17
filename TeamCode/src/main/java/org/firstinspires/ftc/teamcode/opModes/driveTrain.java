package org.firstinspires.ftc.teamcode.opModes;

// Import-uri din SDK

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Import-uri fisiere proprii

import org.firstinspires.ftc.teamcode.hardware;

@TeleOp(name = "TeleOpDriveTrain", group = "TeleOP")
public class driveTrain extends OpMode {

    //***************Declaration-of-values***************

    hardware robot = new hardware(); // creeam obiectul responsabil de harware-ul robotului
    public double leftXJoystick1, leftYJoystick1, rightXJoystick1; // valorile joystickurilor de la controller

    //***************Methods***************

    // verificam daca valorile introduse pentru a merge motorul se afla intr-un interval [-1;+1]
    public boolean verifyMotorValues(double value){
        return value >= 0 && value <= 1;
    }

    /* Metoda moveMotorsByValues misca fiecare motor dupa valoarea specifica
    fiecaruia in parametri*/

    public void moveMotorsByValues(
            double FRmotorValue, double FLmotorValue,
            double BRmotorValue, double BLmotorValue)
    {
        // verificam ca valorile sa se afle intr-un interval [-1;+1]
        if (this.verifyMotorValues(FRmotorValue) && this.verifyMotorValues(FLmotorValue) &&
            this.verifyMotorValues(BRmotorValue) && this.verifyMotorValues(BLmotorValue))
        {
        robot.fr.setPower(FRmotorValue);
        robot.fl.setPower(FLmotorValue);
        robot.br.setPower(BRmotorValue);
        robot.bl.setPower(BLmotorValue);
        }
    }

    // In aceasta metoda drivetrain-ul este condus cu ajutorul valorilor de la joystick
    public void moveDriveTrain(){
        // setam valorile ce ne intereseaza pentru miscarea robotului
        leftYJoystick1 = gamepad1.left_stick_y;
        leftXJoystick1 = -gamepad1.left_stick_x;
        rightXJoystick1 = -gamepad1.right_stick_x;
        // denominator este un numar ce face ca valorile pe care le preiau motoarele sa fie mai mici ca 1
        double denominator = Math.max(Math.abs(leftYJoystick1) + Math.abs(leftXJoystick1)
            + Math.abs(rightXJoystick1), 1);

        // Calculam valorile cu care va merge fiecare motor
        // Valoarea motorului este redata de o relatie alcatuita din leftY, leftX si rightX si de denominator
        /*
        * Fr = (y-x-rx)/d
        * Fl = (y+x+rx)/d
        * Br = (y+x-rx)/d
        * Bl = (y-x+rx)/d
        */
        double motorFRvolt = (leftYJoystick1 - leftXJoystick1 - rightXJoystick1) / denominator;
        double motorFLvolt = (leftYJoystick1 + leftXJoystick1 + rightXJoystick1) / denominator;
        double motorBRvolt = (leftYJoystick1 + leftXJoystick1 - rightXJoystick1) / denominator;
        double motorBLvolt = (leftYJoystick1 - leftXJoystick1 + rightXJoystick1) / denominator;

        // Miscatul propriu-zis al motoarelor prin intermediul valorilor calculate mai devreme
        this.moveMotorsByValues(motorFRvolt, motorFLvolt, motorBRvolt, motorBLvolt);
    }

    //***************Main-methods***************

    @Override
    public void init(){
        /*Initializam toate accesorile robotului, cum ar fi:
        *   -Motoarele:
        *       -fr - "motorFR" (Config - 0)
        *       -fl - "motorFl" (Config - 1)
        *       -br - "motorBR" (Config - 2)
        *       -bl - "motorBL" (Config - 3)
        */
        robot.init(hardwareMap);
    }

    @Override
    public void loop(){
        // Functia ce declanseaza miscarea drivetrain-ului
        this.moveDriveTrain();
    }
}
