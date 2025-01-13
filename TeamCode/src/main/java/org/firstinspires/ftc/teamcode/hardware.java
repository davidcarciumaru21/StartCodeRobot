package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class hardware {

    // Declarare motoare drivetrain
    public DcMotor fr = null;
    public DcMotor fl = null;
    public DcMotor br = null;
    public DcMotor bl = null;

    private HardwareMap hardwareMap = null;

    // Functia "init" reda o stare initiala a fiecarui motor din alcatuirea drivetrain-ului
    /* Aceasta funtie poate fi folosita atat in teleOp cat si in autonomie, insa se
    recomanda sa folosim doua fisiere diferite caci putem denumi altfel anumite obiecte
    in funtie de ceea ce este nevoie ca ele sa faca, spre exemplu motoarele pot avea denumiri
    mai scurte sau mai lungi pentru a face codul mai usor de citit ori pentru a spori
    productibilitatea si rapiditatea programtorului*/
    public void init(HardwareMap hwMap) {
        hardwareMap = hwMap;

        // Selectam motoarele conform configului
        /*
        fr - front right - fata dreapta
        fl - front left - fata stanga
        br - back right - spate dreapta
        bl - back left - spate stanga
        */
        fr = hardwareMap.get(DcMotor.class, "motorFR");
        fl = hardwareMap.get(DcMotor.class, "motorFL");
        br = hardwareMap.get(DcMotor.class, "motorBR");
        bl = hardwareMap.get(DcMotor.class, "motorBL");

        /* La un drivetrain de tip mecanum motoarele din fata au sensul miscarii in fata
        insa la cele din spate, sensul de miscare este opus
        */
        fr.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        // Oprim motoarele
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);

        // Motoarele sunt setate initial sa mearga fara a se folosi de encodere
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
