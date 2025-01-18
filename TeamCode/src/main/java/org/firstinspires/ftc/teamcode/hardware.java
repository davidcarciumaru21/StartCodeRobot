package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class hardware {

    // Declarare motoare drivetrain
    public DcMotor fr = null;
    public DcMotor fl = null;
    public DcMotor br = null;
    public DcMotor bl = null;

    //Declarare motoare brate;
    public DcMotor mV1; //motor vertical 1
    public DcMotor mV2; //motor vertical 2
    public DcMotor mO; //motor orizontal
    public DcMotorEx ododreapta; //odometrie dreapta

    private HardwareMap hardwareMap = null;

    // Functia "init" reda o stare initiala a fiecarui motor din alcatuirea drivetrain-ului
    // Aceasta funtie poate fi folosita atat in teleOp cat si in autonomie
    public void init(HardwareMap hwMap) {
        hardwareMap = hwMap;

        // Selectam motoarele conform configului
        /*
        fr - front right - fata dreapta
        fl - front left - fata stanga
        br - back right - spate dreapta
        bl - back left - spate stanga
        */
        fr = hardwareMap.get(DcMotor.class, "MotorFR");
        fl = hardwareMap.get(DcMotor.class, "MotorFL");
        br = hardwareMap.get(DcMotor.class, "MotorBR");
        bl = hardwareMap.get(DcMotor.class, "MotorBL");
        mV1 = hardwareMap.get(DcMotor.class, "motorVert1");
        mV2 = hardwareMap.get(DcMotor.class, "motorVert2");
        mO = hardwareMap.get(DcMotor.class, "motorOriz");

        /* La un drivetrain de tip mecanum motoarele din fata au sensul miscarii in fata
        insa la cele din spate, sensul de miscare este opus
        */
        fr.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);
        mV1.setDirection(DcMotor.Direction.FORWARD);
        mV2.setDirection(DcMotor.Direction.REVERSE);
        mO.setDirection(DcMotor.Direction.FORWARD);


        // Oprim motoarele
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
        mV1.setPower(0);
        mV2.setPower(0);
        mO.setPower(0);

        // Motoarele sunt setate initial sa mearga fara a se folosi de encodere
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mV1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mV2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mO.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mV1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mV2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mO.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}