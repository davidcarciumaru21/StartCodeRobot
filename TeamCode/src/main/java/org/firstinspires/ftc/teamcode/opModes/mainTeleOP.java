package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware; // Importam hardware-ul

@TeleOp(name = "mainTeleOP", group = "TeleOP")
public class mainTeleOP extends OpMode{

    //***************Declaration-of-values***************

    hardware robot = new hardware(); // creeam obiectul responsabil de harware-ul robotului
    double leftXJoystick1, leftYJoystick1, rightXJoystick1, up, forward; // valorile joystickurilor de la controller
    double motorFRvolt, motorFLvolt, motorBRvolt, motorBLvolt;
    final int limMax = -5300;
    final int limMin = 10;
    final int limMaxOriz = 3000;
    final int limMinOriz = 0;
    int posmV, posmO;
    final double speecimenClose = 0.0;
    final double specimentOpen = 0.5;
    final double clawClose = 0.0;
    final double clasOpen = 0.5;
    boolean openSpecimen, openClaw;
    final int[] colourGamepad1 = {255, 0, 0, 15000};
    final int[] colourGamepad2 = {0, 0, 255, 15000};
    boolean ps4Controller1 = true;
    boolean ps4Controller2 = false;

    // Felul in care vireaza gamepadu-rile la inceput
    Gamepad.RumbleEffect effect = new Gamepad.RumbleEffect.Builder()
            .addStep(1.0, 1.0, 250)
            .addStep(0.0, 1.0, 250)
            .addStep(1.0, 0.0, 250).build();

    //***************Methods***************

    public void initGamepad(){

        if (ps4Controller1) {
            gamepad1.runRumbleEffect(effect);
            gamepad1.setLedColor(colourGamepad1[0], colourGamepad1[1], colourGamepad1[2], colourGamepad1[3]);
        }

        if (ps4Controller2) {
            gamepad2.runRumbleEffect(effect);
            gamepad2.setLedColor(colourGamepad2[0], colourGamepad2[1], colourGamepad2[2], colourGamepad2[3]);
        }
    }

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

    public void initArms(){
        // ia pozitia actuala a bratului vertical pentru a seta un target position initial
        posmV = robot.mV1.getCurrentPosition();

        // initializam motoarele pentru a merge cu ajutorul encoder-elor

        robot.mV1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mV1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.mV2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mV2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.mO.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mO.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void armMoveUp() {

        up = -gamepad2.left_stick_y;

        // Verificam daca output-ul cerut se afla in limite ori daca exista un voltaj pe joystick
        if ((up > 0 && posmV + 100 * up >= limMax) || (up < 0 && posmV + 100 * up <= limMin)) {
            posmV -= 10 * up;

            robot.mV1.setTargetPosition(posmV);
            robot.mV2.setTargetPosition(posmV);

            robot.mV1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.mV2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.mV1.setPower(0.5);
            robot.mV2.setPower(0.5);
            // Daca nu exista input mototoarele trebuie sa ramana pe pozitii
        } else {
            robot.mV1.setTargetPosition(posmV);
            robot.mV2.setTargetPosition(posmV);

            robot.mV1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.mV2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.mV1.setPower(0.1);
            robot.mV2.setPower(0.1);
        }

    }

    public void armMoveLateral() {
        forward = -gamepad2.right_stick_x;

        // Verificam daca output-ul cerut se afla in limite ori daca exista un voltaj pe joystick
        if ((forward > 0 && posmO + 100 * forward <= limMaxOriz) || (forward < 0 && posmO + 100 * forward >= limMinOriz)) {
            posmO -= 10 * forward;

            robot.mO.setTargetPosition(posmO);
            robot.mO.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.mO.setPower(0.5);
            // Daca nu exista input mototoarele trebuie sa ramana pe pozitii
        } else {
            robot.mO.setTargetPosition(posmO);
            robot.mO.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.mO.setPower(0.1);
        }
    }

    // verificam daca valorile introduse pentru a merge motorul se afla intr-un interval [-1;+1]
    public boolean verifyMotorValues(double value){
        return value >= -1 && value <= 1;
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
        motorFRvolt = (leftYJoystick1 - leftXJoystick1 - rightXJoystick1) / denominator;
        motorFLvolt = (leftYJoystick1 + leftXJoystick1 + rightXJoystick1) / denominator;
        motorBRvolt = (leftYJoystick1 + leftXJoystick1 - rightXJoystick1) / denominator;
        motorBLvolt = (leftYJoystick1 - leftXJoystick1 + rightXJoystick1) / denominator;

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

        this.initArms(); // Initializarea bratelelor
        this.initServo(); // initializarea servo-urilor

        this.initGamepad();
    }

    @Override
    public void loop(){

        this.moveDriveTrain();
        this.armMoveUp();
        this.armMoveLateral();
        this.miscareServo();

        //***************Telemetry***************

        telemetry.addData("MotorFR", motorFRvolt);
        telemetry.addData("MotorFL", motorFLvolt);
        telemetry.addData("MotorBR", motorBRvolt);
        telemetry.addData("MotorBL", motorBLvolt);
        telemetry.addData("UpValues", up);
        telemetry.addData("ForwardValues", forward);
        telemetry.addData("mV", posmV);
        telemetry.addData("mO", posmO);
        telemetry.addData("specimentServo", openSpecimen);
        telemetry.addData("clawServo", openClaw);
        telemetry.addData("ps4Controller1", ps4Controller1);
        telemetry.addData("ps4Controller2", ps4Controller2);
        telemetry.addLine("version 1.24.2025.8.27");

        telemetry.update();
    }
}
