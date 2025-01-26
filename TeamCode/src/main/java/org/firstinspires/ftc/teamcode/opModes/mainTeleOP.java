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
    final int limMin = 0;
    final int limMaxOriz = 1000;
    final int limMinOriz = 0;
    int posmV, posmO;

    final double speecimenClose = 0.0;
    final double specimentOpen = 0.5;
    final double clawClose = 0.0;
    final double clasOpen = 0.5;
    boolean openSpecimen, openClaw;

    double xPos = 0.0; // Poziția robotului pe axa X în cm
    double yPos = 0.0; // Poziția robotului pe axa Y în cm
    double heading = 0.0; // Orientarea robotului în radiani

    int leftEncoderPos = 0;
    int rightEncoderPos = 0;
    int centerEncoderPos = 0;

    int prevLeftEncoderPos = 0;
    int prevRightEncoderPos = 0;
    int prevCenterEncoderPos = 0;

    //***************Constants***************

    final double ENCODER_TICKS_PER_ROTATION = 2048; // Numărul de ticks pentru o rotație completă a encoderului
    final double WHEEL_DIAMETER = 10.4; // Diametrul roților de odometrie în cm
    final double TRACKWIDTH = 21; // Distanța între roțile de odometrie stânga și dreapta în cm
    final double FORWARD_OFFSET = 17.5; // Distanța de la roata centrală la centrul robotului
    final double treshHold = 1;

    //***************Methods***************

    // Actualizează odometria robotului pe baza datelor de la encodere
    public void updateOdometry() {
        // Obținem pozițiile curente ale encoderelor
        leftEncoderPos = robot.fl.getCurrentPosition();
        rightEncoderPos = robot.odoDreapta.getCurrentPosition();
        centerEncoderPos = robot.fr.getCurrentPosition();

        // Calculăm schimbările de poziție (delta) ale encoderelor
        double deltaLeftEncoderPos = leftEncoderPos - prevLeftEncoderPos;
        double deltaRightEncoderPos = rightEncoderPos - prevRightEncoderPos;
        double deltaCenterEncoderPos = centerEncoderPos - prevCenterEncoderPos;

        // Convertim ticks de encoder în distanțe parcurse de fiecare roată
        double leftDistance = (deltaLeftEncoderPos / ENCODER_TICKS_PER_ROTATION) * (Math.PI * WHEEL_DIAMETER);
        double rightDistance = (deltaRightEncoderPos / ENCODER_TICKS_PER_ROTATION) * (Math.PI * WHEEL_DIAMETER);
        double centerDistance = (deltaCenterEncoderPos / ENCODER_TICKS_PER_ROTATION) * (Math.PI * WHEEL_DIAMETER);

        // Calculăm schimbarea de heading (rotație)
        double phi = (leftDistance - rightDistance) / TRACKWIDTH;

        // Calculăm deplasările robotului în axa X și Y
        double deltaMiddlePos = (leftDistance + rightDistance) / 2.0;
        double deltaPerpPos = centerDistance - FORWARD_OFFSET * phi;

        // Actualizăm poziția robotului
        double deltaX = deltaMiddlePos * Math.cos(heading) - deltaPerpPos * Math.sin(heading);
        double deltaY = deltaMiddlePos * Math.sin(heading) + deltaPerpPos * Math.cos(heading);

        xPos += deltaX; // Actualizăm poziția X
        yPos += deltaY; // Actualizăm poziția Y
        heading += phi; // Actualizăm rotația (heading)

        // Salvăm pozițiile encoderelor pentru următoarea actualizare
        prevLeftEncoderPos = leftEncoderPos;
        prevRightEncoderPos = rightEncoderPos;
        prevCenterEncoderPos = centerEncoderPos;
    }

    // Returnează poziția curentă a robotului (X, Y, Heading)
    public double[] getPosition() {
        return new double[] {xPos, yPos, Math.toDegrees(heading)};
    }

    public void resetX() {
        while (Math.abs(xPos) > treshHold) { // Verificam daca nu cumva am ajuns deja la pozitia 0
            // Miscam robotul astfel incat sa jaungem la pozitia 0
            if (xPos > 0) {
                this.moveMotorsByValues(-1, -1, -1, -1);
            } else {
                this.moveMotorsByValues(1, 1, 1, 1);
            }
        }
        this.moveMotorsByValues(0, 0, 0, 0); // Stop the robot once the position is within the threshold
    }
    public void resetY () {
        while (Math.abs(yPos) > treshHold) {  // Verificam daca nu cumva am ajuns deja la pozitia 0
            // Miscam robotul astfel incat sa jaungem la pozitia 0
            if (yPos > 0) {
                this.moveMotorsByValues(1, -1, -1, 1);
            } else {
                this.moveMotorsByValues(-1, 1, 1, -1);
            }
        }
        this.moveMotorsByValues(0, 0, 0, 0); // Stop the robot once the position is within the threshold
    }

    // Ne intoarcem  cu robotul la pozitia initiala
    public void returnToZone(){
        if(gamepad1.square){
            this.resetY();
            this.resetX();
        }
    }

    // Cream o noua pozitie initiala
    public void resetPosition(){
        if(gamepad1.circle){
            robot.fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.odoDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        if ((up > 0 && posmV + 100 * -up >= limMax) || (up < 0 && posmV + 100 * -up <= limMin)) {
            posmV -= 10 * up;

            robot.mV1.setTargetPosition(posmV);
            robot.mV2.setTargetPosition(posmV);

            robot.mV1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.mV2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.mV1.setPower(1);
            robot.mV2.setPower(1);
            // Daca nu exista input mototoarele trebuie sa ramana pe pozitii
        } else {
            robot.mV1.setTargetPosition(posmV);
            robot.mV2.setTargetPosition(posmV);

            robot.mV1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.mV2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.mV1.setPower(1);
            robot.mV2.setPower(1);
        }

    }

    public void armMoveLateral() {
        forward = -gamepad2.right_stick_x;

        // Verificam daca output-ul cerut se afla in limite ori daca exista un voltaj pe joystick
        if ((forward < 0 && posmO + 50 * -forward <= limMaxOriz) || (forward > 0 && posmO + 50 * -forward >= limMinOriz)) {
            posmO -= 50 * forward;

            robot.mO.setTargetPosition(posmO);
            robot.mO.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.mO.setPower(1);
            // Daca nu exista input mototoarele trebuie sa ramana pe pozitii
        } else {
            robot.mO.setTargetPosition(posmO);
            robot.mO.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.mO.setPower(1);
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
    }

    @Override
    public void loop(){

        this.moveDriveTrain(); // Controlăm mișcarea robotului pe baza joystickurilor
        this.armMoveUp(); // Miscam bratul vertical
        this.armMoveLateral(); // Miscam bratul orizontal
        this.miscareServo(); // Miscam servo-urile
        this.returnToZone(); // Preset-ul pentru a ajunge inapoi la pozitia initiala
        this.resetPosition(); // Resetam pozitia initiala
        this.updateOdometry(); // Actualizăm odometria robotului

        double[] position = this.getPosition();

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
        telemetry.addData("X (cm)", position[0]);
        telemetry.addData("Y (cm)", position[1]);
        telemetry.addData("Heading (degrees)", position[2]);
        telemetry.addLine("version 1.26.2025.23.57");

        telemetry.update();
    }
}
