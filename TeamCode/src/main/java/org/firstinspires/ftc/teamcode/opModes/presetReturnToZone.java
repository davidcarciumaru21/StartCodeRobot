package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware;

@TeleOp(name = "PresetReturnToZone", group = "Presets")
public class presetReturnToZone extends OpMode {

    //***************Declaration-of-values***************

    hardware robot = new hardware(); // Instanțiem obiectul robot

    // Declaram variabile pentru valorile joystickurilor de la controller
    public double leftXJoystick1, leftYJoystick1, rightXJoystick1;
    double motorFRvolt, motorFLvolt, motorBRvolt, motorBLvolt;

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

    // Verifică dacă valoarea motorului este în intervalul valid [-1, 1]
    public boolean verifyMotorValues(double value) {
        return value >= -1 && value <= 1;
    }

    // Setează viteza motoarelor pe baza valorilor date
    public void moveMotorsByValues(double FRmotorValue, double FLmotorValue, double BRmotorValue, double BLmotorValue) {
        if (this.verifyMotorValues(FRmotorValue) && this.verifyMotorValues(FLmotorValue) &&
                this.verifyMotorValues(BRmotorValue) && this.verifyMotorValues(BLmotorValue)) {
            robot.fr.setPower(FRmotorValue);
            robot.fl.setPower(FLmotorValue);
            robot.br.setPower(BRmotorValue);
            robot.bl.setPower(BLmotorValue);
        }
    }

    // Controlul mișcării robotului pe baza joystickurilor
    public void moveDriveTrain() {
        leftYJoystick1 = gamepad1.left_stick_y;  // Mișcare pe axa Y (înainte/spate)
        leftXJoystick1 = -gamepad1.left_stick_x; // Mișcare pe axa X (stânga/dreapta)
        rightXJoystick1 = -gamepad1.right_stick_x; // Mișcare de rotație pe loc

        // Calculăm un denominator pentru a normaliza mișcările joystick-urilor
        double denominator = Math.max(Math.abs(leftYJoystick1) + Math.abs(leftXJoystick1) + Math.abs(rightXJoystick1), 1);

        // Calculăm viteza fiecărui motor pe baza poziției joystick-urilor
        motorFRvolt = (leftYJoystick1 - leftXJoystick1 - rightXJoystick1) / denominator;
        motorFLvolt = (leftYJoystick1 + leftXJoystick1 + rightXJoystick1) / denominator;
        motorBRvolt = (leftYJoystick1 + leftXJoystick1 - rightXJoystick1) / denominator;
        motorBLvolt = (leftYJoystick1 - leftXJoystick1 + rightXJoystick1) / denominator;

        // Aplicăm valorile calculate pe fiecare motor
        this.moveMotorsByValues(motorFRvolt, motorFLvolt, motorBRvolt, motorBLvolt);
    }

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

    //***************Main-methods***************

    @Override
    public void init() {
        /*Initializam toate accesorile robotului, cum ar fi:
         *   -Motoarele:
         *       -fr - "motorFR" (Config - 0)
         *       -fl - "motorFl" (Config - 1)
         *       -br - "motorBR" (Config - 2)
         *       -bl - "motorBL" (Config - 3)
         */
        robot.init(hardwareMap); // Inițializăm hardware-ul robotului
    }

    @Override
    public void loop() {
        this.updateOdometry(); // Actualizăm odometria robotului
        this.moveDriveTrain(); // Controlăm mișcarea robotului pe baza joystickurilor
        this.returnToZone(); // Preset-ul pentru a ajunge inapoi la pozitia initiala
        this.resetPosition(); // Resetam pozitia initiala

        //***************Telemetry***************

        double[] position = this.getPosition();
        telemetry.addData("MotorFR", motorFRvolt);
        telemetry.addData("MotorFL", motorFLvolt);
        telemetry.addData("MotorBR", motorBRvolt);
        telemetry.addData("MotorBL", motorBLvolt);
        telemetry.addData("X (cm)", position[0]);
        telemetry.addData("Y (cm)", position[1]);
        telemetry.addData("Heading (degrees)", position[2]);
        telemetry.addLine("version 1.26.2025.12.39");
        telemetry.update();
    }
}
