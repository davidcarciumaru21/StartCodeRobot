package org.firstinspires.ftc.teamcode.opModes;

// Import-uri din SDK

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// Import-uri fisiere proprii

import org.firstinspires.ftc.teamcode.hardware;

@TeleOp(name = "PresetReturnToZone", group = "Presets")
public class presetReturnToZone extends OpMode {

    //***************Declaration-of-values***************

    final double WHEEL_DIAMETER = 10.4;
    final double ENCODER_TICKS_PER_ROTATION = 2048;
    final double TRACKWIDTH = 30.0; // distanta dintre odoR si odoL

    double robotX = 0.0;
    double robotY = 0.0;
    double robotHeading = 0.0;

    int rightEncoderPos = 0;
    int leftEncoderPos = 0;
    int previousRightEncoderPos = 0;
    int previousLeftEncoderPos = 0;

    double rightDistance = 0.0;
    double leftDistance = 0.0;
    double deltaRight = 0.0;
    double deltaLeft = 0.0;

    double deltaHeading;
    double deltaX;
    double robotHeadingDegrees;

    public double leftXJoystick1, leftYJoystick1, rightXJoystick1; // valorile joystickurilor de la controller
    double motorFRvolt, motorFLvolt, motorBRvolt, motorBLvolt;

    hardware robot = new hardware();

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
         * Br = (y+x-rx)/dz
         * Bl = (y-x+rx)/d
         */
        motorFRvolt = (leftYJoystick1 - leftXJoystick1 - rightXJoystick1) / denominator;
        motorFLvolt = (leftYJoystick1 + leftXJoystick1 + rightXJoystick1) / denominator;
        motorBRvolt = (leftYJoystick1 + leftXJoystick1 - rightXJoystick1) / denominator;
        motorBLvolt = (leftYJoystick1 - leftXJoystick1 + rightXJoystick1) / denominator;

        // Miscatul propriu-zis al motoarelor prin intermediul valorilor calculate mai devreme
        this.moveMotorsByValues(motorFRvolt, motorFLvolt, motorBRvolt, motorBLvolt);
    }

    public void updateOdometry() {

        // Obținem pozițiile curente ale encoder-elor
        rightEncoderPos = robot.fr.getCurrentPosition(); // Poziția curentă a encoder-ului drept
        leftEncoderPos = robot.fl.getCurrentPosition(); // Poziția curentă a encoder-ului stâng

        // Calculăm diferențele de poziție între actualizarea curentă și anterioară
        deltaRight = rightEncoderPos - previousRightEncoderPos; // Mișcarea roții drepte
        deltaLeft = leftEncoderPos - previousLeftEncoderPos; // Mișcarea roții stângi

        // Calculăm distanța parcursă de fiecare roată în centimetri, folosind formula distanței circulare
        rightDistance = (deltaRight / ENCODER_TICKS_PER_ROTATION) * (Math.PI * WHEEL_DIAMETER); // Distanța roții drepte
        leftDistance = (deltaLeft / ENCODER_TICKS_PER_ROTATION) * (Math.PI * WHEEL_DIAMETER); // Distanța roții stângi

        // Calculăm schimbarea unghiului robotului, pe baza diferenței de distanță a roților
        deltaHeading = (leftDistance - rightDistance) / TRACKWIDTH; // Diferența de unghi (în radiani)
        robotHeading += deltaHeading; // Actualizăm orientarea robotului

        // Convertim unghiul în radiani în grade, pentru a-l face mai ușor de înțeles și folosit
        robotHeadingDegrees = Math.toDegrees(robotHeading); // Conversia unghiului din radiani în grade

        // Calculăm distanța totală parcursă pe direcția înainte (pe axele X și Y)
        deltaX = (leftDistance + rightDistance) / 2.0; // Mișcarea medie a robotului pe axa X

        // Actualizăm pozițiile robotului pe axele X și Y folosind trigonometria
        robotX += deltaX * Math.cos(robotHeading); // Actualizarea poziției pe axa X
        robotY += deltaX * Math.sin(robotHeading); // Actualizarea poziției pe axa Y

        if (deltaX < 0) {
            robotX = -robotX; // If robot is moving left, negate the X value
        }
        if (robotY < 0) {
            robotY = -robotY; // If robot is moving down, negate the Y value
        }

        // Salvăm pozițiile anterioare ale encoder-elor pentru următoarea actualizare
        previousRightEncoderPos = rightEncoderPos; // Actualizăm poziția anterioară a encoder-ului drept
        previousLeftEncoderPos = leftEncoderPos; // Actualizăm poziția anterioară a encoder-ului stâng
    }

    public double[] getPosition() {
        // Convertim unghiul robotului în grade și returnăm poziția ca un array
        robotHeadingDegrees = Math.toDegrees(robotHeading); // Conversia unghiului în grade
        return new double[] {robotX, robotY, robotHeadingDegrees}; // Returnăm poziția în format {X, Y, Heading (în grade)}
    }

    public void returnToOrigin() {
        double threshold = 0.5; // Pentru a fi sigur ca este aproape de zona

        if (gamepad1.square) {
            while (Math.abs(robotX) > threshold) { // Verificam daca nu cumva am ajuns deja la pozitia 0
                this.updateOdometry();
                this.getPosition();
                if (robotY > threshold) {
                    this.moveMotorsByValues(-1, -1, -1, -1);
                } else if (robotY < -threshold) {
                    this.moveMotorsByValues(1, 1, 1, 1);
                }
            }
            this.moveMotorsByValues(0, 0, 0, 0);
            while (Math.abs(robotY) > threshold) {
                this.updateOdometry();
                this.getPosition();
                if (robotX > threshold) {
                    this.moveMotorsByValues(1, -1, -1, 1);
                } else if (robotX < -threshold) {
                    this.moveMotorsByValues(-1, 1, 1, -1);
                }

                this.moveMotorsByValues(0, 0, 0, 0);
            }
            this.moveMotorsByValues(0, 0, 0, 0);
        }
    }

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
        this.returnToOrigin();
    }

    //***************Main-methods***************

    @Override
    public void loop(){
        this.updateOdometry();
        this.moveDriveTrain();

        //***************Telemetry***************

        telemetry.addData("MotorFR", motorFRvolt);
        telemetry.addData("MotorFL", motorFLvolt);
        telemetry.addData("MotorBR", motorBRvolt);
        telemetry.addData("MotorBL", motorBLvolt);
        telemetry.addData("x in cm", (this.getPosition())[0]);
        telemetry.addData("y in cm", (this.getPosition())[1]);
        telemetry.addData("angle in degrees", (this.getPosition())[2]);
        telemetry.addLine("version 1.26.2025.12.39");

        telemetry.update();
    }
}
