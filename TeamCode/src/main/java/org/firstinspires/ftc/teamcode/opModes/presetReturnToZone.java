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

        leftEncoderPos = robot.fl.getCurrentPosition();
        rightEncoderPos = robot.odoDreapta.getCurrentPosition();
        centerEncoderPos = robot.fr.getCurrentPosition();

        double deltaLeftEncoderPos = leftEncoderPos - prevLeftEncoderPos;
        double deltaRightEncoderPos = rightEncoderPos - prevRightEncoderPos;
        double deltaCenterEncoderPos = centerEncoderPos - prevCenterEncoderPos;

        double leftDistance = (deltaLeftEncoderPos / ENCODER_TICKS_PER_ROTATION) * (Math.PI * WHEEL_DIAMETER);
        double rightDistance = (deltaRightEncoderPos / ENCODER_TICKS_PER_ROTATION) * (Math.PI * WHEEL_DIAMETER);
        double centerDistance = (deltaCenterEncoderPos / ENCODER_TICKS_PER_ROTATION) * (Math.PI * WHEEL_DIAMETER);

        double phi = (leftDistance - rightDistance) / TRACKWIDTH;

        double deltaMiddlePos = (leftDistance + rightDistance) / 2.0;
        double deltaPerpPos = centerDistance - FORWARD_OFFSET * phi;

        double deltaX = deltaMiddlePos * Math.cos(heading) - deltaPerpPos * Math.sin(heading);
        double deltaY = deltaMiddlePos * Math.sin(heading) + deltaPerpPos * Math.cos(heading);

        xPos += deltaX;
        yPos += deltaY;
        heading += phi;

        prevLeftEncoderPos = leftEncoderPos;
        prevRightEncoderPos = rightEncoderPos;
        prevCenterEncoderPos = centerEncoderPos;
    }

    // Returnează poziția curentă a robotului (X, Y, Heading)
    public double[] getPosition() {
        return new double[] {xPos, yPos, Math.toDegrees(heading)};
    }

    public void resetX() {
        double startTime = System.currentTimeMillis();
        double timeout = 5000;
        double previousXPos = xPos;

        while (Math.abs(xPos) > treshHold && (System.currentTimeMillis() - startTime) < timeout) {
            this.updateOdometry();

            if (Math.abs(previousXPos - xPos) < 0.1) {
                break;
            }
            previousXPos = xPos;

            double power = Math.min(0.5, Math.abs(xPos) * 0.1);
            if (xPos > 0) {
                this.moveMotorsByValues(-power, -power, -power, -power);
            } else {
                this.moveMotorsByValues(power, power, power, power);
            }
        }
        this.moveMotorsByValues(0, 0, 0, 0);
    }

    public void resetY() {
        double startTime = System.currentTimeMillis();
        double timeout = 5000;
        double previousYPos = yPos;

        while (Math.abs(yPos) > treshHold && (System.currentTimeMillis() - startTime) < timeout) {
            this.updateOdometry();

            if (Math.abs(previousYPos - yPos) < 0.1) {
                break;
            }
            previousYPos = yPos;

            double power = Math.min(0.5, Math.abs(yPos) * 0.1);
            if (yPos > 0) {
                this.moveMotorsByValues(power, -power, -power, power);
            } else {
                this.moveMotorsByValues(-power, power, power, -power);
            }
        }
        this.moveMotorsByValues(0, 0, 0, 0);
    }

    // Ne intoarcem  cu robotul la pozitia initiala
    public void returnToZone(){
        if(gamepad1.square){
            this.resetY();
            this.resetX();
        }
    }

    // Cream o noua pozitie initiala
    public void resetPosition() {
        if (gamepad1.circle) {
            robot.fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.odoDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            xPos = 0.0;
            yPos = 0.0;
            heading = 0.0;
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

        robot.fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.odoDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.odoDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        this.updateOdometry(); // Actualizăm odometria robotului
        this.moveDriveTrain(); // Controlăm mișcarea robotului pe baza joystickurilor
        this.returnToZone(); // Preset-ul pentru a ajunge inapoi la pozitia initiala
        this.resetPosition(); // Resetam pozitia initiala

        double[] position = this.getPosition();

        //***************Telemetry***************

        telemetry.addData("MotorFR", motorFRvolt);
        telemetry.addData("MotorFL", motorFLvolt);
        telemetry.addData("MotorBR", motorBRvolt);
        telemetry.addData("MotorBL", motorBLvolt);
        telemetry.addData("X (cm)", position[0]);
        telemetry.addData("Y (cm)", position[1]);
        telemetry.addData("Heading (degrees)", position[2]);
        telemetry.addLine("version 1.31.2025.1.01");
        telemetry.update();
    }
}
