package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware;

@TeleOp(name = "mainTeleOP", group = "TeleOP")
public class presetSpecimene  extends OpMode {
    hardware robot = new hardware(); // creeam obiectul responsabil de harware-ul robotului
    double leftXJoystick1, leftYJoystick1, rightXJoystick1, up, forward; // valorile joystickurilor de la controller
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

    double TSLv = 0.0;
    double TSRv = 0.0;
    boolean TSLt = false;
    boolean TSRt = false;

    final int limMax = -5300;
    final int limMin = 10;
    final int limMaxOriz = 3000;
    final int limMinOriz = 0;
    int posmV, posmO;


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

    // Metodă pentru a muta robotul pe axa X cu o distanță specifică în cm
    public void moveToX(double distanceCm, double power) {
        double targetX = xPos + distanceCm; // Calculăm poziția țintă

        while (Math.abs(xPos - targetX) > treshHold) {
            this.updateOdometry(); // Actualizăm odometria
            double moveX = Math.signum(distanceCm) * power; // Determinăm direcția de mișcare

            this.moveMotorsByValues(moveX, moveX, moveX, moveX); // Mișcare înainte/înapoi
        }
        this.moveMotorsByValues(0, 0, 0, 0); // Oprire motoare
    }

    // Metodă pentru a muta robotul pe axa Y cu o distanță specifică în cm
    public void moveToY(double distanceCm, double power) {
        double targetY = yPos + distanceCm; // Calculăm poziția țintă

        while (Math.abs(yPos - targetY) > treshHold) {
            this.updateOdometry(); // Actualizăm odometria
            double moveY = Math.signum(distanceCm) * power; // Determinăm direcția de mișcare

            this.moveMotorsByValues(moveY, -moveY, -moveY, moveY); // Mișcare stânga/dreapta
        }
        this.moveMotorsByValues(0, 0, 0, 0); // Oprire motoare
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

        robot.fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.odoDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.odoDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void goUntilObstacle(double speedF, double speedS){
        speedF = -speedF;
        speedS = -speedS;

        while(robot.TSL.getValue() < 0.1 && robot.TSR.getValue() < 0.1) {
            moveMotorsByValues(speedF, speedF, speedF, speedF);
        }
        moveMotorsByValues(0,0,0,0);
        while(robot.TSL.getValue() < 1 || robot.TSR.getValue() < 1) {
            if (robot.TSL.getValue() > 0 && robot.TSR.getValue() < 0.1) {
                while (robot.TSR.getValue() < 0.1) {
                    moveMotorsByValues(speedS, 0, speedS, 0);
                }
            } else if (robot.TSR.getValue() > 0 && robot.TSL.getValue() < 0.1) {
                while (robot.TSL.getValue() < 0.1) {
                    moveMotorsByValues(0, speedS, 0, speedS);
                }

            } else {
                moveMotorsByValues(0, 0, 0, 0);
            }
            moveMotorsByValues(0, 0, 0, 0);
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
            posmV -= 10 * up; // Update the target position first

            robot.mV1.setTargetPosition(posmV);
            robot.mV2.setTargetPosition(posmV);

            robot.mV1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.mV2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Adjust motor power based on the position
            robot.mV1.setPower(0.5);
            robot.mV2.setPower(0.5);
        } else {
            // Ensure target position is set even if no movement is needed
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
            posmO -= 10 * forward; // Update the target position first

            robot.mO.setTargetPosition(posmO);
            robot.mO.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.mO.setPower(0.5);
        } else {

            robot.mO.setTargetPosition(posmO);
            robot.mO.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.mO.setPower(0.1);
        }
    }

    // Preset ca sa mergi la pozitia maxima de sus
    public void goUp(){
        if(gamepad2.dpad_up) {
            robot.mV1.setTargetPosition(limMax + 3000);
            robot.mV2.setTargetPosition(limMax + 3000);

            robot.mV1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.mV2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.mV1.setPower(0.5);
            robot.mV2.setPower(0.5);
        }
        if (robot.mV1.getCurrentPosition() >= robot.mV1.getTargetPosition() &&
                robot.mV2.getCurrentPosition() >= robot.mV2.getTargetPosition()) {
            robot.mV1.setPower(0);
            robot.mV2.setPower(0);
        }
    }

    // Preset ca sa mergi la pozitia maxima de jos
    public void goDown(){
        if(gamepad1.dpad_down) {
            robot.mV1.setTargetPosition(limMin - 100);
            robot.mV2.setTargetPosition(limMin - 100);

            robot.mV1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.mV2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.mV1.setPower(0.5);
            robot.mV2.setPower(0.5);
        }
        if (robot.mV1.getCurrentPosition() <= robot.mV1.getTargetPosition() &&
                robot.mV2.getCurrentPosition() <= robot.mV2.getTargetPosition()) {
            robot.mV1.setPower(0);
            robot.mV2.setPower(0);
        }
    }

    public void fromBaseToSpecimen(){
        if(gamepad1.options) {
            this.updateOdometry();
            // Mergem lana submersinbil
            this.moveToX(-30, 1);
            this.goUntilObstacle(1);
            // Resetam bratul orizontal
            robot.mO.setTargetPosition(limMinOriz + 100);
            robot.mO.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.mO.setPower(0.5);
            // Punem Soecimenul
            this.moveToY(1, 1);
            this.goUp();
            this.moveToY(1, 1);
            this.goDown();
            // Ne intoarcem in baza
            this.resetX();
            this.resetY();
        }
    }

    @Override
    public void loop(){
        this.moveDriveTrain(); // Controlăm mișcarea robotului pe baza joystickurilor
        this.returnToZone(); // Preset-ul pentru a ajunge inapoi la pozitia initiala
        this.resetPosition(); // Resetam pozitia initiala
        this.updateOdometry(); // Actualizăm odometria robotului
        this.fromBaseToSpecimen(); // Mergem sa punem un specimen

        TSLv = robot.TSL.getValue();
        TSRv = robot.TSR.getValue();
        TSLt = robot.TSL.isPressed();
        TSRt = robot.TSR.isPressed();

        double[] position = this.getPosition();

        //***************Telemetry***************

        telemetry.addData("MotorFR", motorFRvolt);
        telemetry.addData("MotorFL", motorFLvolt);
        telemetry.addData("MotorBR", motorBRvolt);
        telemetry.addData("MotorBL", motorBLvolt);
        telemetry.addData("UpValues", up);
        telemetry.addData("ForwardValues", forward);
        telemetry.addData("mV1", posmV);
        telemetry.addData("mO", posmO);
        telemetry.addData("X (cm)", position[0]);
        telemetry.addData("Y (cm)", position[1]);
        telemetry.addData("Heading (degrees)", position[2]);
        telemetry.addData("touch left boolean", TSLt);
        telemetry.addData("touch left value", TSLv);
        telemetry.addData("touch right boolean", TSRt);
        telemetry.addData("touch right value", TSRv);
        telemetry.addLine("version 1.31.2025.1.01");
        telemetry.update();
    }
}
