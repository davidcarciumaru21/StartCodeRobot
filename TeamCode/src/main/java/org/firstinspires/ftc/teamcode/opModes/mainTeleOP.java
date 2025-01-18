package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware;

@TeleOp(name = "mainTeleOP", group = "TeleOP")
public class mainTeleOP extends OpMode{
    //***************Declaration-of-values***************

    hardware robot = new hardware(); // creeam obiectul responsabil de harware-ul robotului
    double leftXJoystick1, leftYJoystick1, rightXJoystick1, up, forward; // valorile joystickurilor de la controller
    int posmV, posmO;
    int limMax = -5300;
    int limMin = 10;
    int limMaxOriz = 3000;
    int limMinOriz = 0;
    double motorFRvolt, motorFLvolt, motorBRvolt, motorBLvolt;

    //***************Methods***************

    public void armMoveUp() {

        up = -gamepad2.left_stick_y;

        if ((up > 0 && posmV + 100 * up >= limMax) || (up < 0 && posmV + 100 * up <= limMin)) {
            posmV -= 10 * up;
            robot.mV1.setTargetPosition(posmV);
            robot.mV2.setTargetPosition(posmV);

            robot.mV1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.mV2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if(robot.mV1.getCurrentPosition() < posmV) {
                robot.mV1.setPower(0.5);
                robot.mV2.setPower(0.5);
            } else{
                robot.mV1.setPower(-0.5);
                robot.mV2.setPower(-0.5);
            }
        } else {
            // If joystick is not moved or the limits are reached, stop the motors
            robot.mV1.setPower(posmV);
            robot.mV2.setPower(posmV);
            robot.mV1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.mV2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.mV1.setPower(0.1);
            robot.mV2.setPower(0.1);
        }
    }

    public void armMoveLateral() {
        forward = -gamepad2.right_stick_x;

        if ((forward > 0 && posmO + 100 * forward <= limMaxOriz) || (forward < 0 && posmO + 100 * forward >= limMinOriz)) {
            posmO -= 10 * forward;
            robot.mO.setTargetPosition(posmO);
            if(robot.mO.getCurrentPosition() < posmO) {
                robot.mO.setPower(0.5);
            } else {
                robot.mO.setPower(-0.5);
            }
            robot.mO.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            robot.mO.setTargetPosition(posmO); // Maintain the current target position
            robot.mO.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Ensure position-holding mode
            robot.mO.setPower(0.1); // Apply minimal
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

        posmV = robot.mV1.getCurrentPosition();

        robot.mV1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mV1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.mV2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mV2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.mO.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mO.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop(){

        this.moveDriveTrain();
        this.armMoveUp();
        this.armMoveLateral();

        telemetry.addData("MotorFR", motorFRvolt);
        telemetry.addData("MotorFL", motorFLvolt);
        telemetry.addData("MotorBR", motorBRvolt);
        telemetry.addData("MotorBL", motorBLvolt);
        telemetry.addData("UpValues", up);
        telemetry.addData("ForwardValues", forward);
        telemetry.addData("mV", posmV);
        telemetry.addData("mO", posmO);

        telemetry.update();
    }
}
