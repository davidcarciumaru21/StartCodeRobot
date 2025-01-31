package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware;

@TeleOp(name = "Preset Go Until Obstacle", group = "Presets")
public class presetGoUntilDetectingObstacle extends OpMode {

    //***************Declaration-of-values***************

    hardware robot = new hardware();
    // Variabile Value Nivel Apasare De Pe Touch
    double TSLv = 0.0;
    double TSRv = 0.0;
    boolean TSLt = false;
    boolean TSRt = false;

    //***************Methods***************

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

    //***************Main-methods***************

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {

        TSLv = robot.TSL.getValue();
        TSRv = robot.TSR.getValue();
        TSLt = robot.TSL.isPressed();
        TSRt = robot.TSR.isPressed();

        //***************Telemetry***************

        telemetry.addData("touch left boolean", TSLt);
        telemetry.addData("touch left value", TSLv);
        telemetry.addData("touch right boolean", TSRt);
        telemetry.addData("touch right value", TSRv);
        telemetry.addLine("version 1.31.2025.1.01");
        telemetry.update();
    }
}