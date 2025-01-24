package org.firstinspires.ftc.teamcode.opModes;

// Import-uri din SDK
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// Import-uri fisiere proprii
import org.firstinspires.ftc.teamcode.hardware;

@TeleOp(name = "ArmTeleOP", group = "TeleOP")
public class armOp extends OpMode {

    //***************Declaration-of-values***************

    hardware robot = new hardware(); // creeam obiectul responsabil de harware-ul robotului
    double up, forward; // valorile joystickurilor de la controller
    final int limMax = -5300;
    final int limMin = 10;
    final int limMaxOriz = 3000;
    final int limMinOriz = 0;
    int posmV, posmO;

    //***************Methods***************

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

            // Adjust motor power based on the position
            robot.mO.setPower(0.5);
        } else {
            // Ensure target position is set even if no movement is needed
            robot.mO.setTargetPosition(posmO);
            robot.mO.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.mO.setPower(0.1);
        }
    }

    //***************Main-methods***************

    @Override
    public void init() {
        // Initialize the robot hardware
        robot.init(hardwareMap);

        this.initArms(); // Initializarea bratelelor
    }

    @Override
    public void loop() {

        this.armMoveUp();
        this.armMoveLateral();

        //***************Telemetry***************

        telemetry.addData("UpValues", up);
        telemetry.addData("ForwardValues", forward);
        telemetry.addData("mV1", posmV);
        telemetry.addData("mO", posmO);
        telemetry.addLine("version 1.23.2025.4.29");

        telemetry.update();
    }
}
