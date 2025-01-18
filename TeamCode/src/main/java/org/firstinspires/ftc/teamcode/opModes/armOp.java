package org.firstinspires.ftc.teamcode.opModes;

// Import-uri din SDK
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// Import-uri fisiere proprii
import org.firstinspires.ftc.teamcode.hardware;

@TeleOp(name = "ArmTeleOP", group = "TeleOP")
public class armOp extends OpMode {

    // Create a hardware object to interact with the hardware
    hardware robot = new hardware();
    int posmV, posmO = 0;
    double up, forward;
    int limMax = -5300;
    int limMin = -100;
    int limMaxOriz = -2000;
    int limMinOriz = -100;

    @Override
    public void init() {
        // Initialize the robot hardware
        robot.init(hardwareMap);

        robot.mV1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mV1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.mV2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mV2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.mO.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mO.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

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

    @Override
    public void loop() {

        this.armMoveUp();
        this.armMoveLateral();

        telemetry.addData("UpValues", up);
        telemetry.addData("ForwardValues", forward);
        telemetry.addData("mV1", posmV);
        telemetry.addData("mO", posmO);

        telemetry.update();
    }
}
