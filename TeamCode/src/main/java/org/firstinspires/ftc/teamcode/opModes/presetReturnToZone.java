package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware;

@TeleOp(name = "presetReturnToZone", group = "Preset")
public class presetReturnToZone extends OpMode {

    //***************Declaration-of-values***************
    hardware robot = new hardware(); // Robot hardware object
    double leftXJoystick1, leftYJoystick1, rightXJoystick1; // Joystick values
    double motorFRvolt, motorFLvolt, motorBRvolt, motorBLvolt;

    // Odometry variables
    private double leftEncoderPos = 0, rightEncoderPos = 0, centerEncoderPos = 0;
    private double prevLeftEncoderPos = 0, prevRightEncoderPos = 0, prevCenterEncoderPos = 0;

    private double xPos = 0; // X position in cm
    private double yPos = 0; // Y position in cm
    private double heading = 0; // Heading in radians

    private final double TRACKWIDTH_CM = 30.0; // Distance between left and right encoders
    private final double FORWARD_OFFSET_CM = 10.0; // Offset of the center encoder from the robot's center

    private double headingDegrees = 0; // Heading in degrees for telemetry

    //***************Methods***************

    // Method to get the current position of the left encoder
    public double getLeftEncoderPosition() {
        return robot.odoL.getCurrentPosition();
    }

    // Method to get the current position of the right encoder
    public double getRightEncoderPosition() {
        return robot.odoR.getCurrentPosition();
    }

    // Method to get the current position of the center encoder
    public double getCenterEncoderPosition() {
        return robot.odoA.getCurrentPosition();
    }

    // Method to calculate odometry
    public void calcOdo() {
        // Get the current encoder positions
        leftEncoderPos = getLeftEncoderPosition();
        rightEncoderPos = getRightEncoderPosition();
        centerEncoderPos = getCenterEncoderPosition();

        // Calculate changes in encoder positions
        double deltaLeft = leftEncoderPos - prevLeftEncoderPos;
        double deltaRight = rightEncoderPos - prevRightEncoderPos;
        double deltaCenter = centerEncoderPos - prevCenterEncoderPos;

        // Calculate the change in heading (phi) in radians
        double phi = (deltaLeft - deltaRight) / TRACKWIDTH_CM;

        // Calculate the changes in forward (middle) and lateral (perpendicular) movements
        double deltaMiddle = (deltaLeft + deltaRight) / 2.0;
        double deltaPerp = deltaCenter - FORWARD_OFFSET_CM * phi;

        // Transform local changes to global X and Y using the current heading
        double deltaX = deltaMiddle * Math.cos(heading) - deltaPerp * Math.sin(heading);
        double deltaY = deltaMiddle * Math.sin(heading) + deltaPerp * Math.cos(heading);

        // Update global positions
        xPos += deltaX;
        yPos += deltaY;
        heading += phi;

        // Update heading in degrees
        headingDegrees = Math.toDegrees(heading);

        // Store the current encoder positions as previous for the next calculation
        prevLeftEncoderPos = leftEncoderPos;
        prevRightEncoderPos = rightEncoderPos;
        prevCenterEncoderPos = centerEncoderPos;
    }

    // Verify if motor values are within the range [-1, +1]
    public boolean verifyMotorValues(double value) {
        return value >= -1 && value <= 1;
    }

    // Method to set motor powers by specific values
    public void moveMotorsByValues(double FRmotorValue, double FLmotorValue, double BRmotorValue, double BLmotorValue) {
        if (this.verifyMotorValues(FRmotorValue) && this.verifyMotorValues(FLmotorValue) &&
                this.verifyMotorValues(BRmotorValue) && this.verifyMotorValues(BLmotorValue)) {
            robot.fr.setPower(FRmotorValue);
            robot.fl.setPower(FLmotorValue);
            robot.br.setPower(BRmotorValue);
            robot.bl.setPower(BLmotorValue);
        }
    }

    // Method to control drivetrain using joystick inputs
    public void moveDriveTrain() {
        leftYJoystick1 = -gamepad1.left_stick_y; // Forward/Backward
        leftXJoystick1 = -gamepad1.left_stick_x; // Left/Right
        rightXJoystick1 = -gamepad1.right_stick_x; // Rotation

        double denominator = Math.max(Math.abs(leftYJoystick1) + Math.abs(leftXJoystick1) + Math.abs(rightXJoystick1), 1);

        // Calculate motor powers
        motorFRvolt = (leftYJoystick1 - leftXJoystick1 - rightXJoystick1) / denominator;
        motorFLvolt = (leftYJoystick1 + leftXJoystick1 + rightXJoystick1) / denominator;
        motorBRvolt = (leftYJoystick1 + leftXJoystick1 - rightXJoystick1) / denominator;
        motorBLvolt = (leftYJoystick1 - leftXJoystick1 + rightXJoystick1) / denominator;

        // Set motor powers
        this.moveMotorsByValues(motorFRvolt, motorFLvolt, motorBRvolt, motorBLvolt);
    }

    public void resetOriz() {
        if (Math.abs(xPos) == xPos) {
            while (xPos != 0) {
                this.moveMotorsByValues(-1, 1, 1, -1);
            }
        } else {
            while (xPos != 0) {
                this.moveMotorsByValues(1, -1, -1, 1);
            }
        }
    }

    public void resetVert() {
        if (Math.abs(yPos) == yPos) {
            while (yPos != 0) {
                this.moveMotorsByValues(1, 1, 1, 1);
            }
        } else {
            while (yPos != 0) {
                this.moveMotorsByValues(-1, -1, -1, -1);
            }
        }
    }

    public void resetPosition(){
        if(gamepad1.square){
            this.resetOriz();
            this.resetVert();
        }
    }

    //***************Main-methods***************

    @Override
    public void init() {
        // Initialize robot hardware
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        // Update odometry calculations
        this.calcOdo();

        // Control the drivetrain
        this.moveDriveTrain();

        this.resetPosition();

        //***************Telemetry***************
        telemetry.addData("MotorFR", motorFRvolt);
        telemetry.addData("MotorFL", motorFLvolt);
        telemetry.addData("MotorBR", motorBRvolt);
        telemetry.addData("MotorBL", motorBLvolt);
        telemetry.addData("X Position (cm)", xPos);
        telemetry.addData("Y Position (cm)", yPos);
        telemetry.addData("Heading (degrees)", headingDegrees);
        telemetry.addLine("version 1.24.2025.8.40");

        telemetry.update();
    }
}
