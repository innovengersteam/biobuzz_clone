
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "BioBuzz TeleOp", group = "BioBuzz")
public class BioBuzzTeleOp extends LinearOpMode {

    // ==========================================
    // 1. HARDWARE
    // ==========================================

    // Control Hub: drive motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;

    // Expansion Hub: mechanism motors
    private DcMotor intake;
    private DcMotorEx outtake;

    // Control Hub: continuous rotation servo
    private CRServo transfer;

    // ==========================================
    // 2. SETTINGS
    // ==========================================

    private static final double DRIVE_SPEED = 0.75;
    private static final double INTAKE_POWER = 1.0;
    private static final double TRANSFER_POWER = 1.0;

    private static final double START_RPM = 2500;
    private static final double RPM_STEP = 50;
    private static final double MAX_RPM = 6000;

    private double targetRPM = START_RPM;

    // ==========================================
    // 3. TOGGLE STATES
    // ==========================================

    private boolean intakeOn = false;
    private boolean transferOn = false;
    private boolean outtakeOn = false;

    // Previous button states for press detection
    private boolean lastLeftTrigger = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightTrigger = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    // ==========================================
    // 4. MAIN TELEOP PROGRAM
    // ==========================================

    @Override
    public void runOpMode() {

        // Get motors from robot configuration
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        transfer = hardwareMap.get(CRServo.class, "transfer");

        // Reverse left-side motors for mecanum drive.
        // Adjust directions if physical wiring requires it.
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        outtake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);

        // Brake drive motors when joystick is released
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Use encoder feedback for outtake RPM control
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Read encoder ticks per output shaft revolution
        double ticksPerRev = outtake.getMotorType().getTicksPerRev();

        if (ticksPerRev <= 0) {
            telemetry.addLine("ERROR: Check outtake motor type.");
            telemetry.update();
            return;
        }

        // Start all mechanisms stopped
        intake.setPower(0);
        transfer.setPower(0);
        outtake.setVelocity(0);

        telemetry.addLine("BioBuzz Robot Initialized");
        telemetry.addLine("Press PLAY to start");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        // ======================================
        // 5. MAIN CONTROL LOOP
        // ======================================

        try {
            while (opModeIsActive()) {

                // ----------------------------------
                // DRIVE: LEFT AND RIGHT JOYSTICKS
                // ----------------------------------

                double forward = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;

                // Mecanum wheel power calculations
                double lf = forward + strafe + turn;
                double rf = forward - strafe - turn;
                double lr = forward - strafe + turn;
                double rr = forward + strafe - turn;

                // Normalize to keep motor powers within range
                double max = Math.max(1.0,
                        Math.max(Math.abs(lf),
                                Math.max(Math.abs(rf),
                                        Math.max(Math.abs(lr), Math.abs(rr)))));

                leftFront.setPower(lf / max * DRIVE_SPEED);
                rightFront.setPower(rf / max * DRIVE_SPEED);
                leftRear.setPower(lr / max * DRIVE_SPEED);
                rightRear.setPower(rr / max * DRIVE_SPEED);

                // ----------------------------------
                // LEFT TRIGGER: TOGGLE INTAKE
                // ----------------------------------

                boolean leftTrigger = gamepad1.left_trigger > 0.5;

                if (leftTrigger && !lastLeftTrigger) {
                    intakeOn = !intakeOn;
                }

                lastLeftTrigger = leftTrigger;

                intake.setPower(intakeOn ? INTAKE_POWER : 0);

                // ----------------------------------
                // LEFT BUMPER: TOGGLE TRANSFER
                // ----------------------------------

                boolean leftBumper = gamepad1.left_bumper;

                if (leftBumper && !lastLeftBumper) {
                    transferOn = !transferOn;
                }

                lastLeftBumper = leftBumper;

                transfer.setPower(transferOn ? TRANSFER_POWER : 0);

                // ----------------------------------
                // RIGHT TRIGGER: TOGGLE OUTTAKE
                // ----------------------------------

                boolean rightTrigger = gamepad1.right_trigger > 0.5;

                if (rightTrigger && !lastRightTrigger) {
                    outtakeOn = !outtakeOn;
                }

                lastRightTrigger = rightTrigger;

                // ----------------------------------
                // D-PAD UP: INCREASE RPM BY 50
                // ----------------------------------

                boolean dpadUp = gamepad1.dpad_up;

                if (dpadUp && !lastDpadUp) {
                    targetRPM += RPM_STEP;
                }

                lastDpadUp = dpadUp;

                // ----------------------------------
                // D-PAD DOWN: DECREASE RPM BY 50
                // ----------------------------------

                boolean dpadDown = gamepad1.dpad_down;

                if (dpadDown && !lastDpadDown) {
                    targetRPM -= RPM_STEP;
                }

                lastDpadDown = dpadDown;

                // Keep RPM in the permitted range
                targetRPM = Range.clip(targetRPM, 0, MAX_RPM);

                // Convert RPM to encoder ticks per second
                double ticksPerSecond =
                        (targetRPM / 60.0) * ticksPerRev;

                // Apply outtake velocity
                if (outtakeOn) {
                    outtake.setVelocity(ticksPerSecond);
                } else {
                    outtake.setVelocity(0);
                }

                // ----------------------------------
                // TELEMETRY
                // ----------------------------------

                double actualRPM =
                        (outtake.getVelocity() / ticksPerRev) * 60.0;

                telemetry.addLine("=== BIOBUZZ TELEOP ===");
                telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
                telemetry.addData("Transfer", transferOn ? "ON" : "OFF");
                telemetry.addData("Outtake", outtakeOn ? "ON" : "OFF");
                telemetry.addData("Target RPM", "%.0f", targetRPM);
                telemetry.addData("Actual RPM", "%.0f", actualRPM);
                telemetry.update();

            }
        } finally {
            // Stop everything when TeleOp ends
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);

            intake.setPower(0);
            transfer.setPower(0);
            outtake.setVelocity(0);
        }
    }
}
