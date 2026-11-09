package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="slowspeed SLOW (Field Oriented)", group="TeleOp")
public class slowspeed extends LinearOpMode {

    // ★スピード上限の初期値 (0.0〜1.0)
    static final double INITIAL_SPEED = 0.5; // 起動時は50%
    static final double SPEED_STEP = 0.05;   // 十字キー1回で5%変化

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        double speedScale = INITIAL_SPEED;

        // ★前回のボタンの状態を記憶する変数
        boolean lastDpadUp = false;
        boolean lastDpadDown = false;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.options || gamepad1.y) {
                imu.resetYaw();
            }

            // ★今のボタンの状態を取得
            boolean currentDpadUp = gamepad1.dpad_up;
            boolean currentDpadDown = gamepad1.dpad_down;

            // ★押した「瞬間」だけ反応する (エッジ検出)
            // 「今は押されている(current=true)」かつ「前は押されていなかった(last=false)」
            if (currentDpadUp && !lastDpadUp) {
                speedScale = Math.min(1.0, speedScale + SPEED_STEP);
            }
            if (currentDpadDown && !lastDpadDown) {
                speedScale = Math.max(0.1, speedScale - SPEED_STEP);
            }

            // ★今回の状態を、次回の「前回の状態」として記憶させる
            lastDpadUp = currentDpadUp;
            lastDpadDown = currentDpadDown;

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower * speedScale);
            backLeftMotor.setPower(backLeftPower * speedScale);
            frontRightMotor.setPower(frontRightPower * speedScale);
            backRightMotor.setPower(backRightPower * speedScale);

            telemetry.addData("--- zettai SLOW ---", "");
            telemetry.addData("Speed Scale", "%.0f%%", speedScale * 100);
            telemetry.addData("Yaw (Degrees)", "%.1f", Math.toDegrees(botHeading));
            telemetry.addData("Stick (x, y)", "(%.2f, %.2f)", x, y);
            telemetry.addData("Adjust Speed", "Dpad Up/Down (1 click = 5%)");
            telemetry.addData("Reset Yaw", "Press Y or Options");
            telemetry.update();
        }
    }
}