package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class Step4_TrapezoidalDriveWithHeadingCorrection extends LinearOpMode {

    DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    IMU imu;
    static final double TICKS_PER_INCH = 40;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        waitForStart();
        if (isStopRequested()) return;

        driveStraight(24, 0.7);
    }

    double shortestAngleDifference(double fromAngle, double toAngle) {
        double diff = toAngle - fromAngle;
        return Math.toDegrees(Math.atan2(
                Math.sin(Math.toRadians(diff)),
                Math.cos(Math.toRadians(diff))
        ));
    }

    void driveStraight(double inches, double maxPower) {
        int targetTicks = (int) (inches * TICKS_PER_INCH);

        frontLeftMotor.setTargetPosition(targetTicks);
        backLeftMotor.setTargetPosition(targetTicks);
        frontRightMotor.setTargetPosition(targetTicks);
        backRightMotor.setTargetPosition(targetTicks);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double startHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double accelDistanceTicks = 400;
        double decelDistanceTicks = 400;
        double correctionGain = 0.02; // 補正の強さ。実機に合わせて調整

        while (opModeIsActive() &&
                frontLeftMotor.isBusy() && backLeftMotor.isBusy() &&
                frontRightMotor.isBusy() && backRightMotor.isBusy()) {

            int currentTicks = Math.abs(frontLeftMotor.getCurrentPosition());
            int remainingTicks = Math.abs(targetTicks) - currentTicks;

            // ---- 台形制御(③と同じ) ----
            double speedFactor;
            if (currentTicks < accelDistanceTicks) {
                speedFactor = currentTicks / accelDistanceTicks;
            } else if (remainingTicks < decelDistanceTicks) {
                speedFactor = remainingTicks / decelDistanceTicks;
            } else {
                speedFactor = 1.0;
            }
            speedFactor = Math.max(0.2, Math.min(1.0, speedFactor));
            double power = maxPower * speedFactor;

            // ---- ここが追加部分:IMU + P制御で向きのズレを補正 ----
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double headingError = shortestAngleDifference(startHeading, currentHeading);
            double correction = headingError * correctionGain;

            // 左右のパワーに補正をプラス/マイナスして向きを微調整
            frontLeftMotor.setPower(power - correction);
            backLeftMotor.setPower(power - correction);
            frontRightMotor.setPower(power + correction);
            backRightMotor.setPower(power + correction);
        }

        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}