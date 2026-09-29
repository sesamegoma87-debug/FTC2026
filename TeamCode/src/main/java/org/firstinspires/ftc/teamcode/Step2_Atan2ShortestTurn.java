package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class Step2_Atan2ShortestTurn extends LinearOpMode {

    DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    IMU imu;

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

        turnToHeading(-170); // 極端な角度でテスト。①だと大回りするはず
    }

    // ===== atan2で、どんな角度でも必ず最短経路(-180〜180度)を計算 =====
    double shortestAngleDifference(double fromAngle, double toAngle) {
        double diff = toAngle - fromAngle;
        return Math.toDegrees(Math.atan2(
                Math.sin(Math.toRadians(diff)),
                Math.cos(Math.toRadians(diff))
        ));
    }

    void turnToHeading(double targetAngle) {
        double maxSpeedAngle = 30;
        double minSpeed = 0.15;

        while (opModeIsActive()) {
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            // ①との違いはここだけ:単純な引き算ではなくatan2で最短差を求める
            double error = shortestAngleDifference(currentHeading, targetAngle);

            if (Math.abs(error) < 1.5) break;

            double power = Math.max(Math.abs(error) / maxSpeedAngle, minSpeed);
            power = Math.min(power, 1.0);
            double direction = Math.signum(error);

            frontLeftMotor.setPower(-power * direction);
            backLeftMotor.setPower(-power * direction);
            frontRightMotor.setPower(power * direction);
            backRightMotor.setPower(power * direction);
        }

        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}