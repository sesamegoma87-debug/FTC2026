package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class Step1_PControlTurn extends LinearOpMode {

    DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontLeftMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backRightMotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        waitForStart();
        if (isStopRequested()) return;

        turnPControl(90); // 90度回転してみるテスト
    }

    void turnPControl(double targetAngle) {
        double maxSpeedAngle = 30; // このズレ以内から減速し始める
        double minSpeed = 0.15;    // 最低速度(止まりきらないように)

        while (opModeIsActive()) {
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = targetAngle - currentHeading; // ← ここがP制御の核心:誤差の計算

            if (Math.abs(error) < 1.5) break; // 誤差が十分小さくなったら終了

            // 誤差が大きいほど強く、小さいほど弱く(P制御そのもの)
            double power = Math.max(Math.abs(error) / maxSpeedAngle, minSpeed);
            power = Math.min(power, 1.0);
            double direction = Math.signum(error); // 誤差がプラスかマイナスかで回転方向を決める

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