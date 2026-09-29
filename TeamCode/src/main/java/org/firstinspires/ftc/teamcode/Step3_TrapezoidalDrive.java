package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class Step3_TrapezoidalDrive extends LinearOpMode {

    DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    static final double TICKS_PER_INCH = 40; // 実機に合わせて要調整

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (isStopRequested()) return;

        driveStraightTrapezoidal(24, 0.7); // 24インチ前進
    }

    void driveStraightTrapezoidal(double inches, double maxPower) {
        int targetTicks = (int) (inches * TICKS_PER_INCH);

        frontLeftMotor.setTargetPosition(targetTicks);
        backLeftMotor.setTargetPosition(targetTicks);
        frontRightMotor.setTargetPosition(targetTicks);
        backRightMotor.setTargetPosition(targetTicks);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double accelDistanceTicks = 400; // ここまでの距離で徐々に加速
        double decelDistanceTicks = 400; // ゴール手前この距離から徐々に減速

        while (opModeIsActive() &&
                frontLeftMotor.isBusy() && backLeftMotor.isBusy() &&
                frontRightMotor.isBusy() && backRightMotor.isBusy()) {

            int currentTicks = Math.abs(frontLeftMotor.getCurrentPosition());
            int remainingTicks = Math.abs(targetTicks) - currentTicks;

            double speedFactor;
            if (currentTicks < accelDistanceTicks) {
                speedFactor = currentTicks / accelDistanceTicks;      // 加速フェーズ(台形の左斜辺)
            } else if (remainingTicks < decelDistanceTicks) {
                speedFactor = remainingTicks / decelDistanceTicks;    // 減速フェーズ(台形の右斜辺)
            } else {
                speedFactor = 1.0;                                    // 巡航フェーズ(台形の上辺)
            }
            speedFactor = Math.max(0.2, Math.min(1.0, speedFactor));

            double power = maxPower * speedFactor;

            frontLeftMotor.setPower(power);
            backLeftMotor.setPower(power);
            frontRightMotor.setPower(power);
            backRightMotor.setPower(power);
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