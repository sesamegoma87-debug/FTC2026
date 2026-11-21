package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "ShooterTest", group = "Test")
public class ShooterTeleOp extends OpMode {
    private DcMotorEx motor;
    private double power = 1.0;
    private boolean spin = false, prevRb = false, prevU = false, prevD = false;

    // 1150RPMモデルの定数
    private static final double PPR = 145.1;

    @Override
    public void init() {
        // コンフィグから "shoot_motor" を探す
        motor = hardwareMap.get(DcMotorEx.class, "shoot_motor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setPower(0);

        // INITを押した時点で画面に文字を出す
        telemetry.addData("状態", "準備OK！ ▶(再生ボタン)を押してください");
        telemetry.update();
    }

    @Override
    public void loop() {
        // コントローラー操作（RBで回転ON/OFF、十字上下でパワー調整）
        boolean rb = gamepad1.right_bumper;
        boolean u  = gamepad1.dpad_up;
        boolean d  = gamepad1.dpad_down;

        if (rb && !prevRb) spin = !spin;
        if (u && !prevU)   power = Math.min(1.0, power + 0.05);
        if (d && !prevD)   power = Math.max(0.0, power - 0.05);
        prevRb = rb; prevU = u; prevD = d;

        // モーターを回す
        double currentPower = spin ? power : 0.0;
        motor.setPower(currentPower);

        // エンコーダーの値を取得
        int currentPos = motor.getCurrentPosition(); // 現在の位置
        double ticksPerSec = motor.getVelocity();    // 1秒間の変化量
        double rps = ticksPerSec / PPR;              // 回転/秒
        double rpm = rps * 60.0;                     // 回転/分

        // 画面に表示
        telemetry.addData("状態", spin ? "回転中！" : "停止中 (RBボタンでスタート)");
        telemetry.addData("1. パワー", "%.2f", currentPower);
        telemetry.addData("2. エンコーダー現在位置", currentPos);
        telemetry.addData("3. 速度 (回転/秒)", "%.1f RPS", rps);
        telemetry.addData("4. 速度 (回転/分)", "%.0f RPM", rpm);
        telemetry.update();
    }
}