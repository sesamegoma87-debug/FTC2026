package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx; // 【重要】Ex付きのクラスを使います
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Shoot Control (Velocity PID)", group = "Iterative Opmode")
public class shootcontrol extends OpMode {

    // DcMotorではなく DcMotorEx を使います（速度制御のために必要）
    private DcMotorEx shootMotor;
    private Servo shootServo;

    private boolean isShooting = false;
    private boolean wasCirclePressed = false;
    private boolean isServoLocked = false;
    private boolean wasXPressed = false;

    // サーボの位置定義
    private final double SERVO_LOCK_POSITION = 0.0;
    private final double SERVO_RELEASE_POSITION = 1.0;

    // 【超重要】目標とする回転数 (RPM)
    // 1150rpmモーターなら、負荷を考慮して 1000〜1100 くらいに設定するのが現実的です
    private final double TARGET_RPM = 1050.0;

    @Override
    public void init() {
        // DcMotorEx としてモーターを取得
        shootMotor = hardwareMap.get(DcMotorEx.class, "shoot_motor");
        shootServo = hardwareMap.servo.get("shoot_servo");

        // 【重要】エンコーダーを使って速度を維持するモードに変更
        shootMotor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);

        shootServo.setPosition(SERVO_LOCK_POSITION);
    }


    @Override
    public void loop() {
        // --- 1. モーターの制御 (Circleボタン) ---
        boolean isCirclePressed = gamepad1.circle;
        if (isCirclePressed && !wasCirclePressed) {
            isShooting = !isShooting;
        }
        wasCirclePressed = isCirclePressed;

        if (isShooting) {
            // 【魔法の命令】setVelocity を使うと、SDK内部が自動でPID制御をして
            // ボールを射出する負荷がかかっても、TARGET_RPMを維持しようと頑張ってくれます！
            shootMotor.setVelocity(TARGET_RPM);
        } else {
            shootMotor.setPower(0.0); // 止めるときは通常通り power 0
        }

        // --- 2. サーボの制御 (Xボタン) ---
        boolean isXPressed = gamepad1.x;
        if (isXPressed && !wasXPressed) {
            isServoLocked = !isServoLocked;
        }
        wasXPressed = isXPressed;

        if (isServoLocked) {
            shootServo.setPosition(SERVO_LOCK_POSITION);
        } else {
            shootServo.setPosition(SERVO_RELEASE_POSITION);
        }

        // --- 3. 画面に表示（デバッグ用） ---
        // 現在の実際のRPMを取得して表示（setVelocityの凄さがここでわかります）
        double currentRpm = shootMotor.getVelocity();

        telemetry.addData("Motor", isShooting ? "RUNNING 🔄" : "STOPPED ⏹️");
        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.addData("Current RPM", "%.0f", currentRpm); // 負荷がかかってもTARGET_RPMに張り付くはず！
        telemetry.addData("Servo", isServoLocked ? "LOCKED 🔒" : "RELEASED 🔓");
        telemetry.update();
    }
}