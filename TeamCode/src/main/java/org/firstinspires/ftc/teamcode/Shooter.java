package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** モーター単体テスト用（ギア比なし・高精細速度取得版） */
public class Shooter {
    // goBILDA 5203 (1150 RPM) の出力軸1回転あたりのカウント数
    private static final double PPR = 145.1;

    private final DcMotorEx motor;
    private double ticksPerSec = 0;
    private double motorRps = 0; // 1秒あたりの回転数 (回転/s)
    private double motorRpm = 0; // 1分あたりの回転数 (RPM)

    public Shooter(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "shoot_motor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // 生パワー制御
        motor.setDirection(DcMotor.Direction.FORWARD);      // 逆ならREVERSE
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setPower(0);
    }

    public void setPower(double p) {
        motor.setPower(Math.max(-1.0, Math.min(1.0, p)));
    }

    /** loop()ごとに呼ぶ。Control Hub内部の正確な速度を取得 */
    public void update() {
        // 1秒間に進んだエンコーダーのカウント数を直接取得（通信ズレの影響を受けない）
        ticksPerSec = motor.getVelocity();

        // 1秒あたりの回転数 (RPS) = 1秒のカウント数 ÷ 1回転のカウント数
        motorRps = ticksPerSec / PPR;

        // 1分あたりの回転数 (RPM) = 1秒あたりの回転数 × 60秒
        motorRpm = motorRps * 60.0;
    }

    public double getTicksPerSec() { return ticksPerSec; }
    public double getMotorRps()    { return motorRps; }
    public double getMotorRpm()    { return motorRpm; }
}