package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** シューター部品クラス。TeleOp/Autonomousから生成して使う */
public class Shooter {
    /** ホイール1回転あたりのtick = 28(モーター軸counts/rev) × 5.2(減速比) ÷ 3(増速比)
     *  [要確認] 1150rpm版(5.2:1)の仮定。1620rpm版なら 103.6/3.0 に差し替え */
    private static final double TICKS_PER_WHEEL_REV = 145.6 / 3.0; // ≈48.53
    /** ホイール側無負荷上限 = 1150 × 3 [要確認] 1620rpm版なら4860 */
    private static final double MAX_WHEEL_RPM = 3450.0;
    private static final double READY_TOL_RPM = 100.0;

    private final DcMotorEx motor;
    private double targetRpm = 0;

    public Shooter(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "shoot_motor"); // Configurationの名前と完全一致
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setDirection(DcMotor.Direction.FORWARD); // [要確認] 逆回転ならREVERSE
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /** 目標rpm。0で惰行停止 */
    public void setTargetRpm(double rpm) {
        targetRpm = Math.max(0, Math.min(rpm, MAX_WHEEL_RPM));
        if (targetRpm == 0) {
            motor.setPower(0);
        } else {
            motor.setVelocity(targetRpm * TICKS_PER_WHEEL_REV / 60.0);
        }
    }

    public double getRpm() {
        return motor.getVelocity() * 60.0 / TICKS_PER_WHEEL_REV;
    }

    /** 目標±許容誤差に入ったらtrue。フィードの許可に使う */
    public boolean isReady() {
        return targetRpm > 0 && Math.abs(getRpm() - targetRpm) <= READY_TOL_RPM;
    }

    public double getTargetRpm() { return targetRpm; }
}