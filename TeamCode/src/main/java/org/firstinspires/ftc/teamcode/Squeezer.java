package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** ボール押し込み用の連続回転サーボ2個 */
public class Squeezer {
    private final CRServo left, right;

    public Squeezer(HardwareMap hardwareMap) {
        left = hardwareMap.crservo.get("left_squeeze_motor");
        right = hardwareMap.crservo.get("right_squeeze_motor");

        // 左右は向かい合って付くので、片方を反転して「ボールを押し込む向き」にそろえる。
        // 逆に回ったら、REVERSE をつける側を入れ替える(または両方反転/両方なし)。
        right.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /** 100%で回し続ける(stop() を呼ぶまで止まらない) */
    public void start() {
        left.setPower(1.0);
        right.setPower(1.0);
    }

    /** 逆回転(詰まったボールを戻す用) */
    public void reverse() {
        left.setPower(-1.0);
        right.setPower(-1.0);
    }

    public void stop() {
        left.setPower(0);
        right.setPower(0);
    }
}