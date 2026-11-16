package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TurretRampTest", group = "Test")
public class TurretRampTest extends LinearOpMode {

    private static final String MOTOR_NAME = "turret_motor";
    private static final double MAX_POWER = 0.75;  // 上限パワー
    private static final double RAMP_SEC  = 3.0;  // 0 → MAX_POWER に掛ける秒数

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setDirection(DcMotor.Direction.FORWARD);   // 逆なら REVERSE
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setPower(0);

        telemetry.addData("motor", MOTOR_NAME);
        telemetry.addData("ramp", "%.1f s -> %.2f", RAMP_SEC, MAX_POWER);
        telemetry.update();

        waitForStart();

        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive()) {
            double ratio = Math.min(1.0, t.seconds() / RAMP_SEC);
            // もっと緩やかに立ち上げたいなら ratio * ratio にする
            double power = MAX_POWER * ratio;
            turret.setPower(power);

            telemetry.addData("time", "%.2f s", t.seconds());
            telemetry.addData("power", "%.3f", power);
            telemetry.update();
            sleep(20);
        }
        turret.setPower(0);
    }
}