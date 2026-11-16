package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** シューター単体検証用TeleOp。right_bumperで回転トグル */
@TeleOp(name = "ShooterTest", group = "Test")
public class ShooterTeleOp extends OpMode {
    private static final double TARGET_RPM = 3300; // 初回は500に変更して確認。収束しなければ2900へ

    private Shooter shooter;
    private boolean spinning = false;
    private boolean prevRb = false;

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap);
    }

    @Override
    public void loop() {
        boolean rb = gamepad1.right_bumper;
        if (rb && !prevRb) {          // 押した瞬間だけ反応（エッジ検出）
            spinning = !spinning;
            shooter.setTargetRpm(spinning ? TARGET_RPM : 0);
        }
        prevRb = rb;

        // 供給機構ができたら: if (shooter.isReady()) { フィード動作 }

        telemetry.addData("shoot rpm", "%.0f / %.0f", shooter.getRpm(), shooter.getTargetRpm());
        telemetry.addData("ready", shooter.isReady());
        telemetry.update();
    }
}