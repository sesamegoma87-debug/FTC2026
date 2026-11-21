package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "BIOBUZZ_HiveTeleOp", group = "TeleOp")
public class HiveTeleOp extends OpMode {

    // ===== 🔫 タレット定数（70ギア 対 30ギア） =====
    static final double TURRET_GEAR_RATIO         = 70.0 / 30.0; // ★ 70歯 ÷ 30歯 = 2.3333...
    static final double TURRET_BASE_TICKS_PER_REV = 537.7 * TURRET_GEAR_RATIO; // 1254.63 ticks/rev
    static final int    TURRET_DIRECTION          = +1;          // 逆に回るなら -1

    // 微調整用スケール (70:30基準で 1.000。十字キー左右で 0.005 ずつ微調整可能)
    private double turretScale = 1.000;

    // ★★★ 可動範囲: 右270° (-270°) 〜 左90° (+90°) ★★★
    static final double TURRET_MIN_RAD = Math.toRadians(-270.0); // 右に270°まで
    static final double TURRET_MAX_RAD = Math.toRadians(+90.0);  // 左に90°まで
    static final double WRAP_HYST_RAD  = Math.toRadians(4.0);    // 境界でのチャタリング防止角

    // ===== 🎛️ PID & ブレーキ設定（力強い初動 ＆ 行き過ぎ防止） =====
    static final double KP = 1.50;             // 初動からしっかり回す比例ゲイン
    static final double KI = 0.40;             // 最終的な微小ズレを押し切る積分ゲイン
    static final double KD_VEL = 0.095;        // 実測回転速度に対するブレーキゲイン(行き過ぎ防止)
    static final double I_ZONE_RAD = Math.toRadians(6.0);
    static final double I_CLAMP    = 0.15;

    static final double MAX_POWER      = 0.75; // 通常追従時の最大パワー
    static final double MIN_MOVE_POWER = 0.14; // 摩擦に負けず確実に動き出す最低保証パワー
    static final double KS             = 0.11; // 静止摩擦補償
    static final double FADE_RAD       = Math.toRadians(3.0);
    static final double DEADBAND_RAD   = Math.toRadians(0.5);

    // 加速・減速スルーレート
    static final double SLEW_UP   = 12.0; // 初動の遅れをなくす素早い加速 [power/s]
    static final double SLEW_DOWN = 80.0; // 減速・逆転ブレーキは即座に反映 [power/s]

    // 左90°を超えて右(-270°)へ折り返す時の減速設定
    static final double REWIND_MAX_POWER      = 0.60;                 // 折り返し中盤の最大パワー
    static final double REWIND_BRAKE_ZONE_RAD = Math.toRadians(65.0); // 目標の手前65°に入ったら強力減速
    static final double REWIND_APPROACH_POWER = 0.16;                 // 目標直前のアプローチ上限パワー

    // ===== 🛞 オドメトリ ("odo") 定数 =====
    static final double POD_X_OFFSET_MM      = 0.0;
    static final double POD_Y_OFFSET_MM      = 0.0;
    static final double MIN_TRACKING_DIST_MM = 400.0; // Hive(0,0)から40cm以内は初期方位を維持

    // ===== 🎮 走行スティックの向き補正 =====
    static final double DRIVE_FORWARD_SIGN = +1.0;
    static final double DRIVE_STRAFE_SIGN  = +1.0;
    static final double DRIVE_TURN_SIGN    = +1.0;

    // ===== ハードウェア =====
    private DcMotor fl, bl, fr, br;
    private DcMotorEx turret;
    private GoBildaPinpointDriver pinpointOdo = null;
    private DcMotorEx encoderOdo = null;
    private IMU imu = null;
    private boolean usePinpoint = false;
    private boolean hasEncoderOdo = false;
    private CRServo sqL, sqR;
    private Shooter shooter;

    // ===== ステート変数 =====
    private double robotX = 0.0;
    private double robotY = 0.0;
    private double hiveX  = 0.0;
    private double hiveY  = 0.0;
    private int lastOdoTicks = 0;
    private double lastHeading = 0.0;

    private double initBaseRad = 0.0;
    private double targetFieldAngle = 0.0;
    private double currentSetpoint = 0.0;
    private boolean isRewinding = false;

    private double integral  = 0.0;
    private double lastError = 0.0;
    private double lastPower = 0.0;

    private boolean firing = false;
    private double shooterPower = 0.8;
    private boolean prevRb, prevU, prevD, prevL, prevR, prevYaw;

    private final ElapsedTime dtTimer = new ElapsedTime();

    @Override
    public void init() {
        // --- 走行モーター ---
        fl = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        bl = hardwareMap.get(DcMotor.class, "backLeftMotor");
        fr = hardwareMap.get(DcMotor.class, "frontRightMotor");
        br = hardwareMap.get(DcMotor.class, "backRightMotor");
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotor m : new DcMotor[]{fl, bl, fr, br}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setPower(0);
        }

        // --- タレット ---
        turret = hardwareMap.get(DcMotorEx.class, "turret_motor");
        turret.setDirection(DcMotor.Direction.FORWARD);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // INIT時の向き = 0°
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setPower(0);

        // --- オドメトリ ("odo") : Pinpoint と 通常エンコーダの両方に自動対応 ---
        try {
            pinpointOdo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
            pinpointOdo.setOffsets(POD_X_OFFSET_MM, POD_Y_OFFSET_MM, DistanceUnit.MM);
            pinpointOdo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpointOdo.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD);
            pinpointOdo.resetPosAndIMU();
            usePinpoint = true;
        } catch (Exception e) {
            usePinpoint = false;
            imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
            imu.resetYaw();
            try {
                encoderOdo = hardwareMap.get(DcMotorEx.class, "odo");
                if (encoderOdo != fl && encoderOdo != bl && encoderOdo != fr && encoderOdo != br) {
                    encoderOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    encoderOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                lastOdoTicks = encoderOdo.getCurrentPosition();
                hasEncoderOdo = true;
            } catch (Exception ex) {
                hasEncoderOdo = false;
            }
        }

        // --- Squeeze & Shooter ---
        sqL = hardwareMap.crservo.get("left_squeeze_motor");
        sqR = hardwareMap.crservo.get("right_squeeze_motor");
        sqL.setDirection(DcMotorSimple.Direction.REVERSE);
        sqL.setPower(0);
        sqR.setPower(0);

        shooter = new Shooter(hardwareMap);

        robotX = 0.0;
        robotY = 0.0;
        hiveX  = 0.0;
        hiveY  = 0.0;
        currentSetpoint = 0.0;
        isRewinding = false;
    }

    @Override
    public void init_loop() {
        double initHeading;
        if (usePinpoint) {
            pinpointOdo.update();
            initHeading = pinpointOdo.getHeading(AngleUnit.RADIANS);
        } else {
            initHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            if (hasEncoderOdo) lastOdoTicks = encoderOdo.getCurrentPosition();
        }
        lastHeading = initHeading;
        initBaseRad = initHeading;
        targetFieldAngle = initHeading;

        telemetry.addData("状態", "現在地をHive(0,0)としてロック完了");
        telemetry.addData("ギア比設定", "70歯 : 30歯 (2.333 / 1254.6 ticks)");
        telemetry.addData("タレット可動域", "左90°(+90°) 〜 右270°(-270°)");
        telemetry.addData("オドメトリ方式", usePinpoint ? "Pinpoint" : (hasEncoderOdo ? "IMU + odo" : "IMUのみ"));
        telemetry.update();
    }

    @Override
    public void loop() {
        double dt = clamp(dtTimer.seconds(), 0.002, 0.05);
        dtTimer.reset();

        // スティック入力
        double joyY  = -gamepad1.left_stick_y * DRIVE_FORWARD_SIGN;
        double joyX  =  gamepad1.left_stick_x * DRIVE_STRAFE_SIGN;
        double joyRx =  gamepad1.right_stick_x * DRIVE_TURN_SIGN;

        // 1. リセットボタン (Y または Options で現在地を新しいHive(0,0)にリセット)
        boolean yawBtn = gamepad1.options || gamepad1.y;
        if (yawBtn && !prevYaw) {
            if (usePinpoint) {
                pinpointOdo.resetPosAndIMU();
            } else {
                imu.resetYaw();
            }
            robotX = 0.0;
            robotY = 0.0;
            hiveX  = 0.0;
            hiveY  = 0.0;
            lastHeading = 0.0;
            initBaseRad = 0.0;
            targetFieldAngle = 0.0;
            isRewinding = false;
        }
        prevYaw = yawBtn;

        // 十字キー左右で回転スケールの微調整 (左: 回転角を減らす / 右: 回転角を増やす)
        boolean l = gamepad1.dpad_left, r = gamepad1.dpad_right;
        if (l && !prevL) turretScale = Math.max(0.700, turretScale - 0.005);
        if (r && !prevR) turretScale = Math.min(1.300, turretScale + 0.005);
        prevL = l; prevR = r;

        // 2. 自己位置 (robotX, robotY) と ヨー角 (heading) の取得
        double heading;
        if (usePinpoint) {
            pinpointOdo.update();
            robotX  = pinpointOdo.getPosX(DistanceUnit.MM);
            robotY  = pinpointOdo.getPosY(DistanceUnit.MM);
            heading = pinpointOdo.getHeading(AngleUnit.RADIANS);
        } else {
            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double dHeading = wrap(heading - lastHeading);
            double yawRate = Math.abs(dHeading / dt);
            if (hasEncoderOdo) {
                int nowTicks = encoderOdo.getCurrentPosition();
                int dTicks = nowTicks - lastOdoTicks;
                lastOdoTicks = nowTicks;
                boolean isTurning = (Math.abs(joyRx) > 0.05) || (yawRate > Math.toRadians(15.0));
                double dForward = isTurning ? 0.0 : (dTicks * (Math.PI * 48.0 / 8192.0));
                double midH = heading - dHeading / 2.0;
                robotX += dForward * Math.cos(midH);
                robotY += dForward * Math.sin(midH);
            }
        }
        lastHeading = heading;

        // 3. Hive(0, 0) への目標方位を計算
        double dx = hiveX - robotX;
        double dy = hiveY - robotY;
        if (Math.hypot(dx, dy) > MIN_TRACKING_DIST_MM) {
            targetFieldAngle = Math.atan2(dy, dx);
        } else {
            targetFieldAngle = initBaseRad;
        }

        // 4. 現在のタレット角と実測角速度 [rad/s]（70:30ギア比 = 1254.63 ticks/rev）
        double effectiveTicksPerRev = TURRET_BASE_TICKS_PER_REV * turretScale;
        double turretAngle  = TURRET_DIRECTION * turret.getCurrentPosition()
                / effectiveTicksPerRev * 2.0 * Math.PI;
        double turretVelRad = TURRET_DIRECTION * turret.getVelocity()
                / effectiveTicksPerRev * 2.0 * Math.PI;

        // 5. 目標タレット角を [右270°(-270°), 左90°(+90°)] の範囲で決定
        double desiredTurret = targetFieldAngle - heading;
        double setpoint = selectTurretSetpoint(desiredTurret, currentSetpoint);

        // 左90° ⇔ 右270° の折り返し（大回り）開始を検知
        if (Math.abs(setpoint - currentSetpoint) > Math.PI) {
            isRewinding = true;
            integral = 0.0;
            lastPower = 0.0;
        }
        currentSetpoint = setpoint;

        // 6. 誤差の計算
        double error  = setpoint - turretAngle;
        double absErr = Math.abs(error);

        // 目標の4°以内に収まったら折り返しモード完了
        if (isRewinding && absErr < Math.toRadians(4.0)) {
            isRewinding = false;
        }

        // 7. PID制御（初動パワー保証 ＆ 実測速度ブレーキによる行き過ぎ防止）
        double targetPower = 0.0;
        if (absErr < DEADBAND_RAD) {
            integral = 0.0;
        } else {
            // 目標を跨いだ瞬間や折り返し中は積分をゼロにしてオーバーシュートを防止
            if (error * lastError <= 0.0 || absErr > I_ZONE_RAD || isRewinding) {
                integral = 0.0;
            } else {
                integral = clamp(integral + error * dt, -I_CLAMP, I_CLAMP);
            }

            // タレットの実測速度(turretVelRad)に対する直接ブレーキ
            boolean inRewindBrakeZone = isRewinding && (absErr < REWIND_BRAKE_ZONE_RAD);
            double activeKd = inRewindBrakeZone ? (KD_VEL * 2.0) : KD_VEL;
            double dBrake = -activeKd * turretVelRad;

            double ks = KS * clamp((absErr - DEADBAND_RAD) / FADE_RAD, 0.0, 1.0);
            double rawPower = KP * error + KI * integral + dBrake + Math.signum(error) * ks;

            // 折り返し時の速度プロファイル（序盤は素早く回り、残り65°から滑らかに減速）
            double currentMaxP = MAX_POWER;
            if (isRewinding) {
                if (absErr < REWIND_BRAKE_ZONE_RAD) {
                    double ratio = clamp(absErr / REWIND_BRAKE_ZONE_RAD, 0.0, 1.0);
                    currentMaxP = REWIND_APPROACH_POWER
                            + (REWIND_MAX_POWER - REWIND_APPROACH_POWER) * ratio;
                } else {
                    currentMaxP = REWIND_MAX_POWER;
                }
            }

            // リミット(-270° / +90°)の直前30°以内ではリミット衝突防止のため上限を絞る
            double distToLimit = (error >= 0.0)
                    ? (TURRET_MAX_RAD - turretAngle)
                    : (turretAngle - TURRET_MIN_RAD);
            if (distToLimit < Math.toRadians(30.0)) {
                double limRatio = clamp(distToLimit / Math.toRadians(30.0), 0.0, 1.0);
                double limMaxP = MIN_MOVE_POWER + (currentMaxP - MIN_MOVE_POWER) * limRatio;
                currentMaxP = Math.min(currentMaxP, limMaxP);
            }

            targetPower = clamp(rawPower, -currentMaxP, currentMaxP);

            // 初動でパワーが弱すぎて回らない現象を防ぐため、誤差が2°以上ある時は最低パワー(0.14)を保証
            if (absErr > Math.toRadians(2.0) && Math.abs(turretVelRad) < Math.toRadians(15.0)) {
                if (Math.abs(targetPower) < MIN_MOVE_POWER) {
                    targetPower = Math.copySign(MIN_MOVE_POWER, error);
                }
            }
        }

        // 目標角を通り過ぎた瞬間は加速メモリを即リセットして逆転ブレーキを効かせる
        if (error * lastError < 0.0) {
            lastPower = 0.0;
        }
        lastError = error;

        // 8. 非対称スルーレート（加速は12.0/sでキビキビ、減速は80.0/sで即座にブレーキ）
        double power;
        boolean isAcceleratingSameDir = (targetPower * lastPower >= 0.0)
                && (Math.abs(targetPower) > Math.abs(lastPower));
        if (isAcceleratingSameDir) {
            double step = SLEW_UP * dt;
            power = clamp(targetPower, lastPower - step, lastPower + step);
        } else {
            double step = SLEW_DOWN * dt;
            power = clamp(targetPower, lastPower - step, lastPower + step);
        }

        // 9. リミット端(-270° / +90°)での保護
        if (turretAngle > TURRET_MAX_RAD)      power = Math.min(power, 0.0);
        else if (turretAngle < TURRET_MIN_RAD) power = Math.max(power, 0.0);

        if ((turretAngle >= TURRET_MAX_RAD && power > 0.0)
                || (turretAngle <= TURRET_MIN_RAD && power < 0.0)) {
            power = 0.0;
            integral = 0.0;
        }
        lastPower = power;
        turret.setPower(TURRET_DIRECTION * power);

        // 10. Shooter & Squeeze (RBトグル、十字キー上下で出力調整)
        boolean rb = gamepad1.right_bumper;
        if (rb && !prevRb) firing = !firing;
        prevRb = rb;

        boolean u = gamepad1.dpad_up, d = gamepad1.dpad_down;
        if (u && !prevU) shooterPower = Math.min(1.0, shooterPower + 0.05);
        if (d && !prevD) shooterPower = Math.max(0.0, shooterPower - 0.05);
        prevU = u; prevD = d;

        shooter.setPower(firing ? shooterPower : 0.0);
        shooter.update();
        sqL.setPower(firing ? 1.0 : 0.0);
        sqR.setPower(firing ? 1.0 : 0.0);

        // 11. フィールドセントリック走行
        double rotX = joyX * Math.cos(-heading) - joyY * Math.sin(-heading);
        double rotY = joyX * Math.sin(-heading) + joyY * Math.cos(-heading);
        rotX *= 1.1;
        double den = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(joyRx), 1.0);
        fl.setPower((rotY + rotX + joyRx) / den);
        bl.setPower((rotY - rotX + joyRx) / den);
        fr.setPower((rotY - rotX - joyRx) / den);
        br.setPower((rotY + rotX - joyRx) / den);

        // Telemetry
        telemetry.addData("Pos X/Y [mm]", "%.0f / %.0f", robotX, robotY);
        telemetry.addData("Heading [deg]", "%.1f", Math.toDegrees(heading));
        telemetry.addData("Turret [deg]", "%.1f / Setpoint: %.1f %s",
                Math.toDegrees(turretAngle), Math.toDegrees(setpoint), isRewinding ? "[折り返し中]" : "");
        telemetry.addData("Scale (70:30基準)", "%.3f (十字左右で微調整)", turretScale);
        telemetry.addData("Turret Power", "%.2f", power);
        telemetry.addData("Firing", "%s (Power: %.2f)", firing ? "ON" : "OFF", shooterPower);
        telemetry.update();
    }

    @Override
    public void stop() {
        fl.setPower(0); bl.setPower(0); fr.setPower(0); br.setPower(0);
        turret.setPower(0); sqL.setPower(0); sqR.setPower(0); shooter.setPower(0);
    }

    // --- ヘルパーメソッド ---
    private static double wrap(double a) {
        while (a >  Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    /**
     * 目標角度を [右270°(-270°), 左90°(+90°)] に変換する
     */
    private static double selectTurretSetpoint(double desiredRad, double prevSetpoint) {
        double unwrapped = prevSetpoint + wrap(desiredRad - prevSetpoint);

        if (unwrapped >= TURRET_MIN_RAD && unwrapped <= TURRET_MAX_RAD) {
            return unwrapped;
        }

        if (unwrapped > TURRET_MAX_RAD && unwrapped <= TURRET_MAX_RAD + WRAP_HYST_RAD) {
            return TURRET_MAX_RAD;
        }
        if (unwrapped < TURRET_MIN_RAD && unwrapped >= TURRET_MIN_RAD - WRAP_HYST_RAD) {
            return TURRET_MIN_RAD;
        }

        double wrapped = unwrapped;
        while (wrapped < TURRET_MIN_RAD) wrapped += 2.0 * Math.PI;
        while (wrapped > TURRET_MAX_RAD) wrapped -= 2.0 * Math.PI;
        return clamp(wrapped, TURRET_MIN_RAD, TURRET_MAX_RAD);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}