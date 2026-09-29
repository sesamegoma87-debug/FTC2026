package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Autonomous用の走行部品クラス(メカナム4輪)。
 *
 * ・driveStraight : 台形制御 + IMUでの向き補正(P制御)
 * ・turnToHeading : PD制御(位置と速度のフィードバック) + 最短経路(atan2)
 *
 * 角度はIMUのヨー角で、上から見て「反時計回りがプラス」。
 *   turnToHeading(90)  → 左(反時計回り)に90度
 *   turnToHeading(-90) → 右(時計回り)に90度
 * 0度は、このクラスを作った瞬間(コンストラクタ)の向き。
 */
public class Drivetrain {

    private final DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private final IMU imu;
    private final LinearOpMode opMode;
    private final Telemetry telemetry;

    // TODO: 実機でキャリブレーションが必要。
    // driveStraight(24, 0.5) を動かして実測し、
    // 新しい値 = 今の値 × (命令した距離 ÷ 実際に進んだ距離) に書き換える。
    static final double TICKS_PER_INCH = 29.6;

    // ---- 直進の調整値 ----
    static final double ACCEL_DISTANCE_TICKS = 400;  // この距離で加速しきる
    static final double DECEL_DISTANCE_TICKS = 400;  // ゴール手前この距離から減速
    static final double MIN_SPEED_FACTOR = 0.2;      // 最低速度の割合
    static final double HEADING_CORRECTION_GAIN = 0.02; // 向き補正の強さ
    static final double MAX_CORRECTION = 0.3;        // 補正の上限

    // ---- 旋回の調整値(PD制御) ----
    static final double TURN_KP = 1.0 / 40.0;      // P: 誤差1度あたりのパワー(バネの強さ)
    static final double TURN_KD = 0.003;           // D: 回転の速さ(度/秒)あたりのブレーキ(ダンパー)
    static final double TURN_MIN_POWER = 0.12;     // 最低回転パワー(静摩擦を越えるため)
    static final double TURN_TOLERANCE_DEG = 0.1;  // 完了とみなす角度誤差
    static final double TURN_SETTLE_RATE = 5.0;    // 完了とみなす回転の速さ(度/秒)
    static final double TURN_SETTLE_TIME = 0.3;   // この時間ずっと落ち着いていたら完了

    // ---- 安全装置(センサー故障などで止まらなくなるのを防ぐ) ----
    static final double DRIVE_TIMEOUT_SEC = 10;
    static final double TURN_TIMEOUT_SEC = 5;

    public Drivetrain(HardwareMap hardwareMap, LinearOpMode opMode, Telemetry telemetry) {
        this.opMode = opMode;
        this.telemetry = telemetry;

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // TeleOpと同じ向き(左側を反転)。TeleOpと必ず揃えること。
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();
    }

    // ===== 直進: 台形制御 + IMU補正(inchesがマイナスなら後退) =====
    public void driveStraight(double inches, double maxPower) {
        resetEncoders();

        double targetTicks = Math.abs(inches) * TICKS_PER_INCH;
        double sign = Math.signum(inches);
        double startHeading = getHeading();
        ElapsedTime timer = new ElapsedTime();

        while (opMode.opModeIsActive() && timer.seconds() < DRIVE_TIMEOUT_SEC) {
            double currentTicks = getAverageTicks();
            double remainingTicks = targetTicks - currentTicks;
            if (remainingTicks <= 0) break;

            // ---- 台形制御 ----
            double speedFactor;
            String phase;
            if (currentTicks < ACCEL_DISTANCE_TICKS) {
                speedFactor = currentTicks / ACCEL_DISTANCE_TICKS;
                phase = "ACCELERATING";
            } else if (remainingTicks < DECEL_DISTANCE_TICKS) {
                speedFactor = remainingTicks / DECEL_DISTANCE_TICKS;
                phase = "DECELERATING";
            } else {
                speedFactor = 1.0;
                phase = "CRUISING";
            }
            speedFactor = Math.max(MIN_SPEED_FACTOR, Math.min(1.0, speedFactor));
            double power = sign * maxPower * speedFactor;

            // ---- 向き補正(P制御) ----
            double headingError = shortestAngleDifference(startHeading, getHeading());
            double correction = clamp(headingError * HEADING_CORRECTION_GAIN,
                    -MAX_CORRECTION, MAX_CORRECTION);

            setDrivePower(power + correction, power - correction);

            telemetry.addData("--- Drive Straight ---", "");
            telemetry.addData("Phase", phase);
            telemetry.addData("Target Ticks", "%.0f", targetTicks);
            telemetry.addData("Current Ticks", "%.0f", currentTicks);
            telemetry.addData("Power", "%.2f", power);
            telemetry.addData("Heading Error (deg)", "%.2f", headingError);
            telemetry.addData("Correction", "%.3f", correction);
            telemetry.update();
        }

        stopMotors();
    }

    // ===== 旋回: PD制御によるフィードバック =====
    public void turnToHeading(double targetAngle) {
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime settleTimer = new ElapsedTime();

        while (opMode.opModeIsActive() && timer.seconds() < TURN_TIMEOUT_SEC) {
            double currentHeading = getHeading();
            double error = shortestAngleDifference(currentHeading, targetAngle);

            // 反時計回りがプラス(ヨー角と同じ向き)
            double rate = imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;

            // 誤差が小さく、回転もほぼ止まった状態が続いたら完了
            boolean settled = Math.abs(error) < TURN_TOLERANCE_DEG
                    && Math.abs(rate) < TURN_SETTLE_RATE;
            if (!settled) {
                settleTimer.reset();
            }
            if (settleTimer.seconds() > TURN_SETTLE_TIME) {
                break;
            }

            // P: 誤差に比例(誤差が大きいうちは最低パワーを保証)
            double p = error * TURN_KP;
            if (Math.abs(error) >= TURN_TOLERANCE_DEG) {
                p = Math.signum(error) * Math.max(Math.abs(p), TURN_MIN_POWER);
            }

            // D: 回転の勢いに比例して逆向きの力をかける(ブレーキ)
            double power = clamp(p - TURN_KD * rate, -1.0, 1.0);

            // 正のパワー = 反時計回り(左を後ろ、右を前)
            setDrivePower(-power, power);

            telemetry.addData("--- Turn To Heading ---", "");
            telemetry.addData("Target Angle", "%.1f", targetAngle);
            telemetry.addData("Current Heading", "%.2f", currentHeading);
            telemetry.addData("Error", "%.2f", error);
            telemetry.addData("Rate (deg/s)", "%.1f", rate);
            telemetry.addData("Power", "%.2f", power);
            telemetry.update();
        }

        stopMotors();
        opMode.sleep(300); // 完全に止まってから最終値を表示する

        telemetry.addData("Turn Complete", "Final Heading: %.2f", getHeading());
        telemetry.update();
    }

    // ===== 内部で使う道具 =====

    /** 現在の向き(度)。反時計回りがプラス。 */
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /** 今の向きを0度にリセットする。 */
    public void resetHeading() {
        imu.resetYaw();
    }

    /** fromからtoまでの、最短経路での角度差(-180〜180度)。 */
    double shortestAngleDifference(double fromAngle, double toAngle) {
        double diff = toAngle - fromAngle;
        return Math.toDegrees(Math.atan2(
                Math.sin(Math.toRadians(diff)),
                Math.cos(Math.toRadians(diff))
        ));
    }

    /** 左側2つ・右側2つにそれぞれパワーを設定する(-1〜1に収める)。 */
    private void setDrivePower(double left, double right) {
        left = clamp(left, -1.0, 1.0);
        right = clamp(right, -1.0, 1.0);
        frontLeftMotor.setPower(left);
        backLeftMotor.setPower(left);
        frontRightMotor.setPower(right);
        backRightMotor.setPower(right);
    }

    /** 4輪のエンコーダー絶対値の平均(tick)。 */
    private double getAverageTicks() {
        return (Math.abs(frontLeftMotor.getCurrentPosition())
                + Math.abs(backLeftMotor.getCurrentPosition())
                + Math.abs(frontRightMotor.getCurrentPosition())
                + Math.abs(backRightMotor.getCurrentPosition())) / 4.0;
    }

    /** エンコーダーを0に戻し、エンコーダー付きの通常走行モードにする。 */
    private void resetEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void stopMotors() {
        setDrivePower(0, 0);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}