package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Squeeze Test", group = "Tune")
public class SqueezeTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        CRServo left = hardwareMap.crservo.get("left_squeeze_motor");
        CRServo right = hardwareMap.crservo.get("right_squeeze_motor");
        left.setDirection(DcMotorSimple.Direction.REVERSE);  // 逆に回るなら left に付け替える

        waitForStart();

        left.setPower(1.0);
        right.setPower(1.0);

        while (opModeIsActive()) {
            idle();   // STOPを押すまで100%で回り続ける
        }

        left.setPower(0);
        right.setPower(0);
    }
}