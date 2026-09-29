package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class GamePadPractice extends OpMode {
    @Override
    public void init() {

    }



    @Override
    public void loop (){
        // runs 50 a second
        double y = -gamepad1.left_stick_y; // Y stick is reversed




        telemetry.addData("left_x", gamepad1.left_stick_x);
        telemetry.addData("left_y", y);
        telemetry.addData("x", gamepad1.a);
        telemetry.addData("right_x", gamepad1.right_stick_x);
        telemetry.addData("right_y", gamepad1.right_stick_y);


    }
}