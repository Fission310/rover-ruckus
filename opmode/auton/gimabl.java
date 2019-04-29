package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.mecanum.HardwareMecanum;

@Autonomous(name = "gimbal", group = "test")
public class gimabl extends LinearOpMode {
    public HardwareMecanum robot = new HardwareMecanum(this);
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            robot.gimbal.setHorizontalRight();
            sleep(500);
            robot.gimbal.setHorizontalNeutral();
            sleep(500);
            robot.gimbal.setHorizontalLeft();
            sleep(500);
            robot.gimbal.setVerticalRight();
            sleep(500);
            robot.gimbal.setVerticalNeutral();
            sleep(500);
            robot.gimbal.setVerticalLeft();
            sleep(500);
        }
    }
}
