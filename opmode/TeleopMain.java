package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.hardware.HardwareMain;
import static java.lang.Math.abs;

/**
 * TeleopMain is the primary TeleOp OpMode for Relic Recovery. All driver-controlled actions should
 * be defined in this class.
 *
 * Gamepad1 BUTTON MAPPINGS:
 * Left stick x:      Control robot's velocity and direction (stafe)
 * Left stick y:      Control robot's velocity and direction
 * Right stick x:     Turn Robot
 * Right stick y:   N/A
 * X:               Un-extends sweeper arm
 * Y:               Extends sweeper arm
 * A:               N/A
 * B:               N/A
 * Left bumper:     Decelerates robot
 * Right bumper:    Accelerates robot
 * Left trigger:    Special CV program: drive to lander - drives slightly left of the middle of the lander + lifts tape measure
 * Right trigger:   Special CV program: Orient robot facing parallel to each wall - press multiple times to orient
 * DPAD_UP:         N/A
 * DPAD_DOWN:       N/A
 * DPAD_LEFT:       N/A
 * DPAD_RIGHT:      N/A
 * START:           N/A
 * BACK:            N/A
 *
 * Gamepad2 BUTTON MAPPINGS:
 * Left stick x:      Rotates linear slides
 * Left stick y:      Raise & Drops linear slides
 * Right stick x:   N/A
 * Right stick y:   Raise and drops tape measure mechanism
 * X:               Un-extends sweeper arm
 * Y:               Extends sweeper arm
 * A:               N/A
 * B:               N/A
 * Left bumper:     Activates acquirer reverse
 * Right bumper:    Activates acquirer inward
 * Left trigger:    N/A
 * Right trigger:   N/A
 * DPAD_UP:         N/A
 * DPAD_DOWN:       N/A
 * DPAD_LEFT:       N/A
 * DPAD_RIGHT:      N/A
 * START:           N/A
 * BACK:            N/A
 *
 */
@TeleOp(name = "Teleop: Main", group = "Teleop")
public class TeleopMain extends OpMode {

    @Override
    public void init() {
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
    }
}