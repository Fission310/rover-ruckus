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
 * Left bumper:     Controls the arm on the tape measure to latch
 * Right bumper:    Controls the arm on the tape measure to unlatch
 * Left trigger:    Drops tape measure mechanism
 * Right trigger:   Raise tape measure mechanism
 * DPAD_UP:         Raise linear slides
 * DPAD_DOWN:       Drops linear slides
 * DPAD_LEFT:       Rotates linear slides back
 * DPAD_RIGHT:      Rotates linear slides forward
 * START:           N/A
 * BACK:            N/A
 *
 * Gamepad2 BUTTON MAPPINGS:
 * Left stick x:      N/A
 * Left stick y:      N/A
 * Right stick x:     N/A
 * Right stick y:   N/A
 * X:               Un-extends sweeper arm
 * Y:               Extends sweeper arm
 * A:               Controls the arm on the tape measure to latch
 * B:               Controls the arm on the tape measure to unlatch
 * Left bumper:     Activates acquirer reverse
 * Right bumper:    Activates acquirer inward
 * Left trigger:    N/A
 * Right trigger:   N/A
 * DPAD_UP:         Raise linear slides
 * DPAD_DOWN:       Drops linear slides
 * DPAD_LEFT:       Rotates linear slides back
 * DPAD_RIGHT:      Rotates linear slides forward
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