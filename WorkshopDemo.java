package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "WorkshopDemo (Blocks to Java)")
public class WorkshopDemo extends LinearOpMode {

  private IMU imu;
  private DcMotor frontLeft;
  private DcMotor backLeft;
  private DcMotor frontRight;
  private DcMotor backRight;

  /**
   * This OpMode offers POV (point-of-view) style TeleOp control for a
   * direct drive robot. In this POV mode, the left joystick (up and down)
   * moves the robot forward and back, and the right joystick (left and
   * right) spins the robot left (counterclockwise) and right (clockwise).
   */
  @Override
  public void runOpMode() {
    double referenceHeading;
    int PValue;
    float vy;
    float vx;
    float omega;
    boolean lockHeading;
    double headingError;
    double currentHeading;
    float FrontLeftVel;
    float BackLeftVel;
    float FrontRightVel;
    float BackRightVel;
    double MaximumValue;

    imu = hardwareMap.get(IMU.class, "imu");
    frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
    backLeft = hardwareMap.get(DcMotor.class, "backLeft");
    frontRight = hardwareMap.get(DcMotor.class, "frontRight");
    backRight = hardwareMap.get(DcMotor.class, "backRight");

    // Setup Motors and Sensors, and initialize variables
    // Initialize the IMU with non-default settings. To use this block,
    // plug one of the "new IMU.Parameters" blocks into the parameters socket.
    // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
    // Expansion Hub, specifying the hub's orientation on the robot via the direction that
    // the REV Robotics logo is facing and the direction that the USB ports are facing.
    imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
    referenceHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    PValue = 0;
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Get Controller Input
        vy = gamepad1.left_stick_y;
        vx = gamepad1.right_stick_x;
        omega = gamepad1.left_stick_y;
        // Correct for Perturbations
        if (Math.abs(vx) < 0.07) {
          vx = 0;
        }
        if (Math.abs(vy) < 0.07) {
          vy = 0;
        }
        // Determine if Heading should be Locked
        if (Math.abs(omega) < 0.07) {
          omega = 0;
          if (lockHeading == false) {
            lockHeading = true;
            referenceHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
          }
          currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        } else {
          lockHeading = false;
        }
        if (lockHeading == true) {
          headingError = AngleUnit.DEGREES.normalize(referenceHeading - currentHeading);
          omega = (float) (PValue * headingError);
        }
        // Set Motor Powers
        FrontLeftVel = vx + vy + omega;
        BackLeftVel = (vx - vy) + omega;
        FrontRightVel = (vx - vy) - omega;
        BackRightVel = (vx + vy) - omega;
        // Normalize Wheel Velocities
        MaximumValue = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(BackLeftVel), Math.abs(BackRightVel), Math.abs(FrontLeftVel), Math.abs(FrontRightVel), 1));
        frontLeft.setPower(FrontLeftVel / MaximumValue);
        backLeft.setPower(BackLeftVel / MaximumValue);
        frontRight.setPower(FrontRightVel / MaximumValue);
        backRight.setPower(BackRightVel / MaximumValue);
        telemetry.update();
      }
    }
  }
}
