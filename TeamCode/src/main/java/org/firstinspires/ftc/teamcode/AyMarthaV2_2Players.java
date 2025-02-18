package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.ColorSensor;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
  *
 * @author Gerry DLIII - 18908 Mighty Hawks
 * @version 1.0, 12/20/2024
 */
@TeleOp(name = "AyMartha 2P RR", group = "ITD Teleop")
public class AyMarthaV2_2Players extends OpMode {

    private enum IntakeState{
        IN,
        OUT
    }
    private enum IntakeCurrState{
        IN,
        OUT
    }
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private DcMotorEx OuttakeSliderRight;
    private DcMotorEx OuttakeSliderLeft;
    private Servo IntakeSliderRight;
    private Servo IntakeSliderLeft;
//    private Servo IntakeClaw;

    //private Servo OuttakeWrist;
    private Servo OuttakeElbowRight;
    private Servo OuttakeElbowLeft;
    private Servo OuttakeClaw;
    private Servo IntakeElbowRight;
    private Servo IntakeElbowLeft;
    //private Servo IntakeClaw;
    private CRServo LeftWheel;
    private CRServo RightWheel;
    private ColorSensor IntakeSensor;
    public static int HIGH_BASKET = 4150;
    public static int HIGH_CHAMBER = 600;
    public static int initialPositionLeft, initialPositionRight;

    private boolean IntakeElbowDown = false;
    private boolean OuttakeElbowDown = false;
    private boolean OuttakeClawOpen = false;
    private boolean IntakeClawOpen = true;
    private boolean IntakeSliderChanged = false;
    final double IntakeClawPositionClose = 1.0;
    final double IntakeClawPositionOpen = 0.00;
    public static double IntakeSliderPositionIN = 0.65;
    final double IntakeSliderPositionOut = 0.15;
    final double IntakeElbowPositionIn = 0.73;
    final double IntakeElbowPositionOut = 0.09;
    final double OuttakeElbowPositionOut = 0.14;
    final double OuttakeElbowPositionSpecimenScoring = 0.38;
    final double OuttakeElbowPositionIn = 0.72;
    final double OuttakeElbowPositionMiddle = 0.48;
    final double OuttakeWristPositionOut = 0.80;
    final double OuttakeWristPositionIn = 0.00;
    final double OuttakeClawPositionClose = 1.0;
    final double OuttakeClawPositionOpen = 0.00;
    boolean TurnOuttakeSlidersOff = false;
    private int intakeCurrentState = 0;
    public int PlayerSelection = 2;
    public int leftPosition, rightPosition;
    public ElapsedTime slidersElapsedTime;


    /**
     * This initializes the drive motors as well as the Follower and motion Vectors.
     */
    @Override
    public void init() {
        //Driving Motors Mapping and Setup
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        //Intake
        //Servo Sliders Mapping and Setup
        IntakeSliderRight = hardwareMap.get(Servo.class, "IntakeSliderRight");
        IntakeSliderLeft = hardwareMap.get(Servo.class, "IntakeSliderLeft");

        IntakeSliderRight.setDirection(Servo.Direction.FORWARD);
        IntakeSliderLeft.setDirection(Servo.Direction.REVERSE);

        IntakeSliderLeft.scaleRange(0.0, 1.0);
        IntakeSliderRight.scaleRange(0.0, 1.0);



        //Servo Claw and Elbow Mapping and Setup
//        IntakeClaw = hardwareMap.get(Servo.class, "IntakeClaw");
        RightWheel = hardwareMap.get(CRServo.class, "RightWheel");
        LeftWheel = hardwareMap.get(CRServo.class, "LeftWheel");
        IntakeElbowRight = hardwareMap.get(Servo.class, "IntakeElbowRight");
        IntakeElbowLeft = hardwareMap.get(Servo.class, "IntakeElbowLeft");
        IntakeSensor = hardwareMap.get(ColorSensor.class, "IntakeSensor");

         RightWheel.setDirection(CRServo.Direction.FORWARD);
         LeftWheel.setDirection(CRServo.Direction.REVERSE);
        IntakeElbowRight.setDirection(Servo.Direction.FORWARD);
        IntakeElbowLeft.setDirection(Servo.Direction.REVERSE);

        //IntakeClaw.scaleRange(0.0, 1.0);
        //IntakeElbowRight.scaleRange(0.0, 1.0);
        //IntakeElbowLeft.scaleRange(0.0, 1.0);

        //Outtake
        // Sliders Mapping and Setup
        OuttakeSliderRight = hardwareMap.get(DcMotorEx.class, "OuttakeSliderRight");
        OuttakeSliderLeft = hardwareMap.get(DcMotorEx.class, "OuttakeSliderLeft");

        OuttakeSliderRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        OuttakeSliderLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        OuttakeSliderRight.setDirection(DcMotorSimple.Direction.FORWARD);
        OuttakeSliderLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        initialPositionLeft = OuttakeSliderLeft.getCurrentPosition();
        initialPositionRight = OuttakeSliderRight.getCurrentPosition();

        //Servo Claw, Elbow, and Wrist Mapping and Setup
        OuttakeClaw = hardwareMap.get(Servo.class, "OuttakeClaw");
        OuttakeElbowRight = hardwareMap.get(Servo.class, "OuttakeElbowRight");
        OuttakeElbowLeft = hardwareMap.get(Servo.class, "OuttakeElbowLeft");
        //OuttakeWrist = hardwareMap.get(Servo.class, "OuttakeWrist");

        OuttakeClaw.setDirection(Servo.Direction.FORWARD);
        OuttakeElbowRight.setDirection(Servo.Direction.FORWARD);
        OuttakeElbowLeft.setDirection(Servo.Direction.REVERSE);
        TurnOuttakeSlidersOff = false;

        slidersElapsedTime = new ElapsedTime();
        slidersElapsedTime.reset();
        telemetry.addData("Initial Pos Right", initialPositionRight);
        telemetry.addData("Initial Pos Left", initialPositionLeft);
    }

    /**
     * This runs the OpMode. This is only drive control with Pedro Pathing live centripetal force
     * correction.
     */
    @Override
    public void loop() {
        //follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        //follower.update();

        // Mecanum Drivetrain Manually Programmed
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
        switch (PlayerSelection){
            case 1:
                //All Intake Code
                //Intake Slider
                if(gamepad1.cross && !IntakeSliderChanged){
                    if(intakeCurrentState==0){
                        intakeSlidersElbow(IntakeState.OUT);
                        intakeCurrentState=1;
                    } else if (intakeCurrentState==1) {
                        intakeSlidersElbow(IntakeState.IN);
                        intakeCurrentState=0;
                    }
                    IntakeSliderChanged = true;
                } else if (!gamepad1.cross) {
                    IntakeSliderChanged = false;
                }
                if(gamepad1.left_trigger>0.25){
                    RightWheel.setPower(-gamepad1.left_trigger);
                    LeftWheel.setPower(-gamepad1.left_trigger);
                } else if (gamepad1.right_trigger>0.25) {
                    RightWheel.setPower(gamepad1.right_trigger);
                    LeftWheel.setPower(gamepad1.right_trigger);
                }

                if(gamepad1.ps){
                    OuttakeSlidersResetInitialPosition();
                }

                //All Outtake Code
                //Outtake Sliders Programming
                if(gamepad1.triangle){
                    outtakeSliders(HIGH_BASKET, 2000, 0);
                    //OuttakeElbowMove(OuttakeElbowPositionOut);
                }else if(gamepad1.square){
                    outtakeSliders(HIGH_CHAMBER, 2000, 0);
                    OuttakeElbowMove(OuttakeElbowPositionSpecimenScoring);
                }else if(gamepad1.circle){
                    OuttakeElbowMove(OuttakeElbowPositionMiddle);
                    outtakeSliders(0, 2000, 0);

                }else if(gamepad1.dpad_down){

                    outtakeSliders(0, 2000, 0);

                }

                //Outtake elbow automatically goes out after reaching high basket position
                if((OuttakeSliderLeft.getCurrentPosition()>(HIGH_BASKET-20) && OuttakeSliderRight.getCurrentPosition()>(HIGH_BASKET-20))
                        && (((OuttakeElbowLeft.getPosition()==OuttakeElbowPositionMiddle || OuttakeElbowRight.getPosition()==OuttakeElbowPositionMiddle))
                        || (OuttakeElbowLeft.getPosition()==OuttakeElbowPositionIn || OuttakeElbowRight.getPosition()==OuttakeElbowPositionIn))){

                    OuttakeElbowMove(OuttakeElbowPositionOut);
                }
                if(TurnOuttakeSlidersOff && OuttakeSliderRight.getCurrentPosition()<=initialPositionRight && OuttakeSliderLeft.getCurrentPosition()<=initialPositionLeft && slidersElapsedTime.seconds() > 5){
                    OuttakeSliderLeft.setPower(0.0);
                    OuttakeSliderRight.setPower(0.0);
                }

                //Outtake Elbow Middle
                if(gamepad1.dpad_up){
                    OuttakeElbowMove(OuttakeElbowPositionMiddle);
                } else if (gamepad1.dpad_left) {
                    OuttakeElbowMove(OuttakeElbowPositionIn);
                    OuttakeClaw.setPosition(OuttakeClawPositionOpen);
                } else if (gamepad1.dpad_right) {
                    OuttakeClaw.setPosition(OuttakeClawPositionClose);
                    OuttakeElbowMove(OuttakeElbowPositionOut);
                }

                //Outtake Claw Toggle
                if(gamepad1.left_bumper && !OuttakeClawOpen){
                    if(OuttakeClaw.getPosition() == OuttakeClawPositionOpen){
                        OuttakeClaw.setPosition(OuttakeClawPositionClose);
                    }else{
                        OuttakeClaw.setPosition(OuttakeClawPositionOpen);
                    }
                    OuttakeClawOpen = true;
                }else if (!gamepad1.left_bumper) {
                    OuttakeClawOpen = false;
                }
                    break;
            case 2:
                //All Intake Code (Player 1)
                //Intake Slider
                if(gamepad1.cross && !IntakeSliderChanged){
                    if(intakeCurrentState==0){
                        intakeSlidersElbow(IntakeState.OUT);
                        intakeCurrentState=1;
                    } else if (intakeCurrentState==1) {
                        intakeSlidersElbow(IntakeState.IN);
                        intakeCurrentState=0;
                    }
                    IntakeSliderChanged = true;
                } else if (!gamepad1.cross) {
                    IntakeSliderChanged = false;
                }
                 /*if(gamepad1.left_trigger>0.25){
                     IntakeClaw.setPosition(IntakeClawPositionClose);
                 } else if (gamepad1.right_trigger>0.25) {
                     IntakeClaw.setPosition(IntakeClawPositionOpen);
                 }*/

                //All Outtake Code (Player 2)
                //Outtake Sliders Programming
                if(gamepad2.triangle){
                    outtakeSliders(HIGH_BASKET, 2000, 0);
                    //OuttakeElbowMove(OuttakeElbowPositionOut);
                }else if(gamepad2.square){
                    outtakeSliders(HIGH_CHAMBER, 2000, 0);
                }else if(gamepad2.cross){
                    OuttakeElbowMove(OuttakeElbowPositionMiddle);
                    outtakeSliders(5, 2000, 0);

                }else if(gamepad2.dpad_down){

                    outtakeSliders(-100, 2000, 1);

                }else if(gamepad2.dpad_up){

                    outtakeSliders(100, 2000, 1);

                }


                //Outtake Elbow Middle
                if(gamepad2.left_bumper){
                    OuttakeElbowMove(OuttakeElbowPositionMiddle);
                } else if (gamepad2.dpad_left) {
                    OuttakeElbowMove(OuttakeElbowPositionOut);
                    OuttakeClaw.setPosition(OuttakeClawPositionClose);
                } else if (gamepad2.dpad_right) {
                    //OuttakeElbowRight.setPosition(OuttakeElbowPositionOut);
                    //OuttakeElbowLeft.setPosition(OuttakeElbowPositionOut);
                    OuttakeClaw.setPosition(OuttakeClawPositionOpen);
                    OuttakeElbowMove(OuttakeElbowPositionIn);
                }

                //Outtake Claw Toggle
                /*if(gamepad1.left_bumper && !OuttakeClawOpen){
                    if(OuttakeClaw.getPosition() == OuttakeClawPositionOpen){
                        OuttakeClaw.setPosition(OuttakeClawPositionClose);
                    }else{
                        OuttakeClaw.setPosition(OuttakeClawPositionOpen);
                    }
                    OuttakeClawOpen = true;
                }else if (!gamepad1.left_bumper) {
                    OuttakeClawOpen = false;
                }*/
                if(gamepad2.left_trigger > .25){
                    OuttakeClaw.setPosition(OuttakeClawPositionClose);
                }else if(gamepad2.right_trigger>0.25){
                    OuttakeClaw.setPosition(OuttakeClawPositionOpen);
                }
                if(gamepad2.ps){
                    OuttakeSlidersResetInitialPosition();
                }
                    break;
            default:

                    break;

        }

        //General code for both options (1 or 2 Players)
        if((TurnOuttakeSlidersOff && slidersElapsedTime.seconds() > 5.0)){
            OuttakeSliderLeft.setPower(0.0);
            OuttakeSliderRight.setPower(0.0);
            TurnOuttakeSlidersOff = false;
        }


        //Outtake elbow automatically goes out after reaching high basket position
        if((OuttakeSliderLeft.getCurrentPosition()>(HIGH_BASKET-20) && OuttakeSliderRight.getCurrentPosition()>(HIGH_BASKET-20))
                && (((OuttakeElbowLeft.getPosition()==OuttakeElbowPositionMiddle || OuttakeElbowRight.getPosition()==OuttakeElbowPositionMiddle))
                || (OuttakeElbowLeft.getPosition()==OuttakeElbowPositionIn || OuttakeElbowRight.getPosition()==OuttakeElbowPositionIn))){

            OuttakeElbowMove(OuttakeElbowPositionOut);
        }

        if(gamepad1.share){
            PlayerSelection = 2;
        }else if(gamepad1.options){
            PlayerSelection = 1;
        }


        //All Telemetry goes here
        // Lets get the target positions.
        telemetry.addData("Intake Slider Right Pos: ", IntakeSliderRight.getPosition() );
        telemetry.addData("Intake Slider Left Pos: ", IntakeSliderLeft.getPosition() );
        telemetry.addData("OuttakeRight Pos: ", OuttakeSliderRight.getCurrentPosition() );
        telemetry.addData("OuttakeLeft Pos: ", OuttakeSliderLeft.getCurrentPosition() );
        telemetry.addData("Outtake Elbow L Pos: ", OuttakeElbowLeft.getPosition() );
        telemetry.addData("Outtake Elbow R Pos: ", OuttakeElbowRight.getPosition() );
        telemetry.addData("Outtake Elbow Down: ", OuttakeElbowDown );
        telemetry.addData("Intake Elbow Down: ", IntakeElbowDown );
//        telemetry.addData("Intake Claw Open: ", IntakeClawOpen );
        telemetry.addData("Right Slider: ", OuttakeSliderRight.getPower() );
        telemetry.addData("Left Slider: ", OuttakeSliderLeft.getPower() );
        telemetry.addData("Left Position: ", leftPosition );
        telemetry.addData("Right Position: ", rightPosition );
        telemetry.addData("intake Sensor red: ", IntakeSensor.red() );
        telemetry.addData("intake Sensor green: ", IntakeSensor.green() );
        telemetry.addData("intake Sensor blue: ", IntakeSensor.blue() );




    }
    public void outtakeSliders(int targetPosition, int velocity, int manual_override){
        // Set run modes for both motors
        OuttakeSliderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        OuttakeSliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Synchronize motion
        if(manual_override == 0) {
            OuttakeSliderRight.setTargetPosition(targetPosition + initialPositionRight);
            OuttakeSliderLeft.setTargetPosition(targetPosition + initialPositionLeft);
        }else{
            OuttakeSliderRight.setTargetPosition(targetPosition + OuttakeSliderRight.getCurrentPosition());
            OuttakeSliderLeft.setTargetPosition(targetPosition + OuttakeSliderLeft.getCurrentPosition());
        }
        if(targetPosition < 10){
            TurnOuttakeSlidersOff = true;
            slidersElapsedTime.reset();
        }
        //Run to target position
        OuttakeSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        OuttakeSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set power
        OuttakeSliderRight.setPower(1.0);
        OuttakeSliderLeft.setPower(1.0);
        // while(OuttakeSliderRight.isBusy() && OuttakeSliderLeft.isBusy()){

        //}

        /*
        // Set the same PID coefficients for both motors
        OuttakeSliderRight.setVelocityPIDFCoefficients(2.0, 0.5, 0.1, 0.0);
        OuttakeSliderLeft.setVelocityPIDFCoefficients(2.0, 0.5, 0.1, 0.0);

        // Synchronize motion
        OuttakeSliderRight.setTargetPosition(targetPosition + initialPositionRight);
        OuttakeSliderLeft.setTargetPosition(targetPosition + initialPositionLeft);

        OuttakeSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        OuttakeSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        OuttakeSliderRight.setVelocity(velocity);  // Set velocity in ticks per second
        OuttakeSliderLeft.setVelocity(velocity);
        */
    }
    public void OuttakeSlidersResetInitialPosition(){
        initialPositionLeft = OuttakeSliderLeft.getCurrentPosition();
        initialPositionRight = OuttakeSliderRight.getCurrentPosition();
        OuttakeSliderRight.setPower(0.0);
        OuttakeSliderLeft.setPower(0.0);
    }
    public void intakeSlidersElbow(IntakeState os){
        // Set run modes for both motors

        switch (os){
            case IN:
                IntakeSliderRight.setPosition(IntakeSliderPositionIN);
                IntakeSliderLeft.setPosition(IntakeSliderPositionIN);
                IntakeElbowRight.setPosition(IntakeElbowPositionIn);
                IntakeElbowLeft.setPosition(IntakeElbowPositionIn);
                RightWheel.setPower(0.0);
                LeftWheel.setPower(0.0);
                break;
            case OUT:
                IntakeSliderRight.setPosition(IntakeSliderPositionOut);
                IntakeSliderLeft.setPosition(IntakeSliderPositionOut);
                IntakeElbowRight.setPosition(IntakeElbowPositionOut);
                IntakeElbowLeft.setPosition(IntakeElbowPositionOut);
                RightWheel.setPower(1.0);
                LeftWheel.setPower(1.0);
                break;

        }

    }
    private void OuttakeElbowMove(double OuttakeElbowTargetPosition){
        OuttakeElbowRight.setPosition(OuttakeElbowTargetPosition);
        OuttakeElbowLeft.setPosition(OuttakeElbowTargetPosition);
    }

}