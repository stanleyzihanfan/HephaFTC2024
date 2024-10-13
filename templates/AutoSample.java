/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Autonomous

public class AutoSample extends OpMode
{
    //arm servo+motor declaration
    public DcMotor  armMotor    = null; //the arm motor
    public CRServo  intake      = null; //the active intake servo
    public Servo    wrist       = null; //the wrist servo
    // drivetrain wheel motor declaration
    private DcMotor wheel_0=null;
    private DcMotor wheel_1=null;
    private DcMotor wheel_2=null;
    private DcMotor wheel_3=null;
    //initiate runtime
    private ElapsedTime runtime=new ElapsedTime();
    //encoder ticks per degree rotation of the arm
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation
    //arm position variables(to be use for armToPosition in your code)
    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 250 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 15  * ARM_TICKS_PER_DEGREE;
    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;
    final double WRIST_FOLDED_IN   = 0.8333;
    final double WRIST_FOLDED_OUT  = 0.5;
    //adjust this variable to change how much the arm adjudsts by for the left and right triggers.
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;
    //variables used to set the arm to a specific position.
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double wristPosition = WRIST_FOLDED_IN;
    double intakeSpeed = INTAKE_OFF;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //Log robot is initializing
        telemetry.addData("Status:", "Initializing");
        telemetry.update();
        //initiate drivetrain motors
        wheel_0   = hardwareMap.get(DcMotor.class, "wheel_0");
        wheel_1    = hardwareMap.get(DcMotor.class, "wheel_1");
        wheel_2   = hardwareMap.get(DcMotor.class, "wheel_2");
        wheel_3   = hardwareMap.get(DcMotor.class, "wheel_3");
        
        wheel_2.setDirection(DcMotor.Direction.REVERSE);
        wheel_3.setDirection(DcMotor.Direction.REVERSE);
        //initiate arm motor
        armMotor   = hardwareMap.get(DcMotor.class, "arm_lift"); //the arm motor
        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5,CurrentUnit.AMPS);
        //define servos
        intake = hardwareMap.get(CRServo.class, "servo_intake");
        wrist  = hardwareMap.get(Servo.class, "servo_rotate");
        //Log that initialization is complete.
        telemetry.addData("Status: ", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        /*assert is a variable developers use for testing errers, 
        if it is false the program will return an assertion error.
        You can also do assert statement message, where statement
        is the boolean and message is an optional message to be displayed
        when the error occurs.*/
        /*here this line is just so people know they can add something here,
        it dosn't do anything.*/
        assert true;
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        //Log the robot is initializing servo+motor positions
        telemetry.addData("Status: ","Intializing Positions");
        telemetry.update();
        //reset runtime(for future needs, if using time controls)
        runtime.reset();
        //reset arm encoder
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //initialize servos
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_IN);
        //Log position intiation is complete
        telemetry.addData("Status: ","Robot Ready");
        telemetry.update();
        
        //ADD MAIN CODE HERE
        //be sure to use telemetry and log all variables for debugging!
        
        
        //sample of rotating robot by time, then raising and lowering the arm.
        /*trigger drive function(the values are: drive,strafe,turn, and whether
        or not to have the function directly write the values the motors)
        Also the function will always return a double list with all of the motor speeds.*/
        mecanumDrive(0, 0, 0.5, true);
        //wait 3 seconds
        waitForTime(3);
        //reset all motors to 0 speed
        mecanumDrive(0, 0, 0, true);
        //wait 1 second
        waitForTime(1);
        //set arm position to vertical
        armPosition=(int)ARM_ATTACH_HANGING_HOOK;
        //wait untill the motor runs to the desired position
        while (armMotor.getCurrentPosition()!=armPosition){
            armToPosition(armPosition, wristPosition, intakeSpeed);
        }
        //wait for 3 seconds
        waitForTime(3);
        //set arm position to starting position
        armPosition=(int)ARM_COLLAPSED_INTO_ROBOT;
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        //Constantly set the arm positions, as the arm motor requires this to operate properly
        armToPosition(armPosition, wristPosition, intakeSpeed);
    }
    
    //Wait Function
    //This is only here for the sake of not writing a for loop every time you need to wait.
    public void waitForTime(double seconds){
        //reset runtime
        runtime.reset();
        //wait untill runtime exeeds time limit
        while(runtime.seconds()<=seconds){
            //update telemtry
            telemetry.addData("Time","%4.1f S Elapsed",runtime.seconds());
            telemetry.update();
            //Constantly call the armToPosition function.
            armToPosition(armPosition, wristPosition, intakeSpeed);
        }
    }
    
    //arm function
    public void armToPosition(double armPos, double wristPos, double intakeSpeed){
        //set arm motor target position
        armMotor.setTargetPosition((int)(armPos));
        //set motor velocity
        ((DcMotorEx) armMotor).setVelocity(2100);
        //run arm motor to position
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //telemetry if motor exceeded current limit
        if (((DcMotorEx) armMotor).isOverCurrent()){
            telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
        }
        telemetry.update();
        //If the wrist pos passed in is greater than 8.333, then reset it to 8.333.
        //This is to prevent the servo from jaming itself against the arm and burning
        //out due to a bad value being passed in.
        if (wristPos<=8.333){
            //If value is within range
            wrist.setPosition(wristPos);
        }
        else if (wristPos<3.333){
            //If value is out of range in the negative direction
            wrist.setPosition(0);
        }
        else{
            //If value is out of range in the positive direction
            wrist.setPosition(8.333);
        }
        //set intake power/speed
        intake.setPower(intakeSpeed);
    }
    
    //Drive Function
    public double[] mecanumDrive(double drive, double strafe, double twist, boolean apply){
        strafe*=-1;
        twist*=-1;
        telemetry.addData("Drive: ",drive);
        telemetry.addData("Strafe: ",strafe);
        telemetry.addData("Twist: ",twist);
        double[] speeds={
            (drive+strafe+twist),//forward-left motor(wheel_0)
            (drive-strafe-twist),//forward-right motor(wheel_1)
            (drive-strafe+twist),//back-left motor(wheel_2)
            (drive+strafe-twist)//back-right motor(wheel_3)
        };
        // Loop through all values in the speeds[] array and find the greatest magnitude.
        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) {
                max = Math.abs(speeds[i]);
            }
        }
        //if max is greater than 1, normalize all values to between 0 and 1.
        if(max>1){
            for (int i=0;i<speeds.length;i++){
                //divide the values by the biggest one
                speeds[i]/=max;
            }
        }
        //if apply is true, then directly apply speeds to motors.
        if (apply){
            wheel_0.setPower(speeds[0]);
            wheel_3.setPower(speeds[1]);
            wheel_1.setPower(speeds[2]);
            wheel_2.setPower(speeds[3]);
            return speeds;
        }
        else{
            return speeds;
        }
    }
    
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        assert true;
    }

}
