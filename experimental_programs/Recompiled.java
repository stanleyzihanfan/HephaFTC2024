package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math;
import java.lang.Thread;
import java.util.*;

@TeleOp(name="Template: Linear OpMode", group="Linear Opmode")
@Disabled

@Autonomous

public class Recompiled {

    // todo: write your code here
}
//classes

/**
 * Motor class with all motor information
 */
class Motor {
    protected DcMotorEx motor_ = null;
    protected int velocity = 0;
    protected int targetPosition = 0;
    protected double maxspeed=0;
    //whether or not to use acceleration deceleration controls
    protected boolean useAccelControls = true;
    //The state of the motor(0=neutral/run at current parameters, 1=accelerating, 2=decelerating)
    protected int motorState = 0;

    /**
     * Set motor
     *
     * @param motor motor as DcMotorEx object
     */
    protected void setMotor(DcMotorEx motor) {
        this.motor_ = motor;
    }


    /**
     * Set motor target position
     *
     * @param target double target position in encoder ticks
     */
    protected void setTargetPos(int target) {
        this.targetPosition = target;
        this.motor_.setTargetPosition(this.targetPosition);
    }

    /**
     * Set motor velocity
     *
     * @param velocity desired integer velocity for the motor in encoder ticks per second
     */
    protected void setMotorVelocity(int velocity) {
        this.velocity = velocity;
        this.motor_.setVelocity(this.velocity);
    }

    /**
     * Function to calculate expected distance at given tick(loop) number
     * @param referencey tick number of referencex
     * @param referencex the acceleration at referencey
     * @param currentspeed the "current" speed
     * @param currentdistance the "current" distance
     * @param currenty the "current" tick number(only has to match with the tick that currentspeed and currentdistance was read from)
     * @param targety the "target" tick number(the tick that the distance should be calculated for)
     * @param accelerationchange the slope of the acceleration(how fast to change acceleration)
     * @return the expected distance at targety
     */
    public double acelToDistance(double referencey, double referencex, double currentspeed, double currentdistance, double currenty, double targety, double accelerationchange) {
        double b = referencey - accelerationchange * referencex;
        return 3 * Math.pow(currenty, 2) * targety + 3 * currenty * Math.pow(targety, 2) + Math.pow(targety, 3) + (b + accelerationchange * currenty) * Math.pow(currenty, 2) + 2 * currenty * targety + currentspeed * currenty + currentdistance;
    }
}

/**
 * IMU class
 */
class Imu {
    protected IMU imu_ = null;

    /**
     * Set IMU
     *
     * @param imu IMU object
     */
    protected void setIMU(IMU imu) {
        this.imu_ = imu;
    }

    /**
     * Reset IMU yaw angle
     */
    protected void resetYaw() {
        this.imu_.resetYaw();
    }

    /**
     * Get robot yaw angle
     *
     * @return Yaw angle as a double in degrees
     */
    protected double getHeading() {
        YawPitchRollAngles orientation = this.imu_.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    /**
     * Get steering correction
     * @param desiredHeading target heading
     * @param proportionalGain rate of change
     * @return the amount to be corrected
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        // Determine the heading current error
        double headingError = desiredHeading - getHeading();
        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;
        // Multiply the error by the gain to determine the required steering correction. Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }
}

/**
 * Drivetrain class
 */
class Drivetrain {
    //motor information list
    protected Motor[] drivetrain = new Motor[4];

    /**
     * Get motor encoder values
     *
     * @return Array of encoder values for each motor, [LF, LR, RR, RF]
     */
    protected int[] readMotorEncoders() {
        int[] encoder = new int[4];
        for (int i = 0; i < 4; i++) {
            encoder[i] = this.drivetrain[i].motor_.getCurrentPosition();
        }
        return encoder;
    }

    /**
     * Set drivetrain motors, 0 is left front, 1 is left rear, 2 is right rear, 3 is right front
     *
     * @param motors Array of DcMotor objects
     */
    protected void setDrivetrain(DcMotorEx[] motors) {
        for (int i = 0; i < 4; i++) {
            Motor motor = new Motor();
            motor.setMotor(motors[i]);
            this.drivetrain[i] = motor;
        }
    }

    /**
     * Set target position of motors.
     * @param encoders int list of length 4, in order of motors array.
     */
    protected void setMotorTargets(int[] encoders){
        for (int i=0;i<4;i++){
            this.drivetrain[i].targetPosition=encoders[i];
        }
    }
}

/**
 * Main robot drivetrain class
 */
class Robot_Drivetrain {
    //robot location information, relative to left-bottom corner of field(0,0).
    //0 degrees theta is facing positive y direction.
    protected double xPos = 0;
    protected double yPos = 0;
    protected double theta = 0;
    //seconds to update robot location via IMU
    protected double secondsToUpdate = 0.5;
    //Seconds to wait for verifying robot location data via external sensors, TO BE IMPLEMENTED
    protected double secondsToVerification = 5;
    protected ElapsedTime timer = new ElapsedTime();
    //imu and drivetrain
    protected Imu imu = new Imu();
    protected Drivetrain drivetrain = new Drivetrain();
    protected Drivetrain drivetrain_save = drivetrain;

    /**
     * Initialize drivetrain and IMU objects, change IMU orientation here
     *
     * @param motors Array of drivetrain DcMotorEx objects(0 is left front, 1 is left rear, 2 is right rear, 3 is right front)
     * @param imu    IMU object
     */
    protected void initiateRobotParts(DcMotorEx[] motors, IMU imu) {
        this.drivetrain.setDrivetrain(motors);
        this.drivetrain_save = this.drivetrain;
        this.imu.setIMU(imu);
        //set the orientation of the REV hub on the robot(the IMU location is on one of the corners of the robot)
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
    }

    /**
     * Initialize robot position
     *
     * @param xPos  Robot x position relative to left-bottom of field
     * @param yPos  Robot y position relative to left-bottom of field
     * @param theta Robot orientation relative to positive y direction
     */
    protected void initiateRobotPosition(double xPos, double yPos, double theta) {
        this.xPos = xPos;
        this.yPos = yPos;
        this.theta = theta;
    }
    /**
     * get gyro heading in degrees
     */
    protected double getHeading(){
        YawPitchRollAngles orientation=imu.imu_.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    /**
     * main gyro drive function.
     * All parameters are cm.
     * Give ~0.5 cm tolerance(especially for drive).
     * @param strafe is how far sideways
     * @param drive is how far forwards
     * @param max_speed is max velocity of motor
     * @param acceleration is how fast motor accelerates
     */
    protected void drivegyro(double strafe, double drive, double max_speed, double acceleration){
        //expected time for motor to finish drive
        double[] drivetimes = new double[4];
        //maximum drive time
        double maxtime=0;
        //distance motor travels
        double[] distance;
        //expectec time it should take for the motor to accelerate to max speed
        double timetoMax=max_speed/acceleration;
        //get wheel targets
        distance=calculateWheelMovement(strafe,drive);
        //calculate time it takes for each wheel to travel to target
        for (int i=0;i<4;i++) {
            distance[i] = distance[i] - drivetrain.readMotorEncoders()[i];
            if ((0.5*acceleration*Math.pow(timetoMax,2))>distance[i]/2)
                drivetimes[i] = timetoMax * 2 + (distance[i] / 2 - acceleration * Math.pow(timetoMax, 2)) / max_speed;
            else drivetimes[i] = 2 * Math.sqrt((2 * distance[i] / 2) / acceleration);
            maxtime=Math.max(maxtime,drivetimes[i]);
        }
        //normalize speeds for time & set all motors to state 1
        for (int i=0;i<4;i++){
            if (drivetimes[i]!=maxtime)
                this.drivetrain.drivetrain[i].maxspeed=findMaxSpeed(acceleration,max_speed,distance[i],maxtime);
            else this.drivetrain.drivetrain[i].maxspeed=max_speed;
            this.drivetrain.drivetrain[i].motorState=1;
        }
        //apply calculated values to motors
        double epsilon=0.005;
        this.timer.reset();
        while (this.timer.time()<=maxtime){
            //iterate motors
            for (int i=0;i<4;i++){
                //swap from accel to constant speed drive
                if (this.drivetrain.drivetrain[i].velocity>=this.drivetrain.drivetrain[i].maxspeed && this.drivetrain.drivetrain[i].motorState==1)
                    this.drivetrain.drivetrain[i].motorState = 0;
                //swap from constant speed to decel
                else if (this.timer.time()>=(maxtime-this.drivetrain.drivetrain[i].maxspeed/acceleration) && this.drivetrain.drivetrain[i].motorState==0)
                    this.drivetrain.drivetrain[i].motorState = 2;
                //change acceleration when accel/decel
                if ((this.timer.time() % 1) <= epsilon){
                    if (this.drivetrain.drivetrain[i].motorState==2)
                        this.drivetrain.drivetrain[i].velocity -= acceleration;
                    else if (this.drivetrain.drivetrain[i].motorState==1)
                        this.drivetrain.drivetrain[i].velocity += acceleration;
                }
                //apply velocity to motor
                this.drivetrain.drivetrain[i].motor_.setVelocity(this.drivetrain.drivetrain[i].velocity);
            }
        }
        //reset motors to 0 speed(a.k.a stop)
        for (int i=0;i<4;i++) {
            this.drivetrain.drivetrain[i].velocity=0;
            this.drivetrain.drivetrain[i].motor_.setVelocity(this.drivetrain.drivetrain[i].velocity);
            this.drivetrain.drivetrain[i].motorState=0;
        }
    }

    /**
     * Calculates expected distance traveled given acceleration, maximum speed, and time.
     * @param acceleration how fast it accelerates
     * @param maxspeed maximum speed it can reach
     * @param time total time traveled
     * @return expected distance traveled
     */
    public double calculateDistance(double acceleration, double maxspeed, double time){
        double timetoMax=maxspeed/acceleration;
        double distance;
        if (timetoMax>time/2) distance = acceleration * Math.pow(time / 2, 2);
        else distance = acceleration * Math.pow(timetoMax, 2) + maxspeed * (time - timetoMax * 2);
        return distance;
    }

    /**
     * Finds the max speed that the motor can travel at using binary search
     * @param acceleration change in speed
     * @param maxspeed maximum speed of motor
     * @param distance how far the motor needs to move
     * @param maxtime time to normalize to
     * @return maximum speed of motor after normalization
     */
    public double findMaxSpeed(double acceleration, double maxspeed, double distance, double maxtime){
        double epsilon=1e-6;
        double l=0,r=maxspeed;
        double mid = 0;
        while ((r-l)>epsilon){
            mid=(l+r)/2;
            if (calculateDistance(acceleration,maxspeed,maxtime)<distance) l = mid;
            else r = mid;
        }
        return mid;
    }

    /**
     * robot movement to wheel movement
     * @param xDistance is x coordinate change
     * @param yDistance is y coordinate change
     * @return double list of leftFront wheel movement, rightFront wheel movement, leftRear wheel movement, rightRear wheel movement, in encoder ticks
     */
    public static double[] calculateWheelMovement(double xDistance, double yDistance) {
        //Calculate target angle
        double angle = Math.atan2(yDistance, xDistance);
        //Calculate total displacement
        double displacement = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
        // Calculate the wheel movement for each direction
        double leftFront = displacement*Math.cos(angle+Math.PI/4);
        double rightFront = displacement*Math.cos(angle-Math.PI/4);
        double leftRear = displacement*Math.cos(angle-Math.PI/4);
        double rightRear = displacement*Math.cos(angle+Math.PI/4);
        return new double[]{leftFront, rightFront, leftRear, rightRear};
    }

    /**
     * Rotates the robot about itself a specified degrees.
     * @param twist Degrees to rotate(negative->clockwise,positive->counterclockwise)
     * @param speed Speed of rotation(in encoder ticks/s)
     */
    public void rotate(double twist,double speed){
        Motor LFront=this.drivetrain.drivetrain[0];
        Motor LRear=this.drivetrain.drivetrain[1];
        Motor RRear=this.drivetrain.drivetrain[2];
        Motor RFront=this.drivetrain.drivetrain[3];
        double targetdirection=getHeading()+twist;
        if (Math.abs(targetdirection)>180){
            if (targetdirection<0){
                targetdirection=360+targetdirection;
            }
            else{
                targetdirection=360-targetdirection;
            }
        }
        boolean target=(getHeading()>targetdirection);
        while ((getHeading()>targetdirection)==target){
            if (target){
                LFront.motor_.setVelocity(speed);
                LRear.motor_.setVelocity(speed);
                RFront.motor_.setVelocity(-speed);
                RRear.motor_.setVelocity(-speed);
            }
            else{
                LFront.motor_.setVelocity(-speed);
                LRear.motor_.setVelocity(-speed);
                RFront.motor_.setVelocity(speed);
                RRear.motor_.setVelocity(speed);
            }
            LFront.motor_.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LRear.motor_.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RFront.motor_.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RRear.motor_.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        LFront.motor_.setTargetPosition(LFront.motor_.getCurrentPosition());
        LRear.motor_.setTargetPosition(LRear.motor_.getCurrentPosition());
        RFront.motor_.setTargetPosition(RFront.motor_.getCurrentPosition());
        RRear.motor_.setTargetPosition(RRear.motor_.getCurrentPosition());
        //stop motors
        LFront.motor_.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LRear.motor_.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFront.motor_.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RRear.motor_.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Function to calculate distance units from rotation distance.
     * @param xdistance Distance moved forwards in cm.
     * @param ydistance Distance moved right in cm.
     * @return Double list of movement units/encoder ticks traveled.
     */
    public static double[] movementToDistanceUnits(double xdistance, double ydistance){
        //forward 17.26 in., 43.8404 cm. --> 0.0438404cm/MU
        //right 13.75 in., 34.925 cm. --> 0.034925cm/MU
        //diag right 13.75 in., 34.925 cm. --> 0.034925/MU
        //diag forward 15.76 in., 40.0304 cm. --> 0.0400304cm/MU
        //average 0.0419372cm/MU forward
        double finalY=ydistance/0.034925;
        double finalX=xdistance/0.0419372;
        double[] ans={finalX,finalY};
        return ans;
    }

    /**
     * Update robot position using IMU
     *
     * @return Updated coordinates in an array [xPos, yPos, theta]
     */
    protected double[] updatePos() {
        //update theta
        this.theta = this.imu.getHeading();
        //read motor encoder changes
        int[] past_encoders;
        int[] current_encoders;
        double[] decomposed_motor_vectors = new double[4];
        //robot vectors storage, [x,y]
        double[] robot_vectors = {0, 0};
        past_encoders = this.drivetrain_save.readMotorEncoders();
        current_encoders = this.drivetrain.readMotorEncoders();
        for (int i = 0; i < 4; i++) {
            decomposed_motor_vectors[i] = Math.sqrt(Math.pow(current_encoders[i] - past_encoders[i], 2) / 2);
            if (current_encoders[i] - past_encoders[i] < 0) {
                decomposed_motor_vectors[i] *= -1;
            }
        }
        //calculate robot vectors
        //positive x and y facing vectors(LF and RR)
        robot_vectors[0] = robot_vectors[0] + decomposed_motor_vectors[0] + decomposed_motor_vectors[2];
        robot_vectors[1] = robot_vectors[1] + decomposed_motor_vectors[0] + decomposed_motor_vectors[2];
        //negative x and positive y facing vectors(LR and RF)
        robot_vectors[0] = robot_vectors[0] - decomposed_motor_vectors[1] - decomposed_motor_vectors[3];
        robot_vectors[1] = robot_vectors[1] + decomposed_motor_vectors[1] + decomposed_motor_vectors[3];
        //add final vectors to robot position
        this.xPos = robot_vectors[0] + this.xPos;
        this.yPos = robot_vectors[1] + this.yPos;
        return new double[]{this.xPos, this.yPos, this.theta};
    }
}