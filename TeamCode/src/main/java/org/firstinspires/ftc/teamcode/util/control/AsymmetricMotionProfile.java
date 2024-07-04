package org.firstinspires.ftc.teamcode.util.control;

public class AsymmetricMotionProfile {

    private double maxVelocity;
    private double acceleration;
    private double deceleration;
    private double startPose;
    private double setPose;
    private boolean isFlipped = false;
    private double startTime;
    private double endTime;

    public AsymmetricMotionProfile(double startPose, double setPose, ProfileConstraints profileConstraints) {
        this.startPose = startPose;
        this.setPose = Math.abs(setPose - startPose);

        maxVelocity = profileConstraints.maxVelocity * 10;
        acceleration = profileConstraints.acceleration * 10;
        deceleration = -profileConstraints.deceleration * 10;

        startTime = maxVelocity / acceleration; // what the sigma
        endTime = maxVelocity / -deceleration;

        if(setPose < startPose) {
            this.startPose = startPose;
            this.setPose = setPose;
            isFlipped = true;

            double temp = this.setPose;
            this.setPose = this.startPose - temp;
            this.startPose = temp;
        }

    }

    private double velocityStartFunction(double time) {
        return acceleration * time;
    }

    private double velocityMiddleFunction(double time) {
        return maxVelocity;
    }

    private double velocityEndFunction(double time) {
        return deceleration * (time - endTime);
    }


    private double startFunctionDisplacement(double time) {
        return (velocityStartFunction(time) * time) / 2;
    }

    private double middleFunctionDisplacement(double time) {
        return maxVelocity * time;
    }

    private double endFunctionDisplacement(double time) {
        return ((maxVelocity + velocityEndFunction(time)) * time) / 2;
    }

    public MotionState getMotionState(double currentTime) {
        MotionState state = new MotionState(0,0,0);


        double finalPositionStart = startFunctionDisplacement(startTime);
        double finalPositionEnd = endFunctionDisplacement(endTime);

        double finalPositionMiddle = setPose - finalPositionStart - finalPositionEnd;

        double middleTime = finalPositionMiddle / maxVelocity;

        if(middleTime < 0) {

             //find overshoot
            double s = (finalPositionStart + finalPositionEnd) - setPose;

            //credit to Photomath <3
            double a = -(Math.pow(acceleration, 2)) + (deceleration * acceleration);
            double b = (maxVelocity * acceleration) - (acceleration * deceleration * (startTime + endTime)) - (deceleration * maxVelocity);
            double c = maxVelocity * deceleration * (startTime + endTime) - 2 * (deceleration * s);


            double discriminant = (b * b) - 4 * a * c;
            double startTimeNew1 = (-b + Math.sqrt(discriminant)) / (2 * a);
            double startTimeNew2 = (-b - Math.sqrt(discriminant)) / (2 * a);

            double startTimeOld = startTime;

            if(startTimeNew1 > startTime) {
                startTime = startTimeNew2;
            } else {
                startTime = startTimeNew1;
            }

            middleTime = (((acceleration * startTime) + deceleration * (startTimeOld + endTime)) / deceleration);
            endTime = (endTime + startTimeOld) - middleTime;

            middleTime -= startTime;
            maxVelocity = velocityStartFunction(startTime);

            finalPositionMiddle = middleFunctionDisplacement(middleTime);


        }

        if(currentTime <= startTime) {
            state.a = acceleration;
            state.v = velocityStartFunction(currentTime);
            state.x = startFunctionDisplacement(currentTime);
        } else if(currentTime <= startTime + middleTime) {
            currentTime -= startTime;
            state.a = 0;
            state.v = velocityMiddleFunction(currentTime);
            state.x = finalPositionStart + middleFunctionDisplacement(currentTime);
        } else if(currentTime <= startTime + middleTime + endTime){
            currentTime -= (startTime + middleTime);
            state.a = -deceleration;
            state.v = velocityEndFunction(currentTime);
            state.x = finalPositionStart + finalPositionMiddle + endFunctionDisplacement(currentTime);
        } else {
            state.a = 0;
            state.v = 0;
            state.x = setPose;
        }

        if(isFlipped) {
            state.x = (setPose - state.x) + startPose;
        } else {
            state.x += startPose;
        }

        return state;
    }
}