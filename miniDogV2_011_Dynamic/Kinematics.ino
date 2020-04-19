void kinematics (int leg, int mode, float x, float y, float z, float roll, float pitch, float yaw) {

      // *** TRANSLATION AXIS ***

    // moving the foot sideways on the end plane
    #define hipOffset 76.5
    float lengthY;
    float hipAngle1a;
    float hipAngle1b;
    float hipAngle1;
    float hipAngle1Degrees;
    float hipHyp;
    
    // moving the foot forwards or backwardes in the side plane
    float shoulderAngle2;
    float shoulderAngle2Degrees;
    float z2;    

    // side plane of individual leg only
    #define shinLength 125     
    #define thighLength 125
    float z3;
    float shoulderAngle1;
    float shoulderAngle1Degrees;
    float shoulderAngle1a;   
    float shoulderAngle1b;
    float shoulderAngle1c;
    float shoulderAngle1d;
    float kneeAngle;  
    float kneeAngleDegrees; 

    // *** ROTATION AXIS

    // roll axis
    #define bodyWidth 126      // half the distance from the middle of the body to the hip pivot  
    float legDiffRoll;            // differnece in height for each leg
    float bodyDiffRoll;           // how much shorter the 'virtual body' gets
    float footDisplacementRoll;   // where the foot actually is
    float footDisplacementAngleRoll; // smaller angle
    float footWholeAngleRoll;     // whole leg angle
    float hipRollAngle;       // angle for hip when roll axis is in use
    float rollAngle;          // angle in RADIANS that the body rolls
    float zz1a;               // hypotenuse of final triangle
    float zz1;                // new height for leg to pass onto the next bit of code
    float yy1;                // new position for leg to move sideways

    // pitch axis

    #define bodyLength 147.5      // distance from centre of body to shoulder pivot
    float legDiffPitch;            // differnece in height for each leg
    float bodyDiffPitch;           // how much shorter the 'virtual body' gets
    float footDisplacementPitch;   // where the foot actually is
    float footDisplacementAnglePitch; // smaller angle
    float footWholeAnglePitch;     // whole leg angle
    float shoulderPitchAngle;      // angle for hip when roll axis is in use
    float pitchAngle;              // angle in RADIANS that the body rolls
    float zz2a;                    // hypotenuse of final triangle
    float zz2;                     // new height for the leg to pass onto the next bit of code
    float xx1;                     // new position to move the leg fowwards/backwards
    
    // yaw axis 

    float yawAngle;                 // angle in RADIANs for rotation in yaw
    float existingAngle;            // existing angle of leg from centre
    float radius;                   // radius of leg from centre of robot based on x and y from sticks
    float demandYaw;                // demand yaw postion - existing yaw plus the stick yaw 
    float xx3;                      // new X coordinate based on demand angle 
    float yy3;                      // new Y coordinate based on demand angle

    // ******************************************************************************************************
    // ***************************** KINEMATIC MODEL CALCS START HERE ***************************************
    // ******************************************************************************************************

    // convert degrees to radians for the calcs
    yawAngle = (PI/180) * yaw;

    // put in offsets from robot's parameters so we can work out the radius of the foot from the robot's centre
    if (leg == 1) {         // front left leg
       y = y - (bodyWidth+hipOffset); 
       x = x - bodyLength;      
    }
    else if (leg == 2) {    // front right leg
       y = y + (bodyWidth+hipOffset);
       x = x - bodyLength; 
    }
    else if (leg == 3) {    // back left leg
       y = y - (bodyWidth+hipOffset); 
       x = x + bodyLength;
    }
    else if (leg == 4) {    // back left leg
       y = y + (bodyWidth+hipOffset); 
       x = x + bodyLength;
    }

    //calc existing angle of leg from cetre
    existingAngle = atan(y/x);   

    // calc radius from centre
    radius = y/sin(existingAngle);

    //calc demand yaw angle
    demandYaw = existingAngle + yawAngle;

    // calc new X and Y based on demand yaw angle
    xx3 = radius * cos(demandYaw);           // calc new X and Y based on new yaw angle
    yy3 = radius * sin(demandYaw);

    // remove the offsets so we pivot around 0/0 x/y
    if (leg == 1) {         // front left leg
       yy3 = yy3 + (bodyWidth+hipOffset); 
       xx3 = xx3 + bodyLength;      
    }
    else if (leg == 2) {    // front right leg
       yy3 = yy3 - (bodyWidth+hipOffset);
       xx3 = xx3 + bodyLength; 
    }
    else if (leg == 3) {    // back left leg
       yy3 = yy3 + (bodyWidth+hipOffset); 
       xx3 = xx3 - bodyLength;
    }
    else if (leg == 4) {    // back left leg
       yy3 = yy3 - (bodyWidth+hipOffset); 
       xx3 = xx3 - bodyLength;
    }

    // *** PITCH AXIS ***

    //turn around the pitch for front or back of the robot
    if (leg == 1 || leg == 2) {
      pitch = 0-pitch;      
    }
    else if (leg == 3 || leg == 4) {
      pitch = 0+pitch;
      xx3 = xx3*-1;       // switch over x for each end of the robot
    }

    // convert pitch to degrees
    pitchAngle = (PI/180) * pitch;

    //calc top triangle sides
    legDiffPitch = sin(pitchAngle) * bodyLength;
    bodyDiffPitch = cos(pitchAngle) * bodyLength;

    // calc actual height from the ground for each side
    legDiffPitch = z - legDiffPitch;

    // calc foot displacement
    footDisplacementPitch = ((bodyDiffPitch - bodyLength)*-1)+xx3;

    //calc smaller displacement angle
    footDisplacementAnglePitch = atan(footDisplacementPitch/legDiffPitch);

    //calc distance from the ground at the displacement angle (the hypotenuse of the final triangle)
    zz2a = legDiffPitch/cos(footDisplacementAnglePitch);

    // calc the whole angle for the leg
    footWholeAnglePitch = footDisplacementAnglePitch + pitchAngle;

    //calc actual leg length - the new Z to pass on
    zz2 = cos(footWholeAnglePitch) * zz2a;

    //calc new Z to pass on
    xx1 = sin(footWholeAnglePitch) * zz2a;

    if (leg == 3 || leg == 4 ){     // switch back X for the back of the robot
      xx1 = xx1 *-1;
    }


    // *** ROLL AXIS ***

    //turn around roll angle for each side of the robot
    if (leg == 1 || leg == 3) {
      roll = 0+roll;
      yy3 = yy3*-1;
    }
    else if (leg == 2 || leg == 4) {
      roll = 0-roll;
    }   

    // convert roll angle to radians
    rollAngle = (PI/180) * roll;    //covert degrees from the stick to radians

    // calc the top triangle sides
    legDiffRoll = sin(rollAngle) * bodyWidth;
    bodyDiffRoll = cos(rollAngle) * bodyWidth;
    
    // calc actual height from the ground for each side
    legDiffRoll = zz2 - legDiffRoll;              

    // calc foot displacement
    footDisplacementRoll = ((bodyDiffRoll - bodyWidth)*-1)-yy3;

    //calc smaller displacement angle
    footDisplacementAngleRoll = atan(footDisplacementRoll/legDiffRoll);  

    //calc distance from the ground at the displacement angle (the hypotenuse of the final triangle)
    zz1a = legDiffRoll/cos(footDisplacementAngleRoll);

    // calc the whole angle for the leg
    footWholeAngleRoll = footDisplacementAngleRoll + rollAngle;

    //calc actual leg length - the new Z to pass on
    zz1 = cos(footWholeAngleRoll) * zz1a;

    //calc new Y to pass on
    yy1 = sin(footWholeAngleRoll) * zz1a;  

    // *** TRANSLATION AXIS ***  
  
    // calculate the hip joint and new leg length based on how far the robot moves sideways
    hipAngle1 = atan(yy1/zz1);    
    hipAngle1Degrees = ((hipAngle1 * (180/PI)));   // convert to degrees and take off rest position
    z2 = zz1/cos(hipAngle1);

    // ****************

    // calculate the shoulder joint offset and new leg length based on now far the foot moves forward/backwards
    shoulderAngle2 = atan(xx1/z2);     // calc how much extra to add to the shoulder joint
    shoulderAngle2Degrees = shoulderAngle2 * (180/PI);
    z3 = z2/cos(shoulderAngle2);     // calc new leg length to feed to the next bit of code below

    // ****************
    
    // calculate leg length based on shin/thigh length and knee and shoulder angle
    shoulderAngle1a = sq(thighLength) + sq(z3) - sq(shinLength);
    shoulderAngle1b = 2 * thighLength * z3;
    shoulderAngle1c = shoulderAngle1a / shoulderAngle1b;
    shoulderAngle1 = acos(shoulderAngle1c);     // radians
    kneeAngle = PI - (shoulderAngle1 *2);       // radians

    //calc degrees from angles
    shoulderAngle1Degrees = shoulderAngle1 * (180/PI);    // degrees
    kneeAngleDegrees = kneeAngle * (180/PI);              // degrees     
   
  
    // ******************************************************************************************************
    // ***************** compliance / filtering / write out servo positoion below this point *****************
    // ******************************************************************************************************

    if (leg == 1) {           // *front left leg*

        // convert degrees to servo microSeconds
        servo11PosTrack = (kneeAngleDegrees-90) * -25;           // positive scaler
        servo7PosTrack = ((shoulderAngle1Degrees-45) * 25) - (shoulderAngle2Degrees * 25) ;       
        servo3PosTrack = (hipAngle1Degrees * -25);
      
        // no compliance mode
        if (mode == 0) {
            servo3Pos = servo3PosTrack;   // front left hip
            servo7Pos = servo7PosTrack;   // front left shoulder
            servo11Pos = servo11PosTrack; // front right knee
        }  
        // compliance mode
        else if (mode == 1) {
            //*** front left hip ***
            if (hall2 > threshholdGlobal || hall2 < (threshholdGlobal*-1)) {
              servo3Pos = servo3Pos + (hall2 * multiplierHipsLeft);
            }
            else {      // return to centre
                servo3Pos = servo3PosTrack;
            }
            //*** front left shoulder
            if (hall3 > threshholdGlobal || hall3 < (threshholdGlobal*-1)) {
              servo7Pos = servo7Pos + (hall3 * multiplierShouldersLeft);
            }
            else {      // return to centre
                servo7Pos = servo7PosTrack;
            }
            // ***front left knee
            if (hall4 > threshholdGlobal || hall4 < (threshholdGlobal*-1)) {
              servo11Pos = servo11Pos + (hall4 * multiplierKneesLeft);
            }
            else {      // return to centre
                servo11Pos = servo11PosTrack;
            }            
        }
        // filter motions
        servo3PosFiltered = filter(servo3Pos, servo3PosFiltered, filterHipsLeft);
        servo3PosFiltered = constrain(servo3PosFiltered,-900,900);
        servo7PosFiltered = filter(servo7Pos, servo7PosFiltered, filterShouldersLeft);
        servo7PosFiltered = constrain(servo7PosFiltered,-900,900);
        servo11PosFiltered = filter(servo11Pos, servo11PosFiltered, filterKneesLeft);
        servo11PosFiltered = constrain(servo11PosFiltered,-900,900);
        // write out servo positions
        servo3.writeMicroseconds(servo3Offset + servo3PosFiltered);     // front left hip
        servo7.writeMicroseconds(servo7Offset + servo7PosFiltered);     // front left shoulder
        servo11.writeMicroseconds(servo11Offset + servo11PosFiltered);     // front right knee        
    }

    else if (leg == 2) {      // *front right leg*

        // convert degrees to servo microSeconds
        servo12PosTrack = (kneeAngleDegrees-90) * 25;
        servo8PosTrack = ((shoulderAngle1Degrees-45) * -25) + (shoulderAngle2Degrees * 25);    
        servo4PosTrack = (hipAngle1Degrees * 25);
        
        // no compliance mode
        if (mode == 0) {      
            servo4Pos = servo4PosTrack;   // front right hip
            servo8Pos = servo8PosTrack;   // front right shoulder
            servo12Pos = servo12PosTrack; // front right knee        
        } 
        // compliance mode
        if (mode == 1) {
            //*** front right hip ***            
            if (hall1 > threshholdGlobal || hall1 < (threshholdGlobal*-1)) {
              servo4Pos = servo4Pos + (hall1 * multiplierHipsRight);
            }
            else {      // return to centre
                servo4Pos = servo4PosTrack;
            }
            //*** front right shoulder
            if (hall6 > threshholdGlobal || hall6 < (threshholdGlobal*-1)) {
              servo8Pos = servo8Pos + (hall6 * multiplierShouldersRight);
            }
            else {      // return to centre
                servo8Pos = servo8PosTrack;
            }
            // ***front right knee
            if (hall5 > threshholdGlobal || hall5 < (threshholdGlobal*-1)) {
              servo12Pos = servo12Pos + (hall5 * multiplierKneesRight);
            }
            else {      // return to centre
                servo12Pos = servo12PosTrack;
            }               
        }
        // filter motions
        servo4PosFiltered = filter(servo4Pos, servo4PosFiltered, filterHipsRight);
        servo4PosFiltered = constrain(servo4PosFiltered,-900,900);
        servo8PosFiltered = filter(servo8Pos, servo8PosFiltered, filterShouldersRight);
        servo8PosFiltered = constrain(servo8PosFiltered,-900,900);
        servo12PosFiltered = filter(servo12Pos, servo12PosFiltered, filterKneesRight);
        servo12PosFiltered = constrain(servo12PosFiltered,-900,900);  
        // write out servo potitions
        servo4.writeMicroseconds(servo4Offset + servo4PosFiltered);     // front right hip     
        servo8.writeMicroseconds(servo8Offset + servo8PosFiltered);     // front right shoulder 
        servo12.writeMicroseconds(servo12Offset + servo12PosFiltered);     // front right knee   
    }

    else if (leg == 3) {      // *back left leg*

        // convert degrees to servo microSeconds
        servo9PosTrack = (kneeAngleDegrees-90) * -25;           
        servo5PosTrack = ((shoulderAngle1Degrees-45) * 25) - (shoulderAngle2Degrees * 25);   
        servo1PosTrack = (hipAngle1Degrees * -25);
        
        // no compliance mode
        if (mode == 0) {
            servo1Pos = servo1PosTrack;   // back left hip
            servo5Pos = servo5PosTrack;   // back left shoulder
            servo9Pos = servo9PosTrack;   // back left knee
        } 
        // compliance mode
        if (mode == 1) {
            //*** back left hip ***
            if (hall7 > threshholdGlobal || hall7 < (threshholdGlobal*-1)) {
              servo1Pos = servo1Pos + (hall7 * multiplierHipsLeft);
            }
            else {      // return to centre
                servo1Pos = servo1PosTrack;
            }
            //*** back left shoulder
            if (hall12 > threshholdGlobal || hall12 < (threshholdGlobal*-1)) {
              servo5Pos = servo5Pos + (hall12 * multiplierShouldersLeft);
            }
            else {      // return to centre
                servo5Pos = servo5PosTrack;
            }
            // ***back left knee
            if (hall11 > threshholdGlobal || hall11 < (threshholdGlobal*-1)) {
              servo9Pos = servo9Pos + (hall11 * multiplierKneesLeft);
            }
            else {      // return to centre
                servo9Pos = servo9PosTrack;
            }               
        }
        // filter motions
        servo1PosFiltered = filter(servo1Pos, servo1PosFiltered, filterHipsLeft);
        servo1PosFiltered = constrain(servo1PosFiltered,-900,900);
        servo5PosFiltered = filter(servo5Pos, servo5PosFiltered, filterShouldersLeft);
        servo5PosFiltered = constrain(servo5PosFiltered,-900,900); 
        servo9PosFiltered = filter(servo9Pos, servo9PosFiltered, filterKneesLeft);
        servo9PosFiltered = constrain(servo9PosFiltered,-900,900);
        // write out servo positions
        servo1.writeMicroseconds(servo1Offset + servo1PosFiltered);     // back left hip
        servo5.writeMicroseconds(servo5Offset + servo5PosFiltered);     // back left shoulder
        servo9.writeMicroseconds(servo9Offset + servo9PosFiltered);     // front left knee        
    }

    else if (leg == 4) {      // *back right leg*

        // convert degrees to servo microSeconds
        servo10PosTrack = (kneeAngleDegrees-90) * 25;           
        servo6PosTrack = ((shoulderAngle1Degrees-45) * -25) + (shoulderAngle2Degrees * 25);  
        servo2PosTrack = (hipAngle1Degrees * 25);

        // no compliance mode
        if (mode == 0) {
            servo2Pos = servo2PosTrack;   // back right hip
            servo6Pos = servo6PosTrack;   // back right shoulder
            servo10Pos = servo10PosTrack; // back right knee
        }  
        // compliance mode
        if (mode == 1) {
            //*** back right hip ***
            if (hall8 > threshholdGlobal || hall8 < (threshholdGlobal*-1)) {
              servo2Pos = servo2Pos + (hall8 * multiplierHipsRight);
            }
            else {      // return to centre
                servo2Pos = servo2PosTrack;
            }
            //*** back right shoulder
            if (hall9 > threshholdGlobal || hall9 < (threshholdGlobal*-1)) {
              servo6Pos = servo6Pos + (hall9 * multiplierShouldersRight);
            }
            else {      // return to centre
                servo6Pos = servo6PosTrack;
            }
            // ***back right knee
            if (hall10 > threshholdGlobal || hall10 < (threshholdGlobal*-1)) {
              servo10Pos = servo10Pos + (hall10 * multiplierKneesRight);
            }
            else {      // return to centre
                servo10Pos = servo10PosTrack;
            }               
        }
        // filter motions
        servo2PosFiltered = filter(servo2Pos, servo2PosFiltered, filterHipsRight);
        servo2PosFiltered = constrain(servo2PosFiltered,-900,900);
        servo6PosFiltered = filter(servo6Pos, servo6PosFiltered, filterShouldersRight);
        servo6PosFiltered = constrain(servo6PosFiltered,-900,900); 
        servo10PosFiltered = filter(servo10Pos, servo10PosFiltered, filterKneesRight);
        servo10PosFiltered = constrain(servo10PosFiltered,-900,900);
        // write out servo positions
        servo2.writeMicroseconds(servo2Offset + servo2PosFiltered);     // back right hip
        servo6.writeMicroseconds(servo6Offset + servo6PosFiltered);     // back left shoulder
        servo10.writeMicroseconds(servo10Offset + servo10PosFiltered);     // front left knee
    }            
        
              

} // end of kinematics
