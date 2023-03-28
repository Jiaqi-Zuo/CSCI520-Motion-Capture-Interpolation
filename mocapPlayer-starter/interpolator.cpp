#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"
#include <iostream>
#include "performanceCounter.h"

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
    LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
    BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
}

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    PerformanceCounter counter;
    counter.StartCounter();
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }
  counter.StopCounter();
  std::cout << "Computation time: " << counter.GetElapsedTime() << std::endl;
  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
    // convert from degree to radian
    double psi = angles[0] * M_PI / 180, theta = angles[1] * M_PI / 180, phi = angles[2] * M_PI / 180;
    /*
     R = Rz(¦Õ)Ry(¦È)Rx(¦×)
    */
    R[0] = cos(theta) * cos(phi);
    R[1] = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi);
    R[2] = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi);
    R[3] = cos(theta) * sin(phi);
    R[4] = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi);
    R[5] = cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi);
    R[6] = -sin(theta); R[7] = sin(psi) * cos(theta); R[8] = cos(psi) * cos(theta);
}

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    PerformanceCounter counter;
    counter.StartCounter();
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    int startKeyframe = 0;
    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;       
        
        Posture* startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture* endPosture = pInputMotion->GetPosture(endKeyframe);

        // control points
        Posture an, bn;

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);
      
        vector rootHat, boneHat;
        // interpolate in between
        for (int frame = 1; frame <= N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N + 1);
            // interpolate root position
            if (startKeyframe == 0) {            
                int nextKeyframe = endKeyframe + N + 1; Posture *nextPosture = pInputMotion->GetPosture(nextKeyframe);               
                // q_n-1, q_n, q_n+1
                vector q1 = startPosture->root_pos, q2 = endPosture->root_pos, q3 = nextPosture->root_pos;
                // a1 = lerp(q1, lerp(q3, q2, 2.0), 1.0 / 3)
                rootHat = q3 + (q2 - q3) * (2.0);
                an.root_pos = q1 + (rootHat - q1) * ((1.0) / 3);

                // ahat = lerp(lerp(qn-1, qn, 2.0), qn+1, 0.5)
                // bn = lerp(qn , ahat , -1.0 / 3)
                rootHat = q1 + (q2 - q1) * 2.0;
                rootHat = rootHat + (q3 - rootHat) * 0.5;
                bn.root_pos = q2 + (rootHat - q2) * ((-1.0) / 3);
            }
            else if ((endKeyframe + N + 1) >= inputLength) {
                int prevKeyframe = startKeyframe - N - 1; Posture* prevPosture = pInputMotion->GetPosture(prevKeyframe);
                // q_n-1, q_n, q_n+1
                vector q1 = prevPosture->root_pos, q2 = startPosture->root_pos, q3 = endPosture->root_pos;
                //ahat = lerp(lerp(qn - 1, qn, 2.0), qn + 1, 0.5)
                //an = lerp(qn, ahat, 1.0 / 3)
                rootHat = q1 + (q2 - q1) * 2.0;
                rootHat = rootHat + (q3 - rootHat) * 0.5;
                an.root_pos = q2 + (rootHat - q2) * ((1.0) / 3);

                // bN = lerp(qN, lerp(qN-2, qN-1, 2.0), 1.0 / 3)
                rootHat = q1 + (q2 - q1) * 2.0;
                bn.root_pos = q3 + (rootHat - q3) * ((1.0) / 3);
            }
            else {                
                int prevKeyframe = startKeyframe - N - 1; Posture* prevPosture = pInputMotion->GetPosture(prevKeyframe);
                int nextKeyframe = endKeyframe + N + 1; Posture* nextPosture = pInputMotion->GetPosture(nextKeyframe);
                // q_n-1, q_n, q_n+1, q_n+2
                vector q1 = prevPosture->root_pos, q2 = startPosture->root_pos, q3 = endPosture->root_pos, q4 = nextPosture->root_pos;
                //ahat = lerp(lerp(qn - 1, qn, 2.0), qn + 1, 0.5)
                //an = lerp(qn, ahat, 1.0 / 3)
                rootHat = q1 + (q2 - q1) * 2.0;
                rootHat = rootHat + (q3 - rootHat) * 0.5;
                an.root_pos = q2 + (rootHat - q2) * ((1.0) / 3);
                //ahat = lerp(lerp(qn, qn+1, 2.0), qn + 2, 0.5)
                //bn = lerp(qn+1, ahat, -1.0 / 3)
                rootHat = q2 + (q3 - q2) * 2.0;
                rootHat = rootHat + (q4 - rootHat) * 0.5;
                bn.root_pos = q3 + (rootHat - q3) * ((-1.0) / 3);
            }
            interpolatedPosture.root_pos = DeCasteljauEuler(t, startPosture->root_pos, an.root_pos, bn.root_pos, endPosture->root_pos);

            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
                if (startKeyframe == 0) {
                    int nextKeyframe = endKeyframe + N + 1; Posture* nextPosture = pInputMotion->GetPosture(nextKeyframe);
                    // a1 = lerp(q1, lerp(q3, q2, 2.0), 1.0 / 3)
                    vector q1 = startPosture->bone_rotation[bone], q2 = endPosture->bone_rotation[bone], q3 = nextPosture->bone_rotation[bone];
                    rootHat = q3 + (q2 - q3) * (2.0);
                    an.bone_rotation[bone] = q1 + (rootHat - q1) * ((1.0) / 3);

                    // ahat = lerp(lerp(qn-1, qn, 2.0), qn+1, 0.5)
                    // bn = lerp(qn , ahat , -1.0 / 3)
                    rootHat = q1 + (q2 - q1) * 2.0;
                    rootHat = rootHat + (q3 - rootHat) * 0.5;
                    bn.bone_rotation[bone] = q2 + (rootHat - q2) * ((-1.0) / 3);
                }
                else if ((endKeyframe+N+1) >= inputLength) {
                    int prevKeyframe = startKeyframe - N - 1; Posture* prevPosture = pInputMotion->GetPosture(prevKeyframe);
                    vector q1 = prevPosture->bone_rotation[bone], q2 = startPosture->bone_rotation[bone], q3 = endPosture->bone_rotation[bone];
                    //ahat = lerp(lerp(qn - 1, qn, 2.0), qn + 1, 0.5)
                    //an = lerp(qn, ahat, 1.0 / 3)
                    rootHat = q1 + (q2 - q1) * 2.0;
                    rootHat = rootHat + (q3 - rootHat) * 0.5;
                    an.bone_rotation[bone] = q2 + (rootHat - q2) * ((1.0) / 3);

                    // bN = lerp(qN, lerp(qN-2, qN-1, 2.0), 1.0 / 3)
                    rootHat = q1 + (q2 - q1) * 2.0;
                    bn.bone_rotation[bone] = q3 + (rootHat - q3) * ((1.0) / 3);
                }
                else {
                    int prevKeyframe = startKeyframe - N - 1; Posture* prevPosture = pInputMotion->GetPosture(prevKeyframe);
                    int nextKeyframe = endKeyframe + N + 1; Posture* nextPosture = pInputMotion->GetPosture(nextKeyframe);
                    vector q1 = prevPosture->bone_rotation[bone], q2 = startPosture->bone_rotation[bone], 
                        q3 = endPosture->bone_rotation[bone], q4 = nextPosture->bone_rotation[bone];
                    //ahat = lerp(lerp(qn - 1, qn, 2.0), qn + 1, 0.5)
                    //an = lerp(qn, ahat, 1.0 / 3)
                    rootHat = q1 + (q2 - q1) * 2.0;
                    rootHat = rootHat + (q3 - rootHat) * 0.5;
                    an.bone_rotation[bone] = q2 + (rootHat - q2) * ((1.0) / 3);
                    //ahat = lerp(lerp(qn, qn+1, 2.0), qn + 2, 0.5)
                    //bn = lerp(qn+1, ahat, -1.0 / 3)
                    rootHat = q2 + (q3 - q2) * 2.0;
                    rootHat = rootHat + (q4 - rootHat) * 0.5;
                    bn.bone_rotation[bone] = q3 + (rootHat - q3) * ((-1.0) / 3);
                }
                // use DeCasteljau to evaluate Bezier curve
                interpolatedPosture.bone_rotation[bone] = 
                    DeCasteljauEuler(t, startPosture->bone_rotation[bone], an.bone_rotation[bone], bn.bone_rotation[bone], endPosture->bone_rotation[bone]);
            }
                
            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }
        startKeyframe = endKeyframe;
    }
    counter.StopCounter();
    std::cout << "Computation time: " << counter.GetElapsedTime() << std::endl;
    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    PerformanceCounter counter;
    counter.StartCounter();
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    int startKeyframe = 0;
    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;

        Posture* startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture* endPosture = pInputMotion->GetPosture(endKeyframe);

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        // interpolate in between
        for (int frame = 1; frame <= N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N + 1);

            // interpolate root position
            interpolatedPosture.root_pos = startPosture->root_pos * (1 - t) + endPosture->root_pos * t;
            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
                // we first convert the individual bones' rotation of startPosture and endPosture from euler to quaternion
                Quaternion<double> startBone; Euler2Quaternion(startPosture->bone_rotation[bone].p, startBone);
                Quaternion<double> endBone; Euler2Quaternion(endPosture->bone_rotation[bone].p, endBone);                              
                // we then apply SLERP for the rotations
                Quaternion<double> interBone = Slerp(t, startBone, endBone);
                // finally we convert the bone from quaternion back to euler and apply to the posture
                Quaternion2Euler(interBone, interpolatedPosture.bone_rotation[bone].p);                          
            }                       
            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }
        startKeyframe = endKeyframe;
    }
    counter.StopCounter();
    std::cout << "Computation time: " << counter.GetElapsedTime() << std::endl;
    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    PerformanceCounter counter;
    counter.StartCounter();
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    int startKeyframe = 0;
    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;        
        Posture* startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture* endPosture = pInputMotion->GetPosture(endKeyframe);
        
        // control points
        Posture an, bn;
        Quaternion<double> anBone, bnBone;
        
        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);
        vector rootHat;
        // interpolate in between
        for (int frame = 1; frame <= N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N + 1);
            // interpolate root position
            if (startKeyframe == 0) {
                int nextKeyframe = endKeyframe + N + 1; Posture* nextPosture = pInputMotion->GetPosture(nextKeyframe);
                // a1 = lerp(q1, lerp(q3, q2, 2.0), 1.0 / 3)
                vector q1 = startPosture->root_pos, q2 = endPosture->root_pos, q3 = nextPosture->root_pos;
                rootHat = q3 + (q2 - q3) * (2.0);
                an.root_pos = q1 + (rootHat - q1) * ((1.0) / 3);

                // ahat = lerp(lerp(qn-1, qn, 2.0), qn+1, 0.5)
                // bn = lerp(qn , ahat , -1.0 / 3)
                rootHat = q1 + (q2 - q1) * 2.0;
                rootHat = rootHat + (q3 - rootHat) * 0.5;
                bn.root_pos = q2 + (rootHat - q2) * ((-1.0) / 3);
            }
            else if ((endKeyframe+N+1) >= inputLength) {
                int prevKeyframe = startKeyframe - N - 1; Posture* prevPosture = pInputMotion->GetPosture(prevKeyframe);
                vector q1 = prevPosture->root_pos, q2 = startPosture->root_pos, q3 = endPosture->root_pos;
                //ahat = lerp(lerp(qn - 1, qn, 2.0), qn + 1, 0.5)
                //an = lerp(qn, ahat, 1.0 / 3)
                rootHat = q1 + (q2 - q1) * 2.0;
                rootHat = rootHat + (q3 - rootHat) * 0.5;
                an.root_pos = q2 + (rootHat - q2) * ((1.0) / 3);

                // bN = lerp(qN, lerp(qN-2, qN-1, 2.0), 1.0 / 3)
                rootHat = q1 + (q2 - q1) * 2.0;
                bn.root_pos = q3 + (rootHat - q3) * ((1.0) / 3);
            }
            else {
                int prevKeyframe = startKeyframe - N - 1; Posture* prevPosture = pInputMotion->GetPosture(prevKeyframe);
                int nextKeyframe = endKeyframe + N + 1; Posture* nextPosture = pInputMotion->GetPosture(nextKeyframe);
                vector q1 = prevPosture->root_pos, q2 = startPosture->root_pos, q3 = endPosture->root_pos, q4 = nextPosture->root_pos;
                //ahat = lerp(lerp(qn - 1, qn, 2.0), qn + 1, 0.5)
                //an = lerp(qn, ahat, 1.0 / 3)
                rootHat = q1 + (q2 - q1) * 2.0;
                rootHat = rootHat + (q3 - rootHat) * 0.5;
                an.root_pos = q2 + (rootHat - q2) * ((1.0) / 3);
                //ahat = lerp(lerp(qn, qn+1, 2.0), qn + 2, 0.5)
                //bn = lerp(qn+1, ahat, -1.0 / 3)
                rootHat = q2 + (q3 - q2) * 2.0;
                rootHat = rootHat + (q4 - rootHat) * 0.5;
                bn.root_pos = q3 + (rootHat - q3) * ((-1.0) / 3);
            }
            interpolatedPosture.root_pos = DeCasteljauEuler(t, startPosture->root_pos, an.root_pos, bn.root_pos, endPosture->root_pos);
            
            Quaternion<double> boneHat;
            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
                if (startKeyframe == 0) {
                    int nextKeyframe = endKeyframe + N + 1; Posture* nextPosture = pInputMotion->GetPosture(nextKeyframe);
                    Quaternion<double> q1; Euler2Quaternion(startPosture->bone_rotation[bone].p, q1);
                    Quaternion<double> q2; Euler2Quaternion(endPosture->bone_rotation[bone].p, q2);
                    Quaternion<double> q3; Euler2Quaternion(nextPosture->bone_rotation[bone].p, q3);
                    //an = Slerp(q1, Double(q3, q2), 1.0 / 3)
                    anBone = Slerp((1.0) / 3, q1, Double(q3, q2));
                    //ahat = Slerp(Double(qn-1, qn), qn+1, 0.5)
                    //bn = Slerp(qn , ahat , -1.0 / 3)
                    boneHat = Slerp(0.5, Double(q1, q2), q3);
                    bnBone = Slerp((-1.0) / 3, q2, boneHat);
                }
                else if ((endKeyframe+N+1) >= inputLength) {
                    int prevKeyframe = startKeyframe - N - 1; Posture* prevPosture = pInputMotion->GetPosture(prevKeyframe);
                    Quaternion<double> q1; Euler2Quaternion(prevPosture->bone_rotation[bone].p, q1);
                    Quaternion<double> q2; Euler2Quaternion(startPosture->bone_rotation[bone].p, q2);
                    Quaternion<double> q3; Euler2Quaternion(endPosture->bone_rotation[bone].p, q3);
                    //ahat = Slerp(Double(qn - 1, qn), qn + 1, 0.5)
                    //an = Slerp(qn, ahat, 1.0 / 3)
                    boneHat = Slerp(0.5, Double(q1, q2), q3);
                    anBone = Slerp((1.0) / 3, q2, boneHat);
                    //bN = Slerp(qN, Double(qN-2, qN-1), 1.0 / 3)
                    bnBone = Slerp((1.0) / 3, q3, Double(q1, q2));
                }
                else {
                    int prevKeyframe = startKeyframe - N - 1; Posture* prevPosture = pInputMotion->GetPosture(prevKeyframe);
                    int nextKeyframe = endKeyframe + N + 1; Posture* nextPosture = pInputMotion->GetPosture(nextKeyframe);
                    Quaternion<double> q1; Euler2Quaternion(prevPosture->bone_rotation[bone].p, q1);
                    Quaternion<double> q2; Euler2Quaternion(startPosture->bone_rotation[bone].p, q2);
                    Quaternion<double> q3; Euler2Quaternion(endPosture->bone_rotation[bone].p, q3);
                    Quaternion<double> q4; Euler2Quaternion(nextPosture->bone_rotation[bone].p, q4);
                    //ahat = Slerp(Double(qn - 1, qn), qn + 1, 0.5)
                    //an = Slerp(qn, ahat, 1.0 / 3)
                    boneHat = Slerp(0.5, Double(q1, q2), q3);
                    anBone = Slerp((1.0) / 3, q2, boneHat);
                    //ahat = Slerp(Double(qn, qn+1), qn + 2, 0.5)
                    //bn = Slerp(qn+1, ahat, -1.0 / 3)
                    boneHat = Slerp(0.5, Double(q2, q3), q4);
                    bnBone = Slerp((-1.0) / 3, q3, boneHat);
                }
                // we need to first convert the individual bones' rotation of startPosture and endPosture from euler to quaternion
                Quaternion<double> startBone; Euler2Quaternion(startPosture->bone_rotation[bone].p, startBone);
                Quaternion<double> endBone; Euler2Quaternion(endPosture->bone_rotation[bone].p, endBone);                                            
                // evaluate Bezier Curve for quaternion with deCasteljau
                Quaternion<double> interBone = DeCasteljauQuaternion(t, startBone, anBone, bnBone, endBone);
                Quaternion2Euler(interBone, interpolatedPosture.bone_rotation[bone].p);
            }
            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }
        startKeyframe = endKeyframe;
    }
    counter.StopCounter();
    std::cout << "Computation time: " << counter.GetElapsedTime() << std::endl;
    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q) 
{
    double R[9];
    //we first convert euler angles to rotation matrix
    Euler2Rotation(angles, R);
    //we then convert rotation matrix to quaternion
    q = q.Matrix2Quaternion(R);
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3]) 
{
    double R[9];
    //we first convert quaternion to rotation matrix
    q.Quaternion2Matrix(R);
    //we then convert rotation matrix to euler angles
    Rotation2Euler(R, angles);
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
    // qStart and qEnd should be unit length
    qStart.Normalize();
    qEnd_.Normalize();
    Quaternion<double> result;
    // cos(theta) = qStart dotProduct qEnd
    double dotProd = qStart.Gets() * qEnd_.Gets() + qStart.Getx() * qEnd_.Getx() + qStart.Gety() * qEnd_.Gety() + qStart.Getz() * qEnd_.Getz();
    if (dotProd < 0) {
        // when cos(theta) < 0, qS and qE correspond to the same rotation,
        // we need to flip either qS or qE by multiplying with -1.
        qStart = qStart * (-1);
        dotProd = dotProd * (-1);
    }
    if (dotProd > 0.9995) {
        // If the inputs are too close for comfort, linearly interpolate
        // and normalize the result.
        // ref:http://number-none.com/product/Understanding%20Slerp,%20Then%20Not%20Using%20It/
        result = qStart + (qEnd_ - qStart) * t;
        result.Normalize();
        return result;
    }
    double theta = acos(dotProd);    
    result = qStart * sin((1 - t) * theta) / sin(theta) + qEnd_ * sin(t * theta) / sin(theta);
    return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
    // Double(p, q) = 2(p dotProduct q)q - p
    Quaternion<double> result;
    double dotProd = p.Gets() * q.Gets() + p.Getx() * q.Getx() + p.Gety() * q.Gety() + p.Getz() * q.Getz();
    result = 2 * dotProd * q - p;
    return result;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
    vector result, Q0, Q1, Q2, R0, R1;
    Q0 = p0 + (p1 - p0) * t;
    Q1 = p1 + (p2 - p1) * t;
    Q2 = p2 + (p3 - p2) * t;
    R0 = Q0 + (Q1 - Q0) * t;
    R1 = Q1 + (Q2 - Q1) * t;
    result = R0 + (R1 - R0) * t;
    return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
    Quaternion<double> result, Q0, Q1, Q2, R0, R1;
    Q0 = Slerp(t, p0, p1);
    Q1 = Slerp(t, p1, p2);
    Q2 = Slerp(t, p2, p3);
    R0 = Slerp(t, Q0, Q1);
    R1 = Slerp(t, Q1, Q2);
    result = Slerp(t, R0, R1);
    return result;
}

