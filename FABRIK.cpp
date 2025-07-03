#include<iostream>
#include "raylib.h"




Vector3 VectorAbs(Vector3 vec){
	return {std::abs(vec.x),std::abs(vec.y),std::abs(vec.z)};
}

std::vector<Vector3> IKsolver(std::vector<Vector3> joints, Vector3 Target){
  std::vector<Vector3> FinalPositions(joints.size()-1);
  std::vector<float> JointDists(joints.size());
  int indexOfLastJoint = joints.size()-1;
  //manipulator max distance
  float IKmaxDist = 0.0;
  for(int i = 0 ; i < indexOfLastJoint; i++)
  {
    JointDists[i] = Vector3Length(joints[i+1] - joints[i]);
//	 IKmaxDist += Vector3Length(joints[i]);
//	 if(i == 1){IKmaxDist += Vector3Length(joints[i-1]);}
  }
  for(auto& i : JointDists){IKmaxDist+=i;}
 float TotalDist = Vector3Length(joints[0]-Target);

 //Distance is longer than the manipulator can reach
 if(TotalDist >= IKmaxDist)
 {
	  for(int i = 0 ; i < indexOfLastJoint; i++)
	  {
		  float targetDist =  Vector3Length(Target - joints[i]);
		  float Ratio = JointDists[i]/targetDist;
		  joints[i+1] = Vector3Scale(joints[i],(1.0 - Ratio)) + Vector3Scale(Target,Ratio);
	  }
 }
 else
 {
 //Otherwise solution is possible
 Vector3 Start = joints[0];
 float endEffectorDist = std::abs(Vector3Length(joints[indexOfLastJoint]) - Vector3Length(Target));
 float Tolerance = .1;
 while(endEffectorDist > Tolerance)
 {
 	joints[indexOfLastJoint] = Target;
	for(int i = indexOfLastJoint-1 ; i >=0 ; i--)
	{
		float newJointDist = Vector3Length(joints[i+1] - joints[i]);
		float Ratio = JointDists[i]/newJointDist;
		joints[i] = Vector3Scale(joints[i+1],(1.0 - Ratio)) + Vector3Scale(joints[i],Ratio);

	}

	//Backward Reaching Stage
	joints[0] = Start;
   for(int i = 0 ; i < indexOfLastJoint; i++)
	{
		float newJointDist = Vector3Length(joints[i+1] - joints[i]);
		float Ratio = JointDists[i]/newJointDist;
		joints[i+1] = Vector3Scale(joints[i],(1.0 - Ratio)) + Vector3Scale(joints[i+1],Ratio);
	}

  endEffectorDist = std::abs(Vector3Length(joints[joints.size()-1]) - Vector3Length(Target));
 }



 }
 return joints;
}
void FKsolver(){


}
