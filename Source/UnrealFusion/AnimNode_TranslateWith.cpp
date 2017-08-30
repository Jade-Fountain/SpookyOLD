// Created by Chance_07 2014
#include "UnrealFusion.h"
#include "AnimNode_TranslateWith.h"

FAnimNode_TranslateWith::FAnimNode_TranslateWith()
	: FAnimNode_Base()
	, bUpdateX(false)// This is only to set
	, bUpdateY(false)// the defaults for the
	, bUpdateZ(true)// Axis pins (Can be changed in BP)
	, AddtoOffset(FVector::ZeroVector)
{
	BlendWeight = 1;
	JointOffset.ZeroVector;
	// NULL so that it will be tested at least once
	CurrentAddedOffset.X = NULL;
	CurrentAddedOffset.Y = NULL;
	CurrentAddedOffset.Z = NULL;
}

void FAnimNode_TranslateWith::Initialize(const FAnimationInitializeContext& Context)
{
	ComponentPose.Initialize(Context);
}

void FAnimNode_TranslateWith::Update(const FAnimationUpdateContext& Context)
{
	ComponentPose.Update(Context);
}

void FAnimNode_TranslateWith::CacheBones(const FAnimationCacheBonesContext & Context)
{
	InitializeBoneReferences(Context.AnimInstance->RequiredBones);
	ComponentPose.CacheBones(Context);
}

//This is just a check to make sure that the bones are present
void FAnimNode_TranslateWith::InitializeBoneReferences(const FBoneContainer & RequiredBones)
{
	// Check the Source Bone
	if (SourceBone.Initialize(RequiredBones)) {}
	else
	{
		UE_LOG(LogAnimation, Warning, TEXT("No Source Bone found, please check input"));
	}

	// Check the Target Bone
	if (TargetBone.Initialize(RequiredBones)) {}
	else
	{
		UE_LOG(LogAnimation, Warning, TEXT("No Target Bone found, please check input"));
	}
}

//This Checks for Updates, if no Axis is checked, returns Base Pose and moves on
void FAnimNode_TranslateWith::EvaluateComponentSpace(FComponentSpacePoseContext& Output)
{
	// Evaluate the input
	ComponentPose.EvaluateComponentSpace(Output);


	// If any Axis is checked, and both bones are present, then we will continue
	if (((bUpdateX) || (bUpdateY) || (bUpdateZ)) && IsValidToEvaluate(Output.AnimInstance->CurrentSkeleton, Output.AnimInstance->RequiredBones))
	{
		USkeletalMeshComponent* Component = Output.AnimInstance->GetSkelMeshComponent();
		TArray<FBoneTransform> BoneTransforms;
		EvaluateBoneTransforms(Component, Output.AnimInstance->RequiredBones, Output.Pose, BoneTransforms);

		checkSlow(!ContainsNaN(BoneTransforms));

		if (BoneTransforms.Num() > 0)
		{
			Output.Pose.LocalBlendCSBoneTransforms(BoneTransforms, BlendWeight);
		}
	}
}

//This Evaluates the bones, and transforms	
void FAnimNode_TranslateWith::EvaluateBoneTransforms(USkeletalMeshComponent* SkelComp, const FBoneContainer & RequiredBones, FA2CSPose& MeshBases, TArray<FBoneTransform>& OutBoneTransforms)
{
	check(OutBoneTransforms.Num() == 0);

	// Component Space Transforms used to modify Target Bone
	FTransform SourceBoneTM = MeshBases.GetComponentSpaceTransform(SourceBone.BoneIndex);
	FTransform TargetBoneTM = MeshBases.GetComponentSpaceTransform(TargetBone.BoneIndex);

	// If any changes have occured, add them, and reset offset
	if (!(CurrentAddedOffset == AddtoOffset))
	{
		CurrentAddedOffset = SkelComp->ComponentToWorld.InverseTransformPosition(AddtoOffset);
		JointOffset.Set((NULL), (NULL), (NULL));
	}

	if ((JointOffset.X == NULL) || (JointOffset.Y == NULL) || (JointOffset.Z == NULL))// If no Axis are checked, no need to do anything
	{
		// These are based on the Base Pose location and used to create the default offset
		FVector tempSourceOffset;
		FVector tempTargetOffset;

		// Get the Base Pose of the Source Bone
		tempSourceOffset.Set((SourceBoneTM.GetLocation().X), (SourceBoneTM.GetLocation().Y), (SourceBoneTM.GetLocation().Z));

		// Get the Base Pose of the Target Bone
		tempTargetOffset.Set((TargetBoneTM.GetLocation().X), (TargetBoneTM.GetLocation().Y), (TargetBoneTM.GetLocation().Z));

		// Get the Starting Offset from ALL inputs
		JointOffset.Set((tempTargetOffset.X - tempSourceOffset.X + CurrentAddedOffset.X), (tempTargetOffset.Y - tempSourceOffset.Y + CurrentAddedOffset.Y), (tempTargetOffset.Z - tempSourceOffset.Z + CurrentAddedOffset.Z));

		// Assign Offset to Mod Target Location
		ModTargetLocationInCompSpace.Set((tempTargetOffset.X), (tempTargetOffset.Y), (tempTargetOffset.Z));


	}
	//need to add a multiplier control so that you can control how much of the source movement is translated to the target(I.E. BlendEffects)
	if (bUpdateX)//Update X if checked
	{
		ModTargetLocationInCompSpace.X = ((SourceBoneTM.GetTranslation().X) + JointOffset.X);//SourceTM + Offset
	}
	if (bUpdateY)//Update Y if checked
	{
		ModTargetLocationInCompSpace.Y = ((SourceBoneTM.GetTranslation().Y) + JointOffset.Y);
	}
	if (bUpdateZ)//Update Z if checked
	{
		ModTargetLocationInCompSpace.Z = ((SourceBoneTM.GetTranslation().Z) + JointOffset.Z); //(SourceBoneTM.GetTranslation().Z);
	}
	//Set the Targets Transform and pass out to Output(End of tick)
	TargetBoneTM.SetTranslation(ModTargetLocationInCompSpace);

	// Returns to caller with new location
	OutBoneTransforms.Add(FBoneTransform(TargetBone.BoneIndex, TargetBoneTM));
}

// Makes sure that both bones are valid
bool FAnimNode_TranslateWith::IsValidToEvaluate(const USkeleton * Skeleton, const FBoneContainer & RequiredBones)
{
	// if both bones are valid
	return (TargetBone.IsValid(RequiredBones) && (SourceBone.IsValid(RequiredBones)));
}