// Created by Chance_07 2014
#pragma once

#include "UnrealFusion.h"
#include "Classes/Animation/AnimNodeBase.h"
#include "AnimNode_TranslateWith.generated.h"


USTRUCT()
struct FAnimNode_TranslateWith : public FAnimNode_Base
{
	GENERATED_USTRUCT_BODY()

		/** Input link(Base Pose) **/
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Links)
		FComponentSpacePoseLink ComponentPose;

	/** Name of bone to control **/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = SkeletalControl)
		FBoneReference TargetBone;

	/** Source Bone Name to get transform from **/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = SkeletalControl)
		FBoneReference SourceBone;

	/** This is typically the FORWARD Axis, and is false by default **/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = AxisLock, meta = (PinHiddenByDefault))
		bool bUpdateX;

	/** This is typically the SIDE-TO-SIDE Axis, and is false by default **/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = AxisLock, meta = (PinHiddenByDefault))
		bool bUpdateY;

	/** This is typically the UP Axis, and is true by default **/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = AxisLock, meta = (PinHiddenByDefault))
		bool bUpdateZ;

	/** Target Location in world space if LookAtBone is empty */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = SkeletalControl)
		FVector AddtoOffset;

	/** Controls how much of the translation is blended to the target, default is 1 **/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = SkeletalControl, meta = (PinShownByDefault))
		float BlendWeight;
public:
	FAnimNode_TranslateWith();
public:
	// FAnimNode_Base interface
	virtual void Initialize(const FAnimationInitializeContext& Context) override;
	virtual void CacheBones(const FAnimationCacheBonesContext & Context)  override;
	virtual void Update(const FAnimationUpdateContext& Context) override;
	virtual void EvaluateComponentSpace(FComponentSpacePoseContext& Output) override;
	// End of FAnimNode_RCAnimNode interface
protected:
	// initialize any bone references you have
	virtual void InitializeBoneReferences(const FBoneContainer & RequiredBones);
	// return true if it is valid to Evaluate
	virtual bool IsValidToEvaluate(const USkeleton * Skeleton, const FBoneContainer & RequiredBones);
	// Evaluate the new component-space transforms for the affected bones.
	virtual void EvaluateBoneTransforms(USkeletalMeshComponent* SkelComp, const FBoneContainer & RequiredBones, FA2CSPose& MeshBases, TArray<FBoneTransform>& OutBoneTransforms);// {}
private:
	FVector ModTargetLocationInCompSpace;
	FVector CurrentAddedOffset;
	FVector JointOffset;
};