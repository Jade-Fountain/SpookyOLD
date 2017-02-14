// Fill out your copyright notice in the Description page of Project Settings.

#include "UnrealFusion.h"
#include "FusionPlant.h"


//===========================
//Setup and initialisation
//===========================

// Sets default values for this component's properties
UFusionPlant::UFusionPlant()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;
	// ...
}


// Called when the game starts
void UFusionPlant::BeginPlay()
{
	Super::BeginPlay();
	// ...
	
}


// Called every frame
void UFusionPlant::TickComponent( float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction )
{
	Super::TickComponent( DeltaTime, TickType, ThisTickFunction );
	if (fusedSkeleton != NULL) {
		fusedSkeleton->RefreshBoneTransforms(ThisTickFunction);
	}
}

UFUNCTION(BlueprintCallable, Category = "Fusion") void UFusionPlant::AddSkeleton(UPoseableMeshComponent* poseable_mesh)
{
	skeletons.push_back(poseable_mesh);
	return;
}


UFUNCTION(BlueprintCallable, Category = "Fusion")
void UFusionPlant::SetOutputTarget(UPoseableMeshComponent * poseable_mesh)
{
	fusedSkeleton = poseable_mesh;
}


//===========================
//Update functions
//===========================


UFUNCTION(BlueprintCallable, Category = "Fusion")
void UFusionPlant::AddPositionMeasurement(FString nodeName, FString systemName, int sensorID, float timestamp_sec, FVector measurement, FVector covariance, float confidence)
{
}

UFUNCTION(BlueprintCallable, Category = "Fusion")
void UFusionPlant::AddRotationMeasurement(FString nodeName, FString systemName, int sensorID, float timestamp_sec, FQuat measurement, FVector covariance, float confidence)
{
}

void UFusionPlant::Fuse()
{
	if (skeletons.size() > 0 && fusedSkeleton != NULL) {
		CopyPose(fusedSkeleton, skeletons[0]);
	}
}

//===========================
//Utility
//===========================

void UFusionPlant::CopyPose(UPoseableMeshComponent* target, const UPoseableMeshComponent* input)
{
	if (target->RequiredBones.IsValid())
	{
		if (target->SkeletalMesh == input->SkeletalMesh)
		{
			check(target->BoneSpaceTransforms.Num() == input->BoneSpaceTransforms.Num());

			// Quick path, we know everything matches, just copy the local atoms
			target->BoneSpaceTransforms = input->BoneSpaceTransforms;
		}
		else
		{
			// The meshes don't match, search bone-by-bone (slow path)

			// first set the localatoms to ref pose from our current mesh
			target->BoneSpaceTransforms = target->SkeletalMesh->RefSkeleton.GetRefBonePose();

			// Now overwrite any matching bones
			const int32 NumSourceBones = input->SkeletalMesh->RefSkeleton.GetNum();

			for (int32 SourceBoneIndex = 0; SourceBoneIndex < NumSourceBones; ++SourceBoneIndex)
			{
				const FName SourceBoneName = input->GetBoneName(SourceBoneIndex);
				const int32 TargetBoneIndex = target->GetBoneIndex(SourceBoneName);

				if (TargetBoneIndex != INDEX_NONE)
				{
					target->BoneSpaceTransforms[TargetBoneIndex] = input->BoneSpaceTransforms[SourceBoneIndex];
				}
			}
		}
		target->RefreshBoneTransforms();
	}
}

PositionMeasurement UFusionPlant::CreatePositionMeasurement(FString system_name, int sensorID, float timestamp_sec, FVector position, FVector uncertainty, float confidence)
{
	Eigen::Vector3f meas(position[0],position[1],position[2]);
	Eigen::Matrix<float, PositionMeasurement::size, PositionMeasurement::size> un;
	un.diagonal() = Eigen::Vector3f(uncertainty[0], uncertainty[1], uncertainty[2]);
	
	PositionMeasurement result;
	result.systemName = TCHAR_TO_UTF8(*system_name);
	result.sensorID = sensorID;
	result.timeStamp = timestamp_sec;
	result.data = meas;
	result.uncertainty = un;
	result.confidence = confidence;
	
	return result;
}

RotationMeasurement UFusionPlant::CreateRotationMeasurement(FString system_name, int sensorID, float timestamp_sec, FQuat rotation, FVector uncertainty, float confidence)
{
	Eigen::Vector4f meas(rotation.W, rotation.X, rotation.Y, rotation.Z);
	Eigen::Matrix<float, RotationMeasurement::size, RotationMeasurement::size> un;
	un.diagonal() = Eigen::Vector4f(uncertainty[0], uncertainty[1], uncertainty[2],uncertainty[3]);

	RotationMeasurement result;
	result.systemName = TCHAR_TO_UTF8(*system_name);
	result.sensorID = sensorID;
	result.timeStamp = timestamp_sec;
	result.data = meas;
	result.uncertainty = un;
	result.confidence = confidence;
	
	return result;
}

//===========================
//DEBUG
//===========================


UFUNCTION(BlueprintCallable, Category = "Fusion")
FVector4 UFusionPlant::GetTestPosition() {
	FVector4 v = fusedSkeleton->GetBoneTransformByName("hand_l",EBoneSpaces::WorldSpace).GetLocation();
	//UE_LOG(LogTemp, Warning, TEXT("Left hand Pose = %s"), *v.ToString());
		

	
	return v;
}
