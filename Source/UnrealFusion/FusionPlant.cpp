/*  This file is part of UnrealFusion, a sensor fusion plugin for VR in the Unreal Engine
    Copyright (C) 2017 Jake Fountain
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "UnrealFusion.h"
#include "FusionPlant.h"
#include <iostream>

using fusion::Measurement;
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
	TArray<FMeshBoneInfo> boneInfo = fusedSkeleton->SkeletalMesh->RefSkeleton.GetRefBoneInfo();
	fusedSkeleton->SkeletalMesh->GetRefPoseMatrix(0);
	for (auto& bone : boneInfo) {
		plant.addNode(fusion::NodeDescriptor(TCHAR_TO_UTF8(*(bone.Name.GetPlainNameString()))), 
					  fusion::NodeDescriptor(TCHAR_TO_UTF8(*(boneInfo[std::max(0,bone.ParentIndex)].Name.GetPlainNameString()))));
	}
}

UFUNCTION(BlueprintCallable, Category = "Fusion")
void UFusionPlant::FinaliseSetup() {
	plant.finaliseSetup();
}

//===========================
//Update functions
//===========================


UFUNCTION(BlueprintCallable, Category = "Fusion")
void UFusionPlant::AddPositionMeasurement(TArray<FString> nodeNames, FString systemName, int sensorID, float timestamp_sec, FVector measurement, FVector covariance, float confidence)
{
	Measurement::Ptr m = CreatePositionMeasurement(systemName, sensorID, timestamp_sec, measurement, covariance, confidence);
	plant.addMeasurement(m, convertToNodeDescriptors(nodeNames));
}

UFUNCTION(BlueprintCallable, Category = "Fusion")
void UFusionPlant::AddRotationMeasurement(TArray<FString> nodeNames, FString systemName, int sensorID, float timestamp_sec, FQuat measurement, FVector covariance, float confidence)
{
	Measurement::Ptr m = CreateRotationMeasurement(systemName,sensorID,timestamp_sec,measurement,covariance,confidence);
	plant.addMeasurement(m, convertToNodeDescriptors(nodeNames));
}

void UFusionPlant::Fuse()
{
	plant.fuse();
	//if (skeletons.size() > 0 && fusedSkeleton != NULL) {
	//	CopyPose(fusedSkeleton, skeletons[0]);
	//}
}
//===========================
//Data retrieval functions
//===========================

FCalibrationResult UFusionPlant::getCalibrationResult(FString s1, FString s2)
{
	fusion::CalibrationResult T = plant.getCalibrationResult(fusion::SystemDescriptor(TCHAR_TO_UTF8(*s1)),fusion::SystemDescriptor(TCHAR_TO_UTF8(*s2)));
	Eigen::Quaternionf q(T.transform.matrix().block<3,3>(0,0));
	Eigen::Vector3f v(T.transform.matrix().block<3, 1>(0, 3));
	FQuat fq(q.x(), q.y(), q.z(), q.w());
	
	FCalibrationResult result;
	result.transform.SetRotation(fq);
	result.transform.SetTranslation(FVector(v[0], v[1], v[2]));
	result.calibrated = T.calibrated();
	result.quality = T.quality;
	result.system1 = FString(T.systems.first.name.c_str());
	result.system2 = FString(T.systems.second.name.c_str());
	return result;
}

FString UFusionPlant::getCorrelationResult(FString s1, int sensorID)
{
	return plant.getCorrelationResult(fusion::SystemDescriptor(TCHAR_TO_UTF8(*s1)),sensorID).name.c_str();
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

Measurement::Ptr UFusionPlant::CreatePositionMeasurement(FString system_name, int sensorID, float timestamp_sec, FVector position, FVector uncertainty, float confidence)
{
	//Create basic measurement
	Eigen::Vector3f meas(position[0],position[1],position[2]);
	Eigen::Matrix<float, 3, 3> un = Eigen::Matrix<float,3,3>::Identity();
	un.diagonal() = Eigen::Vector3f(uncertainty[0], uncertainty[1], uncertainty[2]);
	Measurement::Ptr result = Measurement::createCartesianMeasurement(meas, un);
	
	//Add metadata
	SetCommonMeasurementData(result, system_name, sensorID, timestamp_sec, confidence);

	return std::move(result);
}

Measurement::Ptr UFusionPlant::CreateRotationMeasurement(FString system_name, int sensorID, float timestamp_sec, FQuat rotation, FVector uncertainty, float confidence)
{
	//Create basic measurement
	Eigen::Vector4f meas(rotation.W, rotation.X, rotation.Y, rotation.Z);
	Eigen::Matrix<float, 4, 4> un = Eigen::Matrix<float, 4, 4>::Identity();
	un.diagonal() = Eigen::Vector4f(&uncertainty[0]);
	Measurement::Ptr result = Measurement::createQuaternionMeasurement(meas, un);

	//Add metadata
	SetCommonMeasurementData(result, system_name, sensorID, timestamp_sec, confidence);
	
	return std::move(result);
}

Measurement::Ptr UFusionPlant::CreateScaleMeasurement(FString system_name, int sensorID, float timestamp_sec, FVector scale, FVector uncertainty, float confidence)
{
	//Create basic measurement
	Eigen::Vector3f meas(&scale[0]);
	Eigen::Matrix<float, 3, 3> un = Eigen::Matrix<float, 3, 3>::Identity();
	un.diagonal() = Eigen::Vector3f(&uncertainty[0]);
	Measurement::Ptr result = Measurement::createScaleMeasurement(meas, un);

	//Add metadata
	SetCommonMeasurementData(result, system_name, sensorID, timestamp_sec, confidence);
	
	return std::move(result);
}

Measurement::Ptr UFusionPlant::CreateRigidBodyMeasurement(FString system_name, int sensorID, float timestamp_sec, FVector state, FVector uncertainty, float confidence)
{
	//Create basic measurement
	Eigen::Matrix<float, 7, 1> meas(&state[0]);
	Eigen::Matrix<float, 7, 7> un = Eigen::Matrix<float, 7, 7>::Identity();
	un.diagonal() = Eigen::Matrix<float, 7, 1>(&uncertainty[0]);
	Measurement::Ptr result = Measurement::createRigidBodyMeasurement(meas, un);
	
	//Add metadata
	SetCommonMeasurementData(result, system_name, sensorID, timestamp_sec, confidence);

	return std::move(result);
}

void UFusionPlant::SetCommonMeasurementData(Measurement::Ptr& m, FString system_name, int sensorID, float timestamp_sec, float confidence){
	//Add metadata
	plant.setMeasurementSensorInfo(m, fusion::SystemDescriptor(TCHAR_TO_UTF8(*system_name)), fusion::SensorID(sensorID));
	bool measurementConsistent = m->setMetaData(timestamp_sec, confidence);
	if (!measurementConsistent) {
		std::cout << "WARNING - Measurement not created correctly - " << __LINE__ << std::endl;
	}
}

std::vector<fusion::NodeDescriptor> UFusionPlant::convertToNodeDescriptors(const TArray<FString>& names){
	std::vector<fusion::NodeDescriptor> result;
	for(auto& name : names){
		result.push_back(fusion::NodeDescriptor(TCHAR_TO_UTF8(*name)));
	}
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
