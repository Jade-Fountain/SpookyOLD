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
#include "Fusion/Utilities/TimeProfiling.h"
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


UFUNCTION(BlueprintCallable, Category = "Fusion") void UFusionPlant::Configure(float input_units_m, float output_units_m)
{
	plant.config.units.input_m = input_units_m;
	plant.config.units.output_m = output_units_m;

	plant.config.correlator.ambiguous_threshold = correlator_ambiguous_threshold;
	plant.config.correlator.elimination_threshold = correlator_elimination_threshold;
	plant.config.correlator.diff_threshold = correlator_diff_threshold;

	plant.config.calibrator.diff_threshold = calibration_diff_threshold;
	plant.config.calibrator.min_count_per_node = calibration_min_count_per_node;
	plant.config.calibrator.count_threshold = 
		{	
			{fusion::CalibrationResult::State::UNCALIBRATED,100},
			{fusion::CalibrationResult::State::REFINING,100 },
			{fusion::CalibrationResult::State::CALIBRATED,100}
		};
	plant.config.calibrator.initial_quality_threshold = calibration_initial_quality_threshold;
	plant.config.calibrator.quality_convergence_threshold = calibration_quality_convergence_threshold;
	plant.config.calibrator.fault_hysteresis_rate = calibration_fault_hysteresis_rate;
	plant.config.calibrator.relevance_decay_rate = calibration_relevance_decay_rate;
	plant.config.calibrator.settle_threshold = calibration_settle_threshold;
	plant.config.calibrator.fault_angle_threshold = calibration_fault_angle_threshold;
	plant.config.calibrator.fault_distance_threshold = calibration_fault_distance_threshold;
}

UFUNCTION(BlueprintCallable, Category = "Fusion") void UFusionPlant::AddSkeleton(UPoseableMeshComponent* poseable_mesh, FVector position_var, FVector4 quaternion_var)
{
	//Add skeleton reference
	skeletons.push_back(poseable_mesh);

	//Store uncertainties for later
	Eigen::Vector3f vv(&position_var[0]);
	Eigen::Vector4f vq(&quaternion_var[0]);
	Eigen::Matrix<float, 7, 1> uncertainty;
	uncertainty << vv, vq;
	skeletonCovariances.push_back(uncertainty);
	return;
}


UFUNCTION(BlueprintCallable, Category = "Fusion")
void UFusionPlant::SetOutputTarget(UPoseableMeshComponent * poseable_mesh)
{
	fusedSkeleton = poseable_mesh;
	TArray<FMeshBoneInfo> boneInfo = fusedSkeleton->SkeletalMesh->RefSkeleton.GetRefBoneInfo();
	for (int i = 0; i < boneInfo.Num(); i++) {
		FMeshBoneInfo& bone = boneInfo[i];
		//TODO: make more efficient
		FTransform b = FTransform(fusedSkeleton->SkeletalMesh->GetRefPoseMatrix(i));
		b.SetTranslation(b.GetTranslation() * plant.config.units.input_m);
		fusion::Transform3D bonePoseLocal = convert(b.ToMatrixNoScale());
		fusion::NodeDescriptor parent_desc = (bone.ParentIndex >= 0) ?
			fusion::NodeDescriptor(TCHAR_TO_UTF8(*(boneInfo[bone.ParentIndex].Name.GetPlainNameString()))) :
			fusion::NodeDescriptor();
		fusion::NodeDescriptor bone_desc = fusion::NodeDescriptor(TCHAR_TO_UTF8(*(bone.Name.GetPlainNameString())));
		//TODO: find better way to do this check for pose nodes
		if (bone.Name.GetPlainNameString() == "pelvis") {
			plant.addPoseNode(bone_desc, parent_desc, bonePoseLocal);
		}
		else {
			plant.addBoneNode(bone_desc, parent_desc, bonePoseLocal);
		}
		FUSION_LOG("Adding Bone: " + bone_desc.name + ", parent = " + parent_desc.name);
		
	}
}

UFUNCTION(BlueprintCallable, Category = "Fusion")
void UFusionPlant::FinaliseSetup() {
	plant.finaliseSetup();
}

//Set the reference frame for the skeleton
UFUNCTION(BlueprintCallable, Category = "Fusion")
void UFusionPlant::SetReferenceFrame(FString system_name) {
	plant.setReferenceSystem(fusion::SystemDescriptor(TCHAR_TO_UTF8(*system_name)));
}

//===========================
//Update functions
//===========================


UFUNCTION(BlueprintCallable, Category = "Fusion")
void UFusionPlant::AddPositionMeasurement(TArray<FString> nodeNames, FString systemName, int sensorID, float timestamp_sec, FVector measurement, FVector covariance, bool globalSpace, float confidence)
{
	Measurement::Ptr m = CreatePositionMeasurement(systemName, sensorID, timestamp_sec, measurement, covariance, confidence);
	m->globalSpace = globalSpace;
	plant.addMeasurement(m, convertToNodeDescriptors(nodeNames));
}

UFUNCTION(BlueprintCallable, Category = "Fusion")
void UFusionPlant::AddRotationMeasurement(TArray<FString> nodeNames, FString systemName, int sensorID, float timestamp_sec, FRotator measurement, FVector4 covariance, bool globalSpace, float confidence)
{
	Measurement::Ptr m = CreateRotationMeasurement(systemName,sensorID,timestamp_sec, measurement.Quaternion(),covariance,confidence);
	m->globalSpace = globalSpace;
	plant.addMeasurement(m, convertToNodeDescriptors(nodeNames));
}

UFUNCTION(BlueprintCallable, Category = "Fusion")
void UFusionPlant::AddPoseMeasurement(TArray<FString> nodeNames, FString systemName, int sensorID, float timestamp_sec, FTransform measurement, FVector position_var, FVector4 quaternion_var, bool globalSpace, float confidence)
{
	Eigen::Vector3f vv(&position_var[0]);
	Eigen::Vector4f vq(&quaternion_var[0]);
	Eigen::Matrix<float, 7, 1> uncertainty;
	uncertainty << vv, vq;
	Measurement::Ptr m = CreatePoseMeasurement(systemName, sensorID, timestamp_sec, measurement.GetTranslation(), measurement.GetRotation(), uncertainty, confidence);
	m->globalSpace = globalSpace;
	plant.addMeasurement(m, convertToNodeDescriptors(nodeNames));
}

UFUNCTION(BlueprintCallable, Category = "Fusion")
void UFusionPlant::addSkeletonMeasurement(int skel_index, float timestamp_sec) {
	//For each bone
	auto& skeleton = skeletons[skel_index];
	TArray<FMeshBoneInfo> boneInfo = skeleton->SkeletalMesh->RefSkeleton.GetRefBoneInfo();
	for (int i = 0; i < boneInfo.Num(); i++) {
		FMeshBoneInfo& bone = boneInfo[i];
		fusion::NodeDescriptor bone_name = fusion::NodeDescriptor(TCHAR_TO_UTF8(*(bone.Name.GetPlainNameString())));
		FTransform measurement = skeleton->BoneSpaceTransforms[i];
		//TODO: support confidences
		//TODO: doesnt seem like the best way to do this!
		//TODO: support skeleton group measurement input properly: need skeleton->getUncertianty(i), get confidence, time stamp, etc
		Measurement::Ptr m = CreatePoseMeasurement(skeleton->GetName(), i, timestamp_sec, measurement.GetTranslation(), measurement.GetRotation(), skeletonCovariances[skel_index], 1);
		m->globalSpace = false;
		plant.addMeasurement(m, bone_name);
	}
}
UFUNCTION(BlueprintCallable, Category = "Fusion")
void UFusionPlant::Fuse(float timestamp_sec)
{
	fusion::utility::profiler.startTimer("AAA FUSION TIME");
	for (int i = 0; i < skeletons.size(); i++) {
		addSkeletonMeasurement(i, timestamp_sec);
	}
	plant.fuse();
	fusion::utility::profiler.endTimer("AAA FUSION TIME");
	//FUSION_LOG(fusion::utility::profiler.getReport());
}

UFUNCTION(BlueprintCallable, Category = "Fusion")
void UFusionPlant::UpdateSkeletonOutput() {
	//For each bone
	TArray<FMeshBoneInfo> boneInfo = fusedSkeleton->SkeletalMesh->RefSkeleton.GetRefBoneInfo();
	//FUSION_LOG("\n\n\n\n Skeleton Poses = \n\n\n\n");
	for (int i = 0; i < boneInfo.Num(); i++) {
		FMeshBoneInfo& bone = boneInfo[i];
		fusion::NodeDescriptor bone_name = fusion::NodeDescriptor(TCHAR_TO_UTF8(*(bone.Name.GetPlainNameString())));

		fusion::Transform3D T = plant.getNodeLocalPose(bone_name);
		fusedSkeleton->BoneSpaceTransforms[i] = FTransform(convert(T));
		fusedSkeleton->BoneSpaceTransforms[i].SetTranslation(fusedSkeleton->BoneSpaceTransforms[i].GetTranslation() / plant.config.units.output_m);
		//UE_LOG(LogTemp, Warning, TEXT("skeleton new pose : %s"), *(bone.Name.GetPlainNameString()));
		//UE_LOG(LogTemp, Warning, TEXT("skeleton new pose : %s"), *(fusedSkeleton->BoneSpaceTransforms[i].ToMatrixNoScale().ToString()));

	}
}


//===========================
//Data retrieval functions
//===========================

FCalibrationResult UFusionPlant::getCalibrationResult(FString s1, FString s2)
{
	fusion::CalibrationResult T = plant.getCalibrationResult(fusion::SystemDescriptor(TCHAR_TO_UTF8(*s1)),fusion::SystemDescriptor(TCHAR_TO_UTF8(*s2)));
	Eigen::Quaternionf q(T.transform.matrix().block<3,3>(0,0));
	Eigen::Vector3f v(T.transform.matrix().block<3, 1>(0, 3) / plant.config.units.output_m);
	FQuat fq(q.x(), q.y(), q.z(), q.w());
	
	FCalibrationResult result;
	result.transform.SetRotation(fq);
	result.transform.SetTranslation(FVector(v[0], v[1], v[2]));
	result.calibrated = T.calibrated();
	result.refining = T.refining();
	result.quality = T.quality;
	result.system1 = FString(T.systems.first.name.c_str());
	result.system2 = FString(T.systems.second.name.c_str());
	return result;
}

FString UFusionPlant::getCorrelationResult(FString s1, int sensorID)
{
	return plant.getCorrelationResult(fusion::SystemDescriptor(TCHAR_TO_UTF8(*s1)),sensorID).name.c_str();
}

FTransform UFusionPlant::getNodeGlobalPose(FString node)
{
	fusion::Transform3D result = plant.getNodeGlobalPose(fusion::NodeDescriptor(TCHAR_TO_UTF8(*node)));
	FMatrix unrealMatrix = convert(result);
	unrealMatrix.ScaleTranslation(FVector(1,1,1) * 1 / plant.config.units.output_m);
	//UE_LOG(LogTemp, Warning, TEXT("getNodePose : %s"), *(unrealMatrix.ToString()));
	return FTransform(unrealMatrix);
}
//===========================
//Data saving/loading functions
//===========================

//Sets save/load location	
UFUNCTION(BlueprintCallable, Category = "Fusion")
void UFusionPlant::setSaveDirectory(FString dir) {
	plant.setSaveDirectory(TCHAR_TO_UTF8(*dir));
}

//Saves the calibration result mapping T:s1->s2
UFUNCTION(BlueprintCallable, Category = "Fusion")
void UFusionPlant::saveCalibrationResult(FString s1, FString s2){
	plant.saveCalibration(fusion::SystemDescriptor(TCHAR_TO_UTF8(*s1)),fusion::SystemDescriptor(TCHAR_TO_UTF8(*s2)));
}

//Loads the calibration result mapping T:s1->s2
UFUNCTION(BlueprintCallable, Category = "Fusion")
void UFusionPlant::loadCalibrationResult(FString s1, FString s2){
	plant.loadCalibration(fusion::SystemDescriptor(TCHAR_TO_UTF8(*s1)), fusion::SystemDescriptor(TCHAR_TO_UTF8(*s2)));
}

//===========================
//Utility
//===========================

//Compute axis angle representation (x,y,z,alpha)
UFUNCTION(BlueprintCallable, Category = "Fusion")
FVector4 UFusionPlant::getRotatorAxisAngle(FRotator R) {
	float angle;
	FVector axis;
	R.Quaternion().ToAxisAndAngle(axis,angle);
	return FVector4(axis[0], axis[1], axis[2], angle * 180 / M_PI);
}

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
//TODO: optimise with const ref
Measurement::Ptr UFusionPlant::CreatePositionMeasurement(FString system_name, int sensorID, float timestamp_sec, FVector position, FVector uncertainty, float confidence)
{
	//Create basic measurement
	Eigen::Vector3f meas(position[0],position[1],position[2]);
	meas = meas * plant.config.units.input_m;

	Eigen::Matrix<float, 3, 3> un = Eigen::Matrix<float,3,3>::Identity();
	un.diagonal() = Eigen::Vector3f(uncertainty[0], uncertainty[1], uncertainty[2]);
	Measurement::Ptr result = Measurement::createCartesianMeasurement(meas, un);
	
	//Add metadata
	SetCommonMeasurementData(result, system_name, sensorID, timestamp_sec, confidence);

	return std::move(result);
}

Measurement::Ptr UFusionPlant::CreateRotationMeasurement(FString system_name, int sensorID, float timestamp_sec, FQuat rotation, FVector4 uncertainty, float confidence)
{
	//Create basic measurement
	//BEWARE: dumb format mismatch:
	Eigen::Quaternionf meas(rotation.W, rotation.X, rotation.Y, rotation.Z);
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
	meas = meas * plant.config.units.input_m;
	Eigen::Matrix<float, 3, 3> un = Eigen::Matrix<float, 3, 3>::Identity();
	un.diagonal() = Eigen::Vector3f(&uncertainty[0]);
	Measurement::Ptr result = Measurement::createScaleMeasurement(meas, un);

	//Add metadata
	SetCommonMeasurementData(result, system_name, sensorID, timestamp_sec, confidence);
	
	return std::move(result);
}

Measurement::Ptr UFusionPlant::CreatePoseMeasurement(FString system_name, int sensorID, float timestamp_sec, FVector v, FQuat q, Eigen::Matrix<float, 7, 1> uncertainty, float confidence)
{
	//Convert transform to state vector (v,q)
	Eigen::Vector3f ev(&v[0]);
	ev = ev * plant.config.units.input_m;
	//BEWARE: dumb format mismatch:
	Eigen::Quaternionf eq(q.W,q.X,q.Y,q.Z);
	//Create basic measurement
	Eigen::Matrix<float, 7, 7> un = Eigen::Matrix<float, 7, 7>::Identity();
	un.diagonal() = uncertainty;
	Measurement::Ptr result = Measurement::createPoseMeasurement(ev, eq, un);
	
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

FMatrix UFusionPlant::convert(const fusion::Transform3D& T) {
	FMatrix unrealMatrix;
	memcpy(&(unrealMatrix.M[0][0]), T.data(), sizeof(float) * 16);
	return unrealMatrix;
}

fusion::Transform3D UFusionPlant::convert(const FMatrix& T) {
	fusion::Transform3D matrix;
	memcpy(matrix.data(), &(T.M[0][0]), sizeof(float) * 16);
	return matrix;
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

//For testing blueprints: TODO delete
UFUNCTION(BlueprintCallable, Category = "Fusion")
FString UFusionPlant::GetCalibrationStateSummary() {
	std::string s = plant.getCalibratorStateSummary();
	return s.c_str();
}
//For testing blueprints: TODO delete
UFUNCTION(BlueprintCallable, Category = "Fusion")
FString UFusionPlant::GetCalibrationTimingSummary() {
	std::string s = plant.getTimingSummary();
	return s.c_str();
}
