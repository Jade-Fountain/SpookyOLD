// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Components/ActorComponent.h"
#include "Components/PoseableMeshComponent.h"

#include "Fusion/FusionTypes.h"
#include "Fusion/SkeletonModel.h"

#include <iostream>
#include <vector>
#include <string>
#include "Fusion/FusionPlant.h"

//Must be last include
#include "UFusionPlant.generated.h"


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class UNREALFUSION_API UFusionPlant : public UActorComponent
{
	GENERATED_BODY()

	//Fusionplant
	FusionPlant plant;

	//Input Skeletons
	std::vector<UPoseableMeshComponent*> skeletons;
protected:
	//Fusion result
	UPROPERTY(BlueprintReadOnly)
	UPoseableMeshComponent* fusedSkeleton;

public:	
	// Sets default values for this component's properties
	UFusionPlant();

	// Called when the game starts
	virtual void BeginPlay() override;
	
	// Called every frame
	virtual void TickComponent( float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction ) override;

//===========================
//Setup and initialisation
//===========================

	//Add complete skeleton to list of fusion objects
	UFUNCTION(BlueprintCallable, Category = "Fusion")
	void AddSkeleton(UPoseableMeshComponent* poseable_mesh);
	
	//Set the output target which will have the complete fused skeleton pose applied
	UFUNCTION(BlueprintCallable, Category = "Fusion")
	void SetOutputTarget(UPoseableMeshComponent* poseable_mesh);

//Contruction of sensor nodes

	////Add a new sensor node model
	//UFUNCTION(BlueprintCallable, Category = "Fusion")
	//void AddSensorNode(FString nodeName, FTransform initialState, FTransform initialCovariance);

	////Add a new sensor node model
	//UFUNCTION(BlueprintCallable, Category = "Fusion")
	//void SetHomeCoordinateSpace(FString systemName);


//===========================
//Update functions
//===========================
	UFUNCTION(BlueprintCallable, Category = "Fusion")
	void AddPositionMeasurement(FString nodeName, FString systemName, int sensorID, float timestamp_sec, FVector measurement, FVector covariance, float confidence = 1);
	
	UFUNCTION(BlueprintCallable, Category = "Fusion")
	void AddRotationMeasurement(FString nodeName, FString systemName, int sensorID, float timestamp_sec, FQuat measurement, FVector covariance, float confidence = 1);

	UFUNCTION(BlueprintCallable, Category = "Fusion")
	void Fuse();

//===========================
//Utility
//===========================

	void CopyPose(UPoseableMeshComponent* target, const UPoseableMeshComponent* input);


	Measurement::Ptr CreatePositionMeasurement(FString system_name, int sensorID, float timestamp_sec, FVector position, FVector uncertainty, float confidence = 1);
	Measurement::Ptr CreateRotationMeasurement(FString system_name, int sensorID, float timestamp_sec, FQuat rotation, FVector uncertainty, float confidence = 1);
	Measurement::Ptr CreateScaleMeasurement(FString system_name, int sensorID, float timestamp_sec, FVector scale, FVector uncertainty, float confidence = 1);
	Measurement::Ptr CreateRigidBodyMeasurement(FString system_name, int sensorID, float timestamp_sec, FVector state, FVector uncertainty, float confidence = 1);


//===========================
//DEBUG
//===========================

	//For testing blueprints: TODO delete
	UFUNCTION(BlueprintCallable, Category = "Fusion")
	FVector4 GetTestPosition();

	
};
