// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Components/ActorComponent.h"
#include "Components/PoseableMeshComponent.h"

#include <iostream>
#include <vector>
#include "FusionPlant.generated.h"


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class UNREALFUSION_API UFusionPlant : public UActorComponent
{
	GENERATED_BODY()


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

//===========================
//Update functions
//===========================

	UFUNCTION(BlueprintCallable, Category = "Fusion")
	void Fuse();

//===========================
//Utility
//===========================

	void CopyPose(UPoseableMeshComponent* target, const UPoseableMeshComponent* input);

//===========================
//DEBUG
//===========================

	//For testing blueprints: TODO delete
	UFUNCTION(BlueprintCallable, Category = "Fusion")
	FVector4 GetTestPosition();

	
};
