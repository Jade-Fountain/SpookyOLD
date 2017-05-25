// Test

#pragma once

#include "easywsclient.hpp"
#include <memory>


#include "Components/ActorComponent.h"
#include "Websocket.generated.h"


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class UNREALFUSION_API UWebsocket : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UWebsocket();

	void handle_message(const std::string& message);

	// Called when the game starts
	virtual void BeginPlay() override;
	
	// Called every frame
	virtual void TickComponent( float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction ) override;
	
	std::unique_ptr<easywsclient::WebSocket> ws;


	std::string latestString;
	////Gets latest marker data
	//UFUNCTION(BlueprintCallable, Category = "Fusion")
	//void initialiseWebsocket(FString url);

	//Gets latest websocket data
	UFUNCTION(BlueprintCallable, Category = "Websocket")
	FString getLatestString();

};
