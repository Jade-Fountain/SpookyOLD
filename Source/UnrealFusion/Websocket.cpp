// Test

#include "UnrealFusion.h"
#include "Logging.h"
#include "Websocket.h"
#include <assert.h>
#include <string>

#include "AllowWindowsPlatformTypes.h"
#pragma comment( lib, "ws2_32" )
#include <WinSock2.h>
#include "HideWindowsPlatformTypes.h"

// Sets default values for this component's properties
UWebsocket::UWebsocket()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// ...
}

void UWebsocket::handle_message(const std::string& message){
	FUSION_LOG(message);
}
// Called when the game starts
void UWebsocket::BeginPlay()
{
	Super::BeginPlay();
	
	int rc;
	WSADATA wsaData;

	rc = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (rc) {
		FUSION_LOG("WSAStartup Failed.");
		return;
	}

	ws = std::unique_ptr<easywsclient::WebSocket>(easywsclient::WebSocket::from_url_no_mask("ws://127.0.0.1:82"));

	if (ws) {
		FUSION_LOG("Connection successful " + std::to_string(ws->getReadyState()));
	}
	else {
		FUSION_LOG("Connection unsuccessful ");
	}


/*
		ws->close();
		WSACleanup();*/

	
}


// Called every frame
void UWebsocket::TickComponent( float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction )
{
	Super::TickComponent( DeltaTime, TickType, ThisTickFunction );
	
	// ...
	if (ws) {
		//FUSION_LOG("Polling websocket");
		ws->send("r");
		ws->poll(1000);
		ws->dispatch([this](const std::string & message) {
			latestString = message;
		});
	}
}

UFUNCTION(BlueprintCallable, Category = "Websocket")
FString UWebsocket::getLatestString()
{
	return FString(latestString.c_str());
}

