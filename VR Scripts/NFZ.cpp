// Fill out your copyright notice in the Description page of Project Settings.


#include "NFZ.h"
#include <vector>

extern bool isFTTTriggered;
bool isNFZTriggered;
extern std::vector<float> vecX;
extern std::vector<float> vecY;
extern std::vector<float> vecZ;

// Sets default values
ANFZ::ANFZ()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void ANFZ::BeginPlay()
{
	Super::BeginPlay();
	
}

void ANFZ::DisableDrone(int drone_number)
{
	DisableFTTTopic = NewObject<UTopic>(UTopic::StaticClass());

	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());

	FString drone_number_str = FString::FromInt(drone_number);
	FString FTTTopicName = "/uav" + drone_number_str + "/ftt_enable";
	GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, FString::Printf(TEXT("%s"), *FTTTopicName));

	DisableFTTTopic->Init(rosinst->ROSIntegrationCore, FTTTopicName, TEXT("std_msgs/String"));

	DisableFTTTopic->Advertise();

	TSharedPtr<ROSMessages::std_msgs::String> DisableMessage(new ROSMessages::std_msgs::String("FTT Disabled"));

	isFTTTriggered = false;
	isNFZTriggered = true;

	DisableFTTTopic->Publish(DisableMessage);
	
	UE_LOG(LogTemp, Warning, TEXT("Disabling FTT"));
	GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, FString::Printf(TEXT("Disabling FTT")));
	vecX.clear();
	vecY.clear();
	vecZ.clear();

}

// Called every frame
//void ANFZ::Tick(float DeltaTime)
//{
//	Super::Tick(DeltaTime);
//
//}

