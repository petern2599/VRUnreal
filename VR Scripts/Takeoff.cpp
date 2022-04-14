// Fill out your copyright notice in the Description page of Project Settings.


#include "Takeoff.h"
#include "Components/Button.h"


bool isTakeoffTriggered = false;
extern FString DroneSelected;
FString TopicName16;

bool UTakeoff::Initialize()
{
	Super::Initialize();
	Takeoff->OnClicked.AddDynamic(this, &UTakeoff::StartButtonClicked);
	return true;
}


void UTakeoff::StartButtonClicked()
{
	//Check if button works
	UE_LOG(LogTemp, Warning, TEXT("Our button is working!"));

	// Initialize a topic
	TakeoffTopic = NewObject<UTopic>(UTopic::StaticClass());
	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());

	TopicName16 = DroneSelected + "/takeoff_enable";

	TakeoffTopic->Init(rosinst->ROSIntegrationCore, TopicName16, TEXT("std_msgs/String"));

	// (Optional) Advertise the topic
	TakeoffTopic->Advertise();

	// Publish a string to the topic
	TSharedPtr<ROSMessages::std_msgs::String> EnableMessage(new ROSMessages::std_msgs::String("Takeoff Enabled"));
	TSharedPtr<ROSMessages::std_msgs::String> DisableMessage(new ROSMessages::std_msgs::String("Takeoff Disabled"));

	//Publish message
	if (isTakeoffTriggered == false)
	{
		TakeoffTopic->Publish(EnableMessage);
		UE_LOG(LogTemp, Warning, TEXT("Enabling Takeoff"));
		isTakeoffTriggered = true;
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, FString::Printf(TEXT("Enabling Takeoff")));
	}
	else
	{
		TakeoffTopic->Publish(DisableMessage);
		UE_LOG(LogTemp, Warning, TEXT("Disabling Takeoff"));
		isTakeoffTriggered = false;
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, FString::Printf(TEXT("Disabling Takeoff")));
	}


}