// Fill out your copyright notice in the Description page of Project Settings.


#include "ManualControl.h"
#include "Components/Button.h"

bool isManualControlTriggered = false;
extern FString DroneSelected;
FString TopicName17;

bool UManualControl::Initialize()
{
	Super::Initialize();
	ManualControlButton->OnClicked.AddDynamic(this, &UManualControl::StartButtonClicked);
	return true;
}


void UManualControl::StartButtonClicked()
{
	//Check if button works
	UE_LOG(LogTemp, Warning, TEXT("Our button is working!"));

	// Initialize a topic
	ManualControlTopic = NewObject<UTopic>(UTopic::StaticClass());
	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());

	TopicName17 = DroneSelected + "/manual_enable";
	UE_LOG(LogTemp, Warning, TEXT("%s"), *TopicName17);

	ManualControlTopic->Init(rosinst->ROSIntegrationCore, TopicName17, TEXT("std_msgs/String"));

	// (Optional) Advertise the topic
	ManualControlTopic->Advertise();

	// Publish a string to the topic
	TSharedPtr<ROSMessages::std_msgs::String> EnableMessage(new ROSMessages::std_msgs::String("Manual Enabled"));
	TSharedPtr<ROSMessages::std_msgs::String> DisableMessage(new ROSMessages::std_msgs::String("Manual Disabled"));

	//Publish message
	if (isManualControlTriggered == false)
	{
		ManualControlTopic->Publish(EnableMessage);
		UE_LOG(LogTemp, Warning, TEXT("Manual Enabled"));
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, FString::Printf(TEXT("Manual Enabled")));
		isManualControlTriggered = true;
	}
	else
	{
		ManualControlTopic->Publish(DisableMessage);
		UE_LOG(LogTemp, Warning, TEXT("Manual Disabled"));
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, FString::Printf(TEXT("Manual Disabled")));
		isManualControlTriggered = false;
	}

}
