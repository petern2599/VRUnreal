// Fill out your copyright notice in the Description page of Project Settings.


#include "LandingPadsNavigation.h"
#include "Components/SpinBox.h"
#include "Components/Button.h"

FVector LandingPadPosition;
bool isLPTriggered;
extern FString DroneSelected;

bool ULandingPadsNavigation::Initialize()
{
	Super::Initialize();
	Select_Button->OnClicked.AddDynamic(this, &ULandingPadsNavigation::SelectButtonClicked);
	return true;
}

void ULandingPadsNavigation::SelectButtonClicked()
{
	UE_LOG(LogTemp, Warning, TEXT("Our button is working!"));
	// Initialize a topic
	LandingPadTopic = NewObject<UTopic>(UTopic::StaticClass());
	EnableLPTopic = NewObject<UTopic>(UTopic::StaticClass());
	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
	FString TopicName = "/landingpad_position";
	FString EnableTopicName = DroneSelected + "/lp_enable";
	LandingPadTopic->Init(rosinst->ROSIntegrationCore, TopicName, TEXT("geometry_msgs/Vector3"));
	EnableLPTopic->Init(rosinst->ROSIntegrationCore, EnableTopicName, TEXT("std_msgs/String"));
	// (Optional) Advertise the topic
	LandingPadTopic->Advertise();
	EnableLPTopic->Advertise();

	int LandingPadInt = LandingPadBox->GetValue();
	switch (LandingPadInt)
	{
	case 0:
		LandingPadPosition = FVector(26.9, -27.2, 5);
		break;
	case 1:
		LandingPadPosition = FVector(26.9, -23.2, 5);
		break;
	case 2:
		LandingPadPosition = FVector(22.9, -27.2, 5);
		break;
	case 3:
		LandingPadPosition = FVector(22.9, -23.2, 5);
		break;
	case 4:
		LandingPadPosition = FVector(26.9, 23, 5);
		break;
	case 5:
		LandingPadPosition = FVector(26.9, 27, 5);
		break;
	case 6:
		LandingPadPosition = FVector(22.9, 23, 5);
		break;
	case 7:
		LandingPadPosition = FVector(22.9, 27, 5);
		break;

	}


	// Publish a string to the topic
	TSharedPtr<ROSMessages::geometry_msgs::Vector3> LandingPadPositionMessage(new ROSMessages::geometry_msgs::Vector3(LandingPadPosition));
	TSharedPtr<ROSMessages::std_msgs::String> EnableMessage(new ROSMessages::std_msgs::String("LP Enabled"));
	TSharedPtr<ROSMessages::std_msgs::String> DisableMessage(new ROSMessages::std_msgs::String("LP Disabled"));
	if (isLPTriggered == false)
	{
		LandingPadTopic->Publish(LandingPadPositionMessage);
		EnableLPTopic->Publish(EnableMessage);
		isLPTriggered = true;
		UE_LOG(LogTemp, Warning, TEXT("Sending LP Position"));
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, FString::Printf(TEXT("Sending LP Position")));
	}
	else
	{
		EnableLPTopic->Publish(DisableMessage);
		isLPTriggered = false;
		UE_LOG(LogTemp, Warning, TEXT("Disabling LP"));
		GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, FString::Printf(TEXT("Disabling LP")));
	}

}


