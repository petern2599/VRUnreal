// Fill out your copyright notice in the Description page of Project Settings.


#include "DroneSelection.h"
#include "Components/SpinBox.h"
#include "Components/Button.h"


bool UDroneSelection::Initialize()
{
	Super::Initialize();
	SelectionButton->OnClicked.AddDynamic(this, &UDroneSelection::StartButtonClicked);
	return true;
}

void UDroneSelection::StartButtonClicked()
{

	// Initialize a topic
	DroneSelectionTopic = NewObject<UTopic>(UTopic::StaticClass());
	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
	FString TopicName = "/drone_selected";
	DroneSelectionTopic->Init(rosinst->ROSIntegrationCore, TopicName, TEXT("std_msgs/String"));

	// (Optional) Advertise the topic
	DroneSelectionTopic->Advertise();

	int DroneNumberInt = DroneBox->GetValue();
	FString DroneNumberStr = FString::FromInt(DroneNumberInt);
	FString DroneName = "/uav";
	FString DroneSelected = DroneName + DroneNumberStr;

	// Publish a string to the topic
	TSharedPtr<ROSMessages::std_msgs::String> SelectedDroneMessage(new ROSMessages::std_msgs::String(DroneSelected));

	//Publish message
	DroneSelectionTopic->Publish(SelectedDroneMessage);
	UE_LOG(LogTemp, Warning, TEXT("%s"), *DroneSelected);

	
}