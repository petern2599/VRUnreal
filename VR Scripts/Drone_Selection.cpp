// Fill out your copyright notice in the Description page of Project Settings.

#include "Drone_Selection.h"
#include "Components/SpinBox.h"
#include "Components/Button.h"

FString DroneSelected;
int DroneNumberInt;

bool UDrone_Selection::Initialize()
{
	Super::Initialize();
	SelectionButton->OnClicked.AddDynamic(this, &UDrone_Selection::StartButtonClicked);
	return true;
}

void UDrone_Selection::StartButtonClicked()
{

	// Initialize a topic
	DroneSelectionTopic = NewObject<UTopic>(UTopic::StaticClass());
	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
	FString TopicName = "/drone_selected";
	DroneSelectionTopic->Init(rosinst->ROSIntegrationCore, TopicName, TEXT("std_msgs/String"));

	// (Optional) Advertise the topic
	DroneSelectionTopic->Advertise();

	DroneNumberInt = DroneBox->GetValue();
	FString DroneNumberStr = FString::FromInt(DroneNumberInt);
	FString DroneName = "/uav";
	FString DroneAddress = DroneName + DroneNumberStr;
	DroneSelected = DroneName + DroneNumberStr;
	//GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Blue, FString::Printf(TEXT("Drone Location: %s"), *DroneAddress));

	// Publish a string to the topic
	TSharedPtr<ROSMessages::std_msgs::String> SelectedDroneMessage(new ROSMessages::std_msgs::String(DroneSelected));

	//Publish message
	DroneSelectionTopic->Publish(SelectedDroneMessage);
	UE_LOG(LogTemp, Warning, TEXT("%s"), *DroneSelected);


}