// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "ROSIntegration/Public/std_msgs/String.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "Drone_Selection.generated.h"

/**
 *
 */
UCLASS()
class ROS_VR_API UDrone_Selection : public UUserWidget
{
	GENERATED_BODY()
		//Initializing widget
		virtual bool Initialize();

	//Creating a property of a widget and adding the button class
	UPROPERTY(meta = (BindWidget))
		class UButton* SelectionButton;

	//Creating a property of a widget and adding the button class
	UPROPERTY(meta = (BindWidget))
		class USpinBox* DroneBox;
public:
	UPROPERTY()
		class UTopic* DroneSelectionTopic;

	UFUNCTION()
		void StartButtonClicked();

};



