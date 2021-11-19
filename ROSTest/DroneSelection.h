// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "ROSIntegration/Public/std_msgs/String.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "DroneSelection.generated.h"

/**
 * 
 */
UCLASS()
class ROSTEST_API UDroneSelection : public UUserWidget
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
