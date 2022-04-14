// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "ROSIntegration/Public/std_msgs/String.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "offboard_widget.generated.h"

/**
 *
 */
UCLASS()
class ROS_VR_API Uoffboard_widget : public UUserWidget
{
	GENERATED_BODY()
		//Initializing widget
		virtual bool Initialize();

	//Creating a property of a widget and adding the button class
	UPROPERTY(meta = (BindWidget))
		class UButton* Offboard;

public:
	UPROPERTY()
		class UTopic* OffboardTopic;
public:
	/*UPROPERTY()
		class UTopic* DroneSelectionTopic3;*/

		//Creating reference to function for button
	UFUNCTION()
		void StartButtonClicked();
public:
	// Called every frame
	//virtual void NativeTick(const FGeometry& MyGeometry, float DeltaTime);
};

